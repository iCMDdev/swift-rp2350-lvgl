// The original file can be found in the pico-examples repository: 
// https://github.com/raspberrypi/pico-examples/blob/master/hstx/spi_lcd/hstx_spi_lcd.c.
// It was modified by iCMDdev (icmd.tech) in order to connect 
// the Pimoroni Display Pack and add some LVGL functions.

/*
 Copyright (c) 2024 Raspberry Pi (Trading) Ltd.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
   disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
   disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
*/

// Drive a ST7789 SPI LCD using the HSTX. The SPI clock rate is fully
// independent of (and can be faster than) the system clock.

// You'll need an LCD module for this example. It was tested with: WaveShare
// 1.3 inch ST7789 module. Wire up the signals as per PIN_xxx defines below,
// and don't forget to connect GND and VCC to GND/3V3 on your board!
//
// Theory of operation: Each 32-bit HSTX record contains 3 x 8-bit fields:
//
// 27:20  CSn x8    (noninverted CSn pin)
// 17:10  !DC x 8   (inverted DC pin)
//  7: 0  data bits (DIN pin)
//
// SCK is driven by the HSTX clock generator. We do issue extra clocks whilst
// CSn is high, but this should be ignored by the display. Packing the
// control lines in the HSTX FIFO records makes it easy to drive them in sync
// with SCK without having to reach around and do manual GPIO wiggling.

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/resets.h"
#include "hardware/structs/clocks.h"
#include "hardware/structs/hstx_ctrl.h"
#include "hardware/structs/hstx_fifo.h"
#include "lvgl.h"
#include "hstx_spi_lcd.h"
#include "demos/lv_demos.h"

// These can be any permutation of HSTX-capable pins:
#define PIN_DIN   19
#define PIN_SCK   18
#define PIN_CS    17
#define PIN_DC    16
// These can be any pin:
#define PIN_RESET 21
#define PIN_BL    20

#define FIRST_HSTX_PIN 12
#if   PIN_DIN < FIRST_HSTX_PIN || PIN_DIN >= FIRST_HSTX_PIN + 8
#error "Must be an HSTX-capable pin: DIN"
#elif PIN_SCK < FIRST_HSTX_PIN || PIN_SCK >= FIRST_HSTX_PIN + 8
#error "Must be an HSTX-capable pin: SCK"
#elif PIN_CS  < FIRST_HSTX_PIN || PIN_CS  >= FIRST_HSTX_PIN + 8
#error "Must be an HSTX-capable pin: CS"
#elif PIN_DC  < FIRST_HSTX_PIN || PIN_DC  >= FIRST_HSTX_PIN + 8
#error "Must be an HSTX-capable pin: DC"
#endif

static inline void hstx_put_word(uint32_t data) {
	while (hstx_fifo_hw->stat & HSTX_FIFO_STAT_FULL_BITS)
		;
	hstx_fifo_hw->fifo = data;
}

static inline void lcd_put_dc_cs_data(bool dc, bool csn, uint8_t data) {
	hstx_put_word(
		(uint32_t)data |
		(csn ? 0x0ff00000u : 0x00000000u) |
		// Note DC gets inverted inside of HSTX:
		(dc  ? 0x00000000u : 0x0003fc00u)
	);
}

static inline void lcd_start_cmd(uint8_t cmd) {
	lcd_put_dc_cs_data(false, true, 0);
	lcd_put_dc_cs_data(false, false, cmd);
}

static inline void lcd_put_data(uint32_t data) {
	lcd_put_dc_cs_data(true, false, data);
}

// Format: cmd length (including cmd byte), post delay in units of 5 ms, then cmd payload
// Note the delays have been shortened a little
#define SCREEN_WIDTH 240
#define WIDTH_OFFSET 40
#define SCREEN_HEIGHT 135
#define HEIGHT_OFFSET 53
static const uint8_t st7789_init_seq[] = {
        1, 20, 0x01,                        // Software reset
        1, 10, 0x11,                        // Exit sleep mode
        2, 2, 0x3a, 0x55,                   // Set colour mode to 16 bit
        2, 0, 0x36, 0x00,                   // Set MADCTL: row then column, refresh is bottom to top ????
        5, 0, 0x2a, WIDTH_OFFSET >> 8, WIDTH_OFFSET & 0xff,             // CASET: column addresses
            (SCREEN_WIDTH+WIDTH_OFFSET-1) >> 8, (SCREEN_WIDTH+WIDTH_OFFSET-1) & 0xff,
        5, 0, 0x2b, HEIGHT_OFFSET >> 8, HEIGHT_OFFSET & 0xff,             // RASET: row addresses
            (SCREEN_HEIGHT+HEIGHT_OFFSET) >> 8, (SCREEN_HEIGHT+HEIGHT_OFFSET) & 0xff,
        1, 2, 0x21,                         // Inversion on, then 10 ms delay (supposedly a hack?)
        1, 2, 0x13,                         // Normal display on, then 10 ms delay
        1, 2, 0x29,                         // Main screen turn on, then wait 500 ms
        0                                   // Terminate list
};

static inline void lcd_write_cmd(const uint8_t *cmd, size_t count) {
    lcd_start_cmd(*cmd++);
    if (count >= 2) {
        for (size_t i = 0; i < count - 1; ++i) {
            lcd_put_data(*cmd++);
        }
    }
}

static inline void lcd_init(const uint8_t *init_seq) {
    const uint8_t *cmd = init_seq;
    while (*cmd) {
        lcd_write_cmd(cmd + 2, *cmd);
        sleep_ms(*(cmd + 1) * 5);
        cmd += *cmd + 2;
    }
}

static void lcd_set_memory_area(uint16_t xstart, uint16_t xend, uint16_t ystart, uint16_t yend) {
    uint8_t data[4];

    // Set column address (CASET)
    lcd_start_cmd(0x2A);
    data[0] = (xstart + WIDTH_OFFSET) >> 8;
    data[1] = (xstart + WIDTH_OFFSET) & 0xFF;
    data[2] = (xend + WIDTH_OFFSET) >> 8;
    data[3] = (xend + WIDTH_OFFSET) & 0xFF;
    lcd_write_cmd(data, 4);

    // Set row address (RASET)
    lcd_start_cmd(0x2B);
    data[0] = (ystart + HEIGHT_OFFSET) >> 8;
    data[1] = (ystart + HEIGHT_OFFSET) & 0xFF;
    data[2] = (yend + HEIGHT_OFFSET) >> 8;
    data[3] = (yend + HEIGHT_OFFSET) & 0xFF;
    lcd_write_cmd(data, 4);
}


static inline void lcd_start_pixels(void) {
    uint8_t cmd = 0x2c; // RAMWR
    lcd_write_cmd(&cmd, 1);
}

static void display_init(void) {
    // Switch HSTX to USB PLL (presumably 48 MHz) because clk_sys is probably
    // running a bit too fast for this example -- 48 MHz means 48 Mbps on
    // PIN_DIN. Need to reset around clock mux change, as the AUX mux can
    // introduce short clock pulses:
    reset_block(RESETS_RESET_HSTX_BITS);
    hw_write_masked(
        &clocks_hw->clk[clk_hstx].ctrl,
        CLOCKS_CLK_HSTX_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB << CLOCKS_CLK_HSTX_CTRL_AUXSRC_LSB,
        CLOCKS_CLK_HSTX_CTRL_AUXSRC_BITS
    );
    unreset_block_wait(RESETS_RESET_HSTX_BITS);

    gpio_init(PIN_RESET);
    gpio_init(PIN_BL);
    gpio_set_dir(PIN_RESET, GPIO_OUT);
    gpio_set_dir(PIN_BL, GPIO_OUT);
    gpio_put(PIN_RESET, 1);
    gpio_put(PIN_BL, 1);

    hstx_ctrl_hw->bit[PIN_SCK - FIRST_HSTX_PIN] =
        HSTX_CTRL_BIT0_CLK_BITS;

    hstx_ctrl_hw->bit[PIN_DIN - FIRST_HSTX_PIN] =
        (7u << HSTX_CTRL_BIT0_SEL_P_LSB) |
        (7u << HSTX_CTRL_BIT0_SEL_N_LSB);

    hstx_ctrl_hw->bit[PIN_CS - FIRST_HSTX_PIN] =
        (27u << HSTX_CTRL_BIT0_SEL_P_LSB) |
        (27u << HSTX_CTRL_BIT0_SEL_N_LSB);

    hstx_ctrl_hw->bit[PIN_DC - FIRST_HSTX_PIN] =
        (17u << HSTX_CTRL_BIT0_SEL_P_LSB) |
        (17u << HSTX_CTRL_BIT0_SEL_N_LSB) |
        (HSTX_CTRL_BIT0_INV_BITS);

    // We have packed 8-bit fields, so shift left 1 bit/cycle, 8 times.
    hstx_ctrl_hw->csr =
        HSTX_CTRL_CSR_EN_BITS |
        (31u << HSTX_CTRL_CSR_SHIFT_LSB) |
        (8u << HSTX_CTRL_CSR_N_SHIFTS_LSB) |
        (1u << HSTX_CTRL_CSR_CLKDIV_LSB);

    gpio_set_function(PIN_SCK, 0/*GPIO_FUNC_HSTX*/);
    gpio_set_function(PIN_DIN, 0/*GPIO_FUNC_HSTX*/);
    gpio_set_function(PIN_CS,  0/*GPIO_FUNC_HSTX*/);
    gpio_set_function(PIN_DC,  0/*GPIO_FUNC_HSTX*/);

    lcd_init(st7789_init_seq);
}

void lcd_set_rotation(uint8_t rotation) {
    lcd_start_cmd(0x36);  // MADCTL command - rotate the display 90 deg
    lcd_put_data(rotation);
}

// End of Raspberry Pi (Trading) Ltd. code (with modifications)

// The following lines of code represent modified source code from the LVGL library.
// These represent the necessary configurations / callbacks to port LVGL to the Pico 2 &
// ST7789 display.
//
// Copyright (c) 2021 LVGL Kft

/*
MIT licence
Copyright (c) 2021 LVGL Kft

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/

static void disp_flush(lv_display_t * disp_drv, const lv_area_t * ar, uint8_t * px_map)
{
    int32_t x;
    int32_t y;

    lv_area_t area = *ar;
    if(area.x1 < 0) area.x1 = 0;
    if(area.y1 < 0) area.y1 = 0;
    if(area.x2 > SCREEN_WIDTH) area.x2 = SCREEN_WIDTH;
    if(area.y2 > SCREEN_HEIGHT) area.y1 = SCREEN_HEIGHT;

    lcd_set_memory_area(area.x1, area.x2, area.y1, area.y2);

    lcd_start_pixels();
    
    for(y = area.y1; y <= area.y2; y++) {
        for(x = area.x1; x <= area.x2; x++) {
            lcd_put_data(px_map[2*((area.x2-area.x1+1)*y+x)]);
            lcd_put_data(px_map[2*((area.x2-area.x1+1)*y+x)+1]);
        }
    }

    // Inform LVGL that the flushing is done
    lv_disp_flush_ready(disp_drv);
}

#define BYTE_PER_PIXEL (LV_COLOR_FORMAT_GET_SIZE(LV_COLOR_FORMAT_RGB565))
#define MY_DISP_HOR_RES 240
#define MY_DISP_VER_RES 135

// Pimoroni Pico Display Pack Buttons
#define UP_PIN 14
#define DOWN_PIN 15  
#define LEFT_PIN 12
#define RIGHT_PIN 13

void init_buttons() {
    gpio_init(UP_PIN);                     // Initialize the GPIO pin
    gpio_set_dir(UP_PIN, GPIO_IN);         // Set the GPIO direction as input
    gpio_pull_up(UP_PIN);

    gpio_init(DOWN_PIN);                   
    gpio_set_dir(DOWN_PIN, GPIO_IN);       
    gpio_pull_up(DOWN_PIN);

    gpio_init(LEFT_PIN);                   
    gpio_set_dir(LEFT_PIN, GPIO_IN);       
    gpio_pull_up(LEFT_PIN);                

    gpio_init(RIGHT_PIN);                  
    gpio_set_dir(RIGHT_PIN, GPIO_IN);      
    gpio_pull_up(RIGHT_PIN);       
}

void btn_callback(uint gpio, uint32_t events) {
    // Handle button press (interrupt) event
    if (gpio == UP_PIN) {
        if (events & GPIO_IRQ_EDGE_FALL) {
            // Handle Button 1 press (falling edge)
            printf("UP Pressed\n");
            scrollUP = true;
        } else if (events & GPIO_IRQ_EDGE_RISE) {
            // Handle Button 1 release (rising edge)
            printf("UP Released\n");
            scrollUP = false;
        }
    } else if (gpio == DOWN_PIN) {
        if (events & GPIO_IRQ_EDGE_FALL) {
            // Handle Button 2 press (falling edge)
            printf("DOWN Pressed\n");
            scrollDOWN = true;
        } else if (events & GPIO_IRQ_EDGE_RISE) {
            // Handle Button 2 release (rising edge)
            printf("DOWN Released\n");
            scrollDOWN = false;
        }
    } else if (gpio == LEFT_PIN) {
        if (events & GPIO_IRQ_EDGE_FALL) {
            // Handle Button 2 press (falling edge)
            printf("LEFT Pressed\n");
            scrollLEFT = true;
        } else if (events & GPIO_IRQ_EDGE_RISE) {
            // Handle Button 2 release (rising edge)
            printf("LEFT Released\n");
            scrollLEFT = false;
        }
    } else if (gpio == RIGHT_PIN) {
        if (events & GPIO_IRQ_EDGE_FALL) {
            // Handle Button 2 press (falling edge)
            printf("RIGHT Pressed\n");
            scrollRIGHT = true;
        } else if (events & GPIO_IRQ_EDGE_RISE) {
            // Handle Button 2 release (rising edge)
            printf("RIGHT Released\n");
            scrollRIGHT = false;
        }
    } 
}

static uint32_t ms_since_boot() {
    return (uint32_t)to_ms_since_boot(get_absolute_time());
}

static uint32_t keypad_get_key(void)
{
    if (scrollDOWN && scrollUP) return 3;
    if (scrollDOWN) return 1;
    if (scrollUP) return 2;
    if (scrollLEFT) return 4;
    if (scrollRIGHT) return 5;
    return 0;
}

static void keypad_read(lv_indev_t * indev_drv, lv_indev_data_t * data)
{
    static uint32_t last_key = 0;

    /*Get whether the a key is pressed and save the pressed key*/
    uint32_t act_key = keypad_get_key();
    if(act_key != 0) {
        data->state = LV_INDEV_STATE_PRESSED;
        //printf("%lu\n", act_key);
        /*Translate the keys to LVGL control characters according to your key definitions*/
        switch(act_key) {
            case 1:
                act_key = LV_KEY_LEFT;
                break;
            case 2:
                act_key = LV_KEY_RIGHT;
                break;
            case 3:
                act_key = LV_KEY_ENTER;
                break;
            case 4:
                act_key = LV_KEY_PREV;
                break;
            case 5:
                act_key = LV_KEY_NEXT;
                break;
        }

        last_key = act_key;
    }
    else {
        data->state = LV_INDEV_STATE_RELEASED;
    }

    data->key = last_key;
}

static void graphics_init(void) {
    lv_init();
    display_init();
    stdio_init_all();

    lv_display_t * disp = lv_display_create(MY_DISP_HOR_RES, MY_DISP_VER_RES);
    lv_display_set_flush_cb(disp, disp_flush);

    LV_ATTRIBUTE_MEM_ALIGN
    static uint8_t buf_2_1[MY_DISP_HOR_RES * MY_DISP_HOR_RES * BYTE_PER_PIXEL];

    LV_ATTRIBUTE_MEM_ALIGN
    static uint8_t buf_2_2[MY_DISP_HOR_RES * MY_DISP_HOR_RES * BYTE_PER_PIXEL];
    lv_display_set_buffers(disp, buf_2_1, buf_2_2, sizeof(buf_2_1), LV_DISPLAY_RENDER_MODE_PARTIAL);

    lcd_set_rotation(0x60);

    lv_tick_set_cb(ms_since_boot);

    // input devices (keypad buttons)
    lv_indev_t * indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_KEYPAD);
    lv_indev_set_read_cb(indev, keypad_read);  
}

int show_demo() {
    graphics_init();
    init_buttons();
    gpio_set_irq_enabled_with_callback(UP_PIN, 
                                       GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, 
                                       true, 
                                       &btn_callback);
    gpio_set_irq_enabled_with_callback(DOWN_PIN, 
                                       GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, 
                                       true, 
                                       &btn_callback);
    gpio_set_irq_enabled_with_callback(LEFT_PIN, 
                                       GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, 
                                       true, 
                                       &btn_callback);
    gpio_set_irq_enabled_with_callback(RIGHT_PIN, 
                                       GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, 
                                       true, 
                                       &btn_callback);

    lv_demo_music();
}

// END of LVGL code

/*int display_main() {
    display_init();
    lcd_set_rotation(0x60);
    uint t = 0;
    while (true) {
        uint64_t current_milliseconds = to_ms_since_boot(get_absolute_time());
        printf("ms: %llu", current_milliseconds);
        lcd_start_pixels();
        uint8_t colors[] =;




        lcd_start_pixels();
        for(uint8_t y = 0; y < SCREEN_HEIGHT; y++) {
            for(uint8_t x = 0; x < SCREEN_WIDTH; x++) {
                lcd_put_data(colors[2*((SCREEN_WIDTH)*y+x)]);
                lcd_put_data(colors[2*((SCREEN_WIDTH)*y+x)+1]);
            }
        }
    }
}*/


