# swift-rp2350-lvgl
A simple example of running [Swift Embedded](https://github.com/apple/swift-embedded-examples), porting [LVGL 9](https://lvgl.io) on a [Raspberry Pi Pico 2 (RP2350)](https://www.raspberrypi.com/products/raspberry-pi-pico-2/) with a [ST7789 display (Pimoroni Pico Display Pack)](https://shop.pimoroni.com/products/pico-display-pack?variant=32368664215635). 

This project uses the Pico SDK.
The Pico 2 communicates with the display via HSTX (High-Speed Serial Transmit), which is a new peripheral introduced in the RP2350 that facilitates faster SPI communications (TX only).
