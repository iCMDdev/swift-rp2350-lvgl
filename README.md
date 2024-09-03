<img src="https://github.com/user-attachments/assets/afa7fc1e-6f72-4f2d-9e0d-177b3f3072d5" width="30%" />

# swift-rp2350-lvgl-st7789

A simple example of running [Swift Embedded (currently experimental)](https://github.com/apple/swift-embedded-examples), porting [LVGL 9](https://lvgl.io) on a [Raspberry Pi Pico 2 (RP2350)](https://www.raspberrypi.com/products/raspberry-pi-pico-2/) with a [ST7789 display (Pimoroni Pico Display Pack)](https://shop.pimoroni.com/products/pico-display-pack?variant=32368664215635). 

This project uses the Pico 2.0 (or later?) SDK.
The Pico 2 communicates with the display via HSTX (High-Speed Serial Transmit), which is a new peripheral introduced in the RP2350 that facilitates faster SPI communications (TX only).

## How to build?

You should follow the instructions provided in Apple's [Swift Embedded Examples](https://github.com/apple/swift-embedded-examples/tree/main/pico-blink-sdk) repo. Maybe first try building the blink example, then similarly, use the same CMake commands here.

## Some potential questions, answered:

> Why Swift?

[The modern language features and safety compared to C/C++ (while still being fast)](https://www.swift.org/about/). And technically it's also harder to reverse engineer, but one shouldn't rely just on security through obscurity.

It also has good interoperability with C/C++.

> Why not Rust?

I wanted to test the new Swift Embedded language features. It is true that Rust also provides similar safety benefits.

> Why is there so little Swift code in this repo, compared to C?

This is because the Pico display communication and LVGL porting / display configuration was made in C (following their documentation). I might port them to Swift later, but either way, everything is still controlled from within `Main.swift`. One can take this further and build a project / device and its main logic (UI, other peripherals etc.) upon this example.

