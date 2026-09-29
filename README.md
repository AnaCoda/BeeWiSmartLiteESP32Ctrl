# BeeWi SmartLite ESP32 Remote

A physical remote for BeeWi SmartLite Bluetooth bulbs, made from an ESP32-C3 SuperMini and the buttons and scroll wheel out of an old mouse shell. It talks to the bulbs directly over BLE, so no phone, app or computer needs to be on.

I built it for my own setup of two lamps that I want to control in sync, should be easy to change for other cases and controls.

Companion to [BeeWiSmartLiteWinCTRL](https://github.com/AnaCoda/BeeWiSmartLiteWinCTRL), the Windows tray app for the same bulbs.

## Controls

| Input | Does |
|---|---|
| Left click | Toggle bulb 1 |
| Right click | Toggle bulb 2 |
| Scroll wheel | Brightness of both bulbs |
| Wheel click | Switch the scroll wheel between brightness and white warmth |

The blue LED on the board is on while both bulbs are connected (not that I can see it inside the mouse haha).

## Hardware

- ESP32-C3 SuperMini
- An old mouse: the three button switches and the scroll wheel encoder, wired to the board (the mouse's own chip isn't used)

Everything switches to ground, using the ESP's internal pullups:

| Mouse part | GPIO |
|---|---|
| Wheel encoder A | 20 |
| Wheel encoder B | 3 |
| Left button | 1 |
| Right button | 4 |
| Wheel click | 0 |
| Common | G |

## Setup

1. Install [PlatformIO](https://platformio.org/) (the VS Code extension is easiest).
2. Copy `include/secrets.example.h` to `include/secrets.h` and put in your bulbs' mac addresses
3. Build and upload.

The bulbs only accept one connection at a time, so make sure no other device is connected to them first.

## Making it your own

Most things to change are near the top of `src/main.cpp`:

- **Pins:** the `#define`s at the top.
- **Bulbs:** `secrets.h`, and the `bulbs[]` list in `main.cpp`.
- **What each button does:** the `buttons[]` table, which maps each pin to a function like `onLeft`.
- **Wheel feel:** `ENC_STEPS_PER_DETENT`. Try 2 or 4 if one click moves too far or not at all.

`include/beewi_protocol.h` has all the bulb commands (on/off, brightness, warmth, RGB color, white), so adding things like color presets is a few lines.

## Credits

The BeeWi Bluetooth protocol was reverse-engineered by others: [BeewiPy](https://github.com/delkk0/BeewiPy) by delkk0, [light.beewi](https://github.com/bbo76/light.beewi) by bbo76, and this [Raspberry Pi forum thread](https://forums.raspberrypi.com/viewtopic.php?t=117729).
