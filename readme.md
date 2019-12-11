# PCB3-AVR
This project contains alternative code for the [Christmas Tree Shaped PCB  "PCB3"](https://github.com/witnessmenow/PCB3) by [Brian Lough](https://www.youtube.com/channel/UCezJOfu7OtqGzd5xrP3q6WA). Go check out his awesome projects!

The PCB3 is really a great physical design, but it comes only with few modes / animations. So I tried to reprogram it. This is the result of only two days trying and the modes are, so far, also limited. There might be more in the future.

## Button/Switch
**Short press** on MODE Button will **cycle** through the available **modes**. 

**Long press** on MODE Button will **slow down** the current **animation** by one step each time the star flashes bright.

**Off/on** will reset the mode. No program space left for eeprom...

## Modes

First four modes feature one LED lit bright, cycling throuh in either regular order or random order, with either all other LEDs off or lit dim.

Next four modes feature one LED flashing bright, cycling throuh in either regular order or random order, with either all other LEDs off or lit dim.

Following two modes feature random LEDs gently fading in and out (pulsing) with either going out completly or again, all lit dim.

## How to flash

**Do this on your own risk! You might brick your PCB3!**

Download the HEX-file from [releases](https://github.com/xsrf/pcb3-avr/releases) tab and flash it e.g. with `avrdude.exe -c avrisp -P COM3 -b 19200 -p t13 -v -U flash:w:firmware.hex` using an `Arduino as ISP` on `COM3`.
You may also compile the project yourself. I've used [Visual Studio Code](https://code.visualstudio.com/) with PlatformIO Extension.
You may find `avrdude` under `C:/Users/<user>/.platformio/packages/tool-avrdude`.
