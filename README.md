# Example to use i2s to drive a led display

This is example code to drive one of the common 64x32-pixel RGB LED
screen. It illustrates the parallel output mode of the I2S peripheral.

See main/app_main.c for more information and how to hook up a display.

This is PRELIMINARY CODE and Espressif gives no support on it.

Originally published on:
https://esp32.com/viewtopic.php?t=3188

Modified to match pin-out of Espirgbani Rev 1 board:
https://github.com/yetifrisstlama/Espirgbani

# Building
  * install [PlatformIO](https://platformio.org/)
  * disconnect LED panel
  * connect USB-TTY cable
  * hold the `FLASH` button

then ...

```bash
$ pio run -t upload -t monitor
```
