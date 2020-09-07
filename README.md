# uGUIv3.0-demo-STM32G474RE
demo of the uGUI framework on the STM32G474RE

This demo combines code from:

https://github.com/achimdoebler/UGUI

https://embeddedlightning.com/download/stm32f429-discovery-2/

https://sourceforge.net/projects/nrf52-ssd1331/

to produce a working demo of uGUI 3.0

It uses FreeRTOS

the uGUI code writes into a frame buffer, which is then pushed to the display via SPI 30 frames/sec.

More info on uGUI here:

https://embeddedlightning.com/ugui/

I changed some of the text rendering so it would chop off the end of text rather than refuse to render it if it's too long.
