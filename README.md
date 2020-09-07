# uGUIv3.0-demo-STM32G474RE
demo of the uGUI framework on the STM32G474RE

https://www.st.com/en/microcontrollers-microprocessors/stm32g474re.html

This demo combines code from:

https://github.com/achimdoebler/UGUI

https://embeddedlightning.com/download/stm32f429-discovery-2/

https://sourceforge.net/projects/nrf52-ssd1331/

to produce a working demo of uGUI 3.0

Here is the connection matrix:

| signal name	| pin	| connector |
| --- | --- | --- |
| spi2 mosi	  | pb15	  | ssd1331_data
| ssd1331 DC	| pb14	  | ssd1331_dc
| spi2 sck	  | pb13	  | ssd1331_clk
| spi2 nss	  | pb12	  | ssd1331_cs
| ssd1331 RES	| pb11	  | ssd1331_res


It uses FreeRTOS

the uGUI code writes into a frame buffer, which is then pushed to the display via SPI 30 frames/sec.

Please note that a lot of cleanup is required, there are lots of unused functions due to combining to separate libraries.  I left some of that in, as there are other ways to communicate with the SSD1331, rather than just slamming the full buffer into the chip 30 times a second.

More info on uGUI here:

https://embeddedlightning.com/ugui/

I changed some of the text rendering so it would chop off the end of text rather than refuse to render it if it's too long.

This utilty works well to create arbitrary bitmaps that can be integrated into this project:

http://dot2pic.com/

Data sheet for ssd1331 is here:

https://cdn-shop.adafruit.com/datasheets/SSD1331_1.2.pdf


p.s. I found this repo too late, looks interesting:

https://github.com/0x3333/UGUI

