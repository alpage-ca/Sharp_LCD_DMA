# Sharp_LCD_DMA
Optimized Teensy code for Adafruit Sharp Memory Display Breakout 3502
Based on work by John Schurr, who based his work on the Limor Fried/Ladyada code for Adafruit Industries.

Doing frequent updates on the Sharp Mono LCD can be very time consuming as each update needs to be written to the display which can take up to 30 milliseconds. The main loop just sets a flag and an interval timer at 24 frames per second does the actual update. If the update flag is set then a DMA transfer is triggered to send the entire frame buffer to the display. An EventResponder is attached to the DMA transfer to turn off SS after the DMA transfer is complete. Double buffering of the display is done to reduce flickering.

The various buffers are expanded slightly so they can be sent to the display in a single DMA operation.

The VCOM signal needed by the Sharp Mono LCD is handled by PWM at low frequency.
The following functions are optimized for monochrome operation.

    void writeFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color); 
    void writeFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
    void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
    void drawPixel(int16_t x, int16_t y, uint16_t color);
