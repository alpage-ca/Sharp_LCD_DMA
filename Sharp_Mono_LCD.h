/*********************************************************************
  This is a modified Adafruit library for the monochrome SHARP Memory Displays.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

    Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/1393

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, check license.txt for more information

 *********************************************************************

  Modified by John Schnurr, Sept 2016. Modified to use the Hardware SPI Pins.
  Faster frames can be achieved using the SPI pins.
  Verified to work on a Mega, Due, and Zero.

  The Hardware pins required are:
                     MOSI              SCK                SS
  Arduino Mega:     51 (or ICSP 4)    52 (or ICSP 3)     Any Pin
  Arduino Due:      ICSP 4            ICSP 3             Any Pin
  Arduino Mega:     ICSP 4            ICSP 3             Any Pin

  You may need to adjust the SPI Clock Divisor to get the fastet operation.
  Although the Display is rated for 1Mhz, I was able to use a divisor of 8
  on the Due (12 MHz) and a divisor of 6 on the Zero (8 MHz).

  You can change the SHARPMEM_LCDWIDTH and SHARPMEM_LCDHEIGHT in
  Sharp_Mono_LCD.h for the size of display you are using.

*********************************************************************/
/*
Modified by Alan Page to use PWM to generate VCOM signal at suitably low frequency.
Use DMA and double buffering to allow 24 FPS update without bogging down the CPU with SPI transfers and minimal flickering.
Certain drawing operations sped up with custom implementations.

*/

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Adafruit_GFX.h>
#ifdef __AVR
#include <avr/pgmspace.h>
#elif defined(ESP8266)
#include <pgmspace.h>
#endif
#include <EventResponder.h>

#define VCOM_PIN  9
// LCD Dimensions
#define SHARPMEM_LCDWIDTH       (144)
#define SHARPMEM_LCD_DMAWIDTH    (144+16) // add bytes so we can send entire buffer using DMA in a single operation.
#define SHARPMEM_LCDHEIGHT      (168)
#define SHARPMEM_CMD_PAD        (1)
#define SHARPMEM_DMA_BUFFER_SIZE  (SHARPMEM_CMD_PAD+((SHARPMEM_LCD_DMAWIDTH * SHARPMEM_LCDHEIGHT) / 8))
#define SHARPMEM_LINESPAN (SHARPMEM_LCD_DMAWIDTH/8)

class Sharp_Mono_LCD : public Adafruit_GFX {
  public:
    Sharp_Mono_LCD(uint8_t ss);
    void begin(void);
    inline void writePixel(int16_t x, int16_t y, uint16_t color)
    {
      drawPixel(x,y,color);
    }
    bool check(int tick,int rot); // for code debugging.
    // some fast overrides.
    void writeFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color); 
    void writeFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
    void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
    void drawPixel(int16_t x, int16_t y, uint16_t color);
    void clearDisplay();
    void clearBuffer();
    void refresh(void);
    void scroll_left();
    static uint8_t _ss;
    static volatile bool need_refresh; // flag set to notify that the buffer has been modified.
  private:
    void doLine(int16_t x1, int16_t y1, int16_t x2,int16_t y2, uint16_t color);
    static void internal_refresh();
    static EventResponder ssEvent;
};
