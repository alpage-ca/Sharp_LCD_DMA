

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

 You may need to adjust the SPI Clock Divisor to get the fastest reliable
 operation.
 Although the Display is rated for 1Mhz, I was able to use a divisor of 8 
 on the Due (12 MHz) and a divisor of 6 on the Zero (8 MHz).

 You can change the SHARPMEM_LCDWIDTH and SHARPMEM_LCDHEIGHT in 
 Sharp_Mono_LCD.h for the size of display you are using.

Modified by Alan Page to use PWM to generate VCOM signal at suitably low frequency.
Use DMA and double buffering to allow 24 FPS update without bogging down the CPU with SPI transfers and minimal flickering.
Certain drawing operations sped up with custom implementations.


*********************************************************************/

#include <SPI.h>
#include "Sharp_Mono_LCD.h"
#include <FrequencyTimer2.h>

#ifndef _swap_int16_t
#define _swap_int16_t(a, b) { int16_t t = a; a = b; b = t; }
#endif
#ifndef _swap_uint16_t
#define _swap_uint16_t(a, b) { uint16_t t = a; a = b; b = t; }
#endif

/**************************************************************************
    Adafruit Sharp Memory Display Connector
    -----------------------------------------------------------------------
    Pin   Function        Notes
    ===   ==============  ===============================
      1   VIN             3.3-5.0V (into LDO supply)
      2   3V3             3.3V out
      3   GND
      4   SCLK            SPI Hardware Serial Clock Pin
      5   MOSI            SPI Hardware Serial Data Input Pin
      6   CS              Serial Chip Select (can be any pin)
      9   EXTMODE         COM Inversion Select (Low = SW clock/serial)
      7   EXTCOMIN        External COM Inversion Signal
      8   DISP            Display On(High)/Off(Low)

 **************************************************************************/

#define SHARPMEM_BIT_WRITECMD   (0x01)
#define SHARPMEM_BIT_VCOM       (0x02)
#define SHARPMEM_BIT_CLEAR      (0x04)

/*
 * Double buffering used. All graphics operations are done in the sharpmem_buffer.
 * When ready to update the sharpmem_buffer is copied to the refresh_buffer and an update flag is set.
 * The SPI DMA sends the refresh buffer to the Sharp LCD.
*/
byte sharpmem_buffer[SHARPMEM_DMA_BUFFER_SIZE];
byte refresh_buffer[SHARPMEM_DMA_BUFFER_SIZE];
uint8_t Sharp_Mono_LCD::_ss = 10;
/* ************* */
/* CONSTRUCTORS  */
/* ************* */
Sharp_Mono_LCD::Sharp_Mono_LCD(uint8_t ss) :
  Adafruit_GFX(SHARPMEM_LCDWIDTH, SHARPMEM_LCDHEIGHT) {
  
  _ss = ss;
  digitalWrite(_ss, HIGH);
  pinMode(_ss, OUTPUT);
  pinMode(VCOM_PIN,OUTPUT);
  analogWriteFrequency(VCOM_PIN, 12); 
  analogWriteResolution(14);
  analogWrite(VCOM_PIN,1);
  // initialize memory buffer for single dma transfer.
  memset(sharpmem_buffer, 0xff,SHARPMEM_DMA_BUFFER_SIZE);
  sharpmem_buffer[0] = SHARPMEM_BIT_WRITECMD; // first byte of buffer
  byte *line = &sharpmem_buffer[SHARPMEM_CMD_PAD];
  for (int i = 0;i < SHARPMEM_LCDHEIGHT;i++)
  {
    line[0] = i+1;
    line[1+(SHARPMEM_LCDWIDTH/8)] = 0;
    line += SHARPMEM_LCD_DMAWIDTH/8;
  }
}

void Sharp_Mono_LCD::scroll_left()
{
  byte *start = &sharpmem_buffer[SHARPMEM_CMD_PAD+1];
  for (int i = 0;i < SHARPMEM_LCDHEIGHT;i++)
  {
    byte * line = start;
    byte temp = line[0];
    byte v = temp<<1;
    for (int j = 0;j < 17;j++)
    {
      temp = line[j+1];
      line[j] = v|temp>>7;
      v = temp<<1;
    }
    line[17] = v|1;
    start += SHARPMEM_LINESPAN;
  }
}

/*
 * for debugging the code I added code that would check for overwriting past the DMA buffer boundaries.
 * Not used for normal operations.
*/
bool Sharp_Mono_LCD::check(int tick,int rot)
{
  byte *line = &sharpmem_buffer[SHARPMEM_CMD_PAD];
  bool bad = false;
  for (int i = 0;i < SHARPMEM_LCDHEIGHT;i++)
  {
    if (line[0] != i+1)
    {
      Serial.print("Bad line0:");
      Serial.println(i);
      bad = true;
      line[0] = i+1;
    }
    if (line[1+(SHARPMEM_LCDWIDTH/8)] != 0)
    {
      Serial.print("bad end of line:");
      Serial.println(i);
      line[1+(SHARPMEM_LCDWIDTH/8)] = 0;
      bad = true;
    }
    line += SHARPMEM_LCD_DMAWIDTH/8;
  }
  if (bad)
  {
    Serial.print(tick);
    Serial.print(",");
    Serial.println(rot);
  }
  return bad;
}

void clearSS(EventResponderRef u)
{
  digitalWriteFast(Sharp_Mono_LCD::_ss, LOW);
}

void Sharp_Mono_LCD::begin() {
  SPI.setSCK(27);
#if defined(__SAM3X8E__)
  SPI.begin(_ss);
  SPI.setClockDivider(8); // may have to adjust this clock divisor for best operation
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(LSBFIRST);
#else
  SPI.begin();
  // sharp appears to tolerate up to a 4 MHZ clock even though the specs say 1 MHZ maximum.
  SPI.setClockDivider(SPI_CLOCK_DIV16);  // 1 MHZ is from data sheet, though 2 MHZ runs reliably.
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(LSBFIRST);
//  SPI.setCS(_ss);
  pinMode(_ss, OUTPUT);
  digitalWrite(_ss, LOW);
  ssEvent.attachInterrupt(clearSS);
#endif
  setRotation(0);
  clearDisplay();
  // using frequency timer 2 to free up an interval timer
  FrequencyTimer2::setPeriod(1000000/24); // 24 frames per second.
  FrequencyTimer2::setOnOverflow(internal_refresh);
  /*
   * From logic analyzer timings I know that the DMA transfer will take about 29 milliseconds, well under the 24 FPS rate (41.67 ms)
   * Could probably even go to 30 fps if you wanted.
  */
}

// 1<<n is a costly operation on AVR -- table usu. smaller & faster
static const uint8_t PROGMEM
set[] = {  1,  2,  4,  8,  16,  32,  64,  128 },
        clr[] = { (uint8_t)(~1), (uint8_t)(~2), (uint8_t)(~4), (uint8_t)(~8),
                  (uint8_t)(~16), (uint8_t)(~32), (uint8_t)(~64), (uint8_t)(~128) };

/**************************************************************************/
/*!
    @brief Draws a single pixel in image buffer

    @param[in]  x
                The x position (0 based)
    @param[in]  y
                The y position (0 based)
*/
/**************************************************************************/
void Sharp_Mono_LCD::drawPixel(int16_t x, int16_t y, uint16_t color)
{
  if ((x < 0) || (x >= _width) || (y < 0) || (y >= _height)) return;

  switch (rotation) {
    case 1: // 270
      _swap_int16_t(x, y);
      x = WIDTH  - 1 - x;
      break;
    case 2: // 180
      x = WIDTH  - 1 - x;
      y = HEIGHT - 1 - y;
      break;
    case 3: // 90
      _swap_int16_t(x, y);
      y = HEIGHT - 1 - y;
      break;
  }

  if (color) {
    sharpmem_buffer[1+SHARPMEM_CMD_PAD+((y * SHARPMEM_LCD_DMAWIDTH + x) / 8)] |= set[x & 7];
  } else {
    sharpmem_buffer[1+SHARPMEM_CMD_PAD+((y * SHARPMEM_LCD_DMAWIDTH + x) / 8)] &= clr[x & 7];
  }
}

void Sharp_Mono_LCD::writeFastVLine(int16_t x1, int16_t y1, int16_t h, uint16_t color)
{
  if (h <= 0)
  {
    Serial.print("Out of bounds on height:");
    Serial.println(h);
    return;
  }
  int16_t x2=0,y2=0;
  switch (rotation) {
    case 1: // 270
      x2 = WIDTH - y1-1;
      y1 = x1;
      x1 = x2-h+1;
      y2 = y1;
      break;
    case 2: // 180
      x2 = WIDTH  - 1 - x1;
      y2 = HEIGHT - 1 - y1;
      x1 = x2;
      y1 = y2-h+1;
      break;
    case 3: // 90
      y2 = HEIGHT - x1 -1;
      x1 = y1;
      y1 = y2;
      x2 = x1 + h-1;
      break;
    default:
      x2 = x1;
      y2 = y1+h-1;
      break;
  }
  doLine(x1,y1,x2,y2,color);
}

void Sharp_Mono_LCD::writeFastHLine(int16_t x1, int16_t y1, int16_t w, uint16_t color)
{
  if (w <= 0)
  {
    Serial.print("Out of bounds on width:");
    Serial.println(w);
    return;
  }
  int16_t x2=0,y2=0;
  switch (rotation) {
    case 1: // 270
      x2 = WIDTH - y1-1;
      y1 = x1;
      x1 = x2;
      y2 = y1 + w-1;
      break;
    case 2: // 180
      x2 = WIDTH  - 1 - x1;
      y2 = HEIGHT - 1 - y1;
      x1 = x2-w+1;
      y1 = y2;
      break;
    case 3: // 90
      y2 = HEIGHT - x1 -1;
      x1 = y1;
      y1 = y2 - w + 1;
      x2 = x1;
      break;
    default:
      x2 = x1+w-1;
      y2 = y1;
      break;
  }
  doLine(x1,y1,x2,y2,color);
}

void Sharp_Mono_LCD::doLine(int16_t x1, int16_t y1, int16_t x2,int16_t y2, uint16_t color)
{
  if ((x1 < 0) || (x1 >= WIDTH) || (y1 < 0) || (y1 >= HEIGHT) || (x2 < 0) || (y2 < 0) || (x2 >= WIDTH) ||(y2 >= HEIGHT))
  {
    Serial.println("Out of bounds on doLine");
    return;
  }
  byte *lpStart = &sharpmem_buffer[1+SHARPMEM_CMD_PAD+((y1 * SHARPMEM_LCD_DMAWIDTH + x1) / 8)];
  byte value = (byte)((color)?0xff:0x00);
  int16_t j;
  if(x1 == x2)
  {
    byte temp = (color)?set[x1 & 7]:clr[x1&7];
    if (value)
    {
      for (j = y1;j <= y2;j++,lpStart += SHARPMEM_LINESPAN)
        *lpStart |= temp;
    }
    else
    {
      for (j = y1;j <= y2;j++,lpStart += SHARPMEM_LINESPAN)
        *lpStart &= temp;
    }                 
  }
  else
  {
    int iSize = ((x2+8)/8)-(x1/8);
    byte start = x1&7;
    byte end = 7-(x2&7);
    byte left=0,right=0;
    if (start)
    {
      if (color)
        left = (byte)(0xff << start);
      else
        left = ~(byte)(0xff << start);
    }
    if (end)
    {
      if (color)
        right = (byte)(0xff >> end);
      else
        right = ~(byte)(0xff >> end);
    }
    if (start)
    {
      if (value)
        *lpStart++ |= left;
      else
        *lpStart++ &= left;
      iSize--;
    }
    if (end)
    {
      if (value)
        lpStart[iSize-1] |= right;
      else 
        lpStart[iSize-1] &= right;
      iSize--;
    }
    if (iSize > 0)
      memset(lpStart,value,iSize);
  }
}
 
void Sharp_Mono_LCD::fillRect(int16_t x1, int16_t y1, int16_t w, int16_t h, uint16_t color)
{
  if ((x1 < 0) || (x1 >= _width) || (y1 < 0) || (y1 >= _height))
  {
   //Serial.println("Out of bounds on entry");
    return;
  }
  int16_t x2=0,y2=0;
  if ((w <= 0) || (h <= 0))
  {
    //Serial.println("Out of bounds on entry2");
    return;
  }
  switch (rotation) {
    case 1: // 270
      x2 = WIDTH - y1-1;
      y1 = x1;
      x1 = x2-h+1;
      y2 = y1 + w-1;
      break;
    case 2: // 180
      x2 = WIDTH  - 1 - x1;
      y2 = HEIGHT - 1 - y1;
      x1 = x2-w+1;
      y1 = y2-h+1;
      break;
    case 3: // 90
      y2 = HEIGHT - x1 -1;
      x1 = y1;
      y1 = y2 - w + 1;
      x2 = x1 + h-1;
      break;
    default:
      x2 = x1+w-1;
      y2 = y1+h-1;
      break;
  }
  byte temp,value,left=0,right=0;
  int16_t j,start,end,iSize;
  byte *lpStart = &sharpmem_buffer[1+SHARPMEM_CMD_PAD+((y1 * SHARPMEM_LCD_DMAWIDTH + x1) / 8)];
  byte *lpLine;
  start = x1&7;
  end = 7-(x2&7);
  iSize = ((x2+8)/8)-(x1/8);
  value = (byte)((color)?0xff:0x00);
  if (start)
  {
    if (color)
      left = (byte)(0xff << start);
    else
      left = ~(byte)(0xff << start);
  }
  if (end)
  {
    if (color)
      right = (byte)(0xff >> end);
    else
      right = ~(byte)(0xff >> end);
  }
  if (iSize == 1)
  {
    if (start && end)
      temp = (value)?(right&left):(right|left);
    else if (start)
      temp = left;
    else if (end)
      temp = right;
    else
      temp = value;
    if (value)
    {
      for (j = y1;j <= y2;j++,lpStart += SHARPMEM_LINESPAN)
        *lpStart |= temp;
    }
    else
    {
      for (j = y1;j <= y2;j++,lpStart += SHARPMEM_LINESPAN)
        *lpStart &= temp;
    }
    return;
  }
  
  for (j = y1;j <= y2;j++,lpStart += SHARPMEM_LINESPAN)
  {
    w = iSize;
    lpLine = lpStart;
    if (start)
    {
      if (value)
        *lpLine++ |= left;
      else
        *lpLine++ &= left;
      w--;
    }
    if (end)
    {
      if (value)
        lpLine[w-1] |= right;
      else 
        lpLine[w-1] &= right;
      w--;
    }
    if (w > 0)
      memset(lpLine,value,w);
  }
}


/**************************************************************************/
/*!
    @brief Clears the screen
*/
/**************************************************************************/
void Sharp_Mono_LCD::clearDisplay()
{
  clearBuffer();
  return;
  // could add code to send this instead of full DMA buffer but need to track flags.
  // Send the clear screen command rather than doing a HW refresh (quicker)
  digitalWriteFast(_ss, HIGH);
  SPI.transfer(SHARPMEM_BIT_CLEAR); //change to LSB
  SPI.transfer(0x00);
  digitalWrite(_ss, LOW);
}

/**************************************************************************/
/*!
    @brief Clears the buffer
*/
/**************************************************************************/
void Sharp_Mono_LCD::clearBuffer()
{
  byte *line = &sharpmem_buffer[SHARPMEM_CMD_PAD+1];
  for (int i = 0;i < SHARPMEM_LCDHEIGHT;i++)
  {
    memset(line,0xff,SHARPMEM_LCDWIDTH/8);
    line += SHARPMEM_LCD_DMAWIDTH/8;
  }
}

/**************************************************************************/
/*!
    @brief Renders the contents of the pixel buffer on the LCD
*/
/**************************************************************************/
void Sharp_Mono_LCD::refresh(void)
{
  /*
   * memcpy takes about 35 microseconds on a Teensy 3.5 vs 29 milliseconds for SPI transfer using DMA.
   * Software SPI transfer would take even longer.
   * So this avoids tying up the main loop while still maintaining synchronous control.
  */
  memcpy(refresh_buffer,sharpmem_buffer,SHARPMEM_DMA_BUFFER_SIZE);
  need_refresh = true; // flag so timer will cause DMA cycle.
}

volatile bool Sharp_Mono_LCD::need_refresh = false;
EventResponder Sharp_Mono_LCD::ssEvent;

void Sharp_Mono_LCD::internal_refresh(void)
{
  if (!need_refresh)
    return;
  need_refresh = false;
  digitalWriteFast(_ss, HIGH);
  SPI.transfer(refresh_buffer,NULL,SHARPMEM_DMA_BUFFER_SIZE,ssEvent);
}
