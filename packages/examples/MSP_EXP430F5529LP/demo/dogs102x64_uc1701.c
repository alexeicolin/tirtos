/* --COPYRIGHT--,BSD
 * Copyright (c) 2013, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
#include <msp430.h>
#include "grlib.h"
//#include "driverlibHeaders.h"
#include <usci_spi.h>
#include <ucs.h>
#include <gpio.h>
#include <timer_b.h>
#include "Dogs102x64_UC1701.h"


unsigned char DOGS102x64Memory[816] = {0};
unsigned char ucBacklightLevel = 11;
unsigned char ucContrast = 11;

//*****************************************************************************
//
// Writes a data word to the UC1701.  This function implements the basic SPI
// interface to the LCD display.
//
//*****************************************************************************
static void
WriteData(unsigned short usData)
{
    // Store current GIE state
    unsigned int gie = __get_SR_register() & GIE;

    // Make this operation atomic
    __disable_interrupt();

    // CS Low
    GPIO_setOutputLowOnPin(
        GPIO_PORT_P7,
        GPIO_PIN4
        );

    //CD High
    GPIO_setOutputHighOnPin(
        GPIO_PORT_P5,
        GPIO_PIN6
        );

    // USCI_B1 TX buffer ready?
    while(!USCI_SPI_getInterruptStatus(__MSP430_BASEADDRESS_USCI_B1__,
                                  USCI_SPI_TRANSMIT_INTERRUPT
                                  ));

    // Transmit data and increment pointer
    USCI_SPI_transmitData(__MSP430_BASEADDRESS_USCI_B1__,
                     usData
                     );

    // Wait for all TX/RX to finish
    while(USCI_SPI_isBusy(__MSP430_BASEADDRESS_USCI_B1__));

    // Dummy read to empty RX buffer and clear any overrun conditions
    USCI_SPI_receiveData(__MSP430_BASEADDRESS_USCI_B1__);

    // CS High
    GPIO_setOutputHighOnPin(
        GPIO_PORT_P7,
        GPIO_PIN4
        );

    // Restore original GIE state
    __bis_SR_register(gie);
}

//*****************************************************************************
//
// Writes a command to the UC1701.  This function implements the basic SPI
// interface to the LCD display.
//
//*****************************************************************************
static void
WriteCommand(unsigned char ucData)
{
    // Store current GIE state
    unsigned int gie = __get_SR_register() & GIE;

    // Make this operation atomic
    __disable_interrupt();

    // CS Low
    GPIO_setOutputLowOnPin(
        GPIO_PORT_P7,
        GPIO_PIN4
        );

    // CD Low
    GPIO_setOutputLowOnPin(
        GPIO_PORT_P5,
        GPIO_PIN6
        );

    // USCI_B1 TX buffer ready?
    while(USCI_SPI_TRANSMIT_INTERRUPT
            != USCI_SPI_getInterruptStatus(__MSP430_BASEADDRESS_USCI_B1__,
                    USCI_SPI_TRANSMIT_INTERRUPT));
    // Transmit data
    USCI_SPI_transmitData(
       __MSP430_BASEADDRESS_USCI_B1__,
       ucData
       );

    // Wait for all TX/RX to finish
    while (USCI_SPI_BUSY == USCI_SPI_isBusy(__MSP430_BASEADDRESS_USCI_B1__)) ;

    // Dummy read to empty RX buffer and clear any overrun conditions
    USCI_SPI_receiveData(__MSP430_BASEADDRESS_USCI_B1__);

    // CS High
    GPIO_setOutputHighOnPin(
        GPIO_PORT_P7,
        GPIO_PIN4
        );

    // Restore original GIE state
    __bis_SR_register(gie);
}

//*****************************************************************************
//
// Initializes the pins required for the GPIO-based LCD interface.
//
// This function configures the GPIO pins used to control the LCD display
// when the basic GPIO interface is in use. On exit, the LCD controller
// has been reset and is ready to receive command and data writes.
//
// \return None.
//
//*****************************************************************************
static void
InitGPIOLCDInterface(void)
{
    //
    // Configure the pins that connect to the LCD as outputs.
    //

    // Port initialization for LCD operation
    GPIO_setAsOutputPin(
        GPIO_PORT_P5,
        GPIO_PIN7
          );

    // Reset is active low
    GPIO_setOutputLowOnPin(
        GPIO_PORT_P5,
        GPIO_PIN7
        );

    // Reset is inactive high
    GPIO_setOutputHighOnPin(
        GPIO_PORT_P5,
        GPIO_PIN7
        );

    GPIO_setOutputLowOnPin(
        GPIO_PORT_P7,
        GPIO_PIN4
        );

    // Chip select for LCD
    GPIO_setAsOutputPin(
        GPIO_PORT_P7,
        GPIO_PIN4
          );
    // CS is active low
    GPIO_setOutputLowOnPin(
        GPIO_PORT_P7,
        GPIO_PIN4
        );

    // Command/Data for LCD
    GPIO_setAsOutputPin(
        GPIO_PORT_P5,
        GPIO_PIN6
          );
    // CD Low for command
    GPIO_setOutputLowOnPin(
        GPIO_PORT_P5,
        GPIO_PIN6
        );

    // P4.1 option select SIMO
    GPIO_setAsPeripheralModuleFunctionOutputPin(
        GPIO_PORT_P4,
        GPIO_PIN1
          );

    // P4.3 option select CLK
    GPIO_setAsPeripheralModuleFunctionOutputPin(
        GPIO_PORT_P4,
        GPIO_PIN3
          );

    // Initialize USCI_B1 for SPI Master operation
    USCI_SPI_masterInit(__MSP430_BASEADDRESS_USCI_B1__,
        USCI_SPI_CLOCKSOURCE_SMCLK,
        UCS_getSMCLK(__MSP430_BASEADDRESS_UCS__),
        12500000,
        USCI_SPI_MSB_FIRST,
        USCI_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT,
        USCI_SPI_CLOCKPOLARITY_INACTIVITY_LOW
        );

    USCI_SPI_enable(__MSP430_BASEADDRESS_USCI_B1__);

    USCI_SPI_clearInterruptFlag(__MSP430_BASEADDRESS_USCI_B1__,
                           USCI_SPI_RECEIVE_INTERRUPT
                           );
}

//*****************************************************************************
//
//! Initializes the display driver.
//!
//! This function initializes the UC1701 display controller on the panel,
//! preparing it to display data.
//!
//! \return None.
//
//*****************************************************************************

void
Dogs102x64_UC1701Init(void)
{
    InitGPIOLCDInterface();

    WriteCommand(SET_SCROLL_LINE + 0x00);
#if (defined LANDSCAPE)     // 6:00 viewing angle
    WriteCommand(SET_SEG_DIRECTION + SEG_MIRROR);
    WriteCommand(SET_COM_DIRECTION + COM_NORMAL);
#else                       // 12:00 viewing angle
    WriteCommand(SET_SEG_DIRECTION + SEG_NORMAL);
    WriteCommand(SET_COM_DIRECTION + COM_MIRROR);
#endif
    WriteCommand(SET_ALL_PIXEL_ON + DISABLE);
    WriteCommand(SET_INVERSE_DISPLAY + INVERSE);
    WriteCommand(SET_LCD_BIAS_RATIO + NINTH);
    WriteCommand(SET_POWER_CONTROL + BOOSTER + REGULATOR + FOLLOWER);
    WriteCommand(SET_VLCD_RESISTOR_RATIO + INTERNAL_RESISTOR_RATIO);
    WriteCommand(SET_ELECTRONIC_VOLUME_MSB);
    WriteCommand(SET_ELECTRONIC_VOLUME_LSB + ELECTRONIC_VOLUME_PM);
    WriteCommand(SET_ADV_PROGRAM_CONTROL0_MSB);
    WriteCommand(SET_ADV_PROGRAM_CONTROL0_LSB + TEMP_COMP_11);
    WriteCommand(SET_DISPLAY_ENABLE + ENABLE);

    // Deselect chip
    GPIO_setOutputHighOnPin(
        GPIO_PORT_P7,
        GPIO_PIN4
        );

    Dogs102x64_backlightInit();
    Dogs102x64_setBacklight(ucBacklightLevel);
    Dogs102x64_setContrast(ucContrast);
    Dogs102x64_fillScreen(0x01);
}

//*****************************************************************************
//
//! Disables the display driver.
//!
//! This function disables the UC1701 display controller on the panel,
//! and also clears display data.
//!
//! \return None.
//
//*****************************************************************************
void Dogs102x64_disable(void)
{
    Dogs102x64_fillScreen(0x01);
    WriteCommand(SET_DISPLAY_ENABLE + ENABLE);
}

//*****************************************************************************
//
//! Inverts the display color.
//!
//! This function inverts the colors displayed without changing
//! any data in the buffer
//!
//! \return None.
//
//*****************************************************************************
void Dogs102x64_InverseDisplay(void)
{
    WriteCommand(SET_INVERSE_DISPLAY + REGULAR);
}

//*****************************************************************************
//
//! Sets display color back to the default.
//!
//! This function sets the display color back to normal
//!
//! \return None.
//
//*****************************************************************************
void Dogs102x64_ClearInverseDisplay(void)
{
    WriteCommand(SET_INVERSE_DISPLAY + INVERSE);
}

//*****************************************************************************
//
//! Initializes the display driver for playing LaunchpadDefender.
//!
//! This function initializes the UC1701 display controller on the panel,
//! preparing it to display data.
//!
//! \return None.
//
//*****************************************************************************
void Dogs102x64_DefenderInit(void)
{
    WriteCommand(SET_SCROLL_LINE + 0x00);
#if (defined LANDSCAPE)     // 6:00 viewing angle
    WriteCommand(SET_SEG_DIRECTION + SEG_MIRROR);
    WriteCommand(SET_COM_DIRECTION + COM_MIRROR);
#else                       // 12:00 viewing angle
    WriteCommand(SET_SEG_DIRECTION + SEG_NORMAL);
    WriteCommand(SET_COM_DIRECTION + COM_NORMAL);
#endif
    WriteCommand(SET_ALL_PIXEL_ON + DISABLE);
    WriteCommand(SET_INVERSE_DISPLAY + REGULAR);
    WriteCommand(SET_LCD_BIAS_RATIO + NINTH);
    WriteCommand(SET_POWER_CONTROL + BOOSTER + REGULATOR + FOLLOWER);
    WriteCommand(SET_VLCD_RESISTOR_RATIO + INTERNAL_RESISTOR_RATIO);
    WriteCommand(SET_ELECTRONIC_VOLUME_MSB);
    WriteCommand(SET_ELECTRONIC_VOLUME_LSB + ELECTRONIC_VOLUME_PM);
    WriteCommand(SET_ADV_PROGRAM_CONTROL0_MSB);
    WriteCommand(SET_ADV_PROGRAM_CONTROL0_LSB + TEMP_COMP_11);
    WriteCommand(SET_DISPLAY_ENABLE + ENABLE);

    Dogs102x64_backlightInit();
    Dogs102x64_setBacklight(ucBacklightLevel);
    Dogs102x64_setContrast(ucContrast);
    Dogs102x64_fillScreen(0x00);
}
//*****************************************************************************
//
//! Fill screen with specified color
//!
//! This function fills the entire screen with white or black pixels
//! and changes the corresponding pixels in the buffer as well
//!
//! \return None.
//
//*****************************************************************************
void Dogs102x64_fillScreen(unsigned long ulValue)
{
  unsigned int i, j;

  // White Pixels
  if(ulValue)
  {
    //Page Address
    for(i = 0; i < 8; i++)
    {
      SetAddress(0,i);
      //Column Address
      for(j = 0; j < 102; j++)
      {
        DOGS102x64Memory[i * 102 + j] = 0xFF;
        WriteData(0xFF);
      }
    }
  }
  // Black Pixels
  else
  {
    //Page Address
    for(i = 0; i < 8; i++)
    {
      SetAddress(0,i);
      //Column Address
      for(j = 0; j < 102; j++)
      {
        DOGS102x64Memory[i * 102 + j] = 0x00;
        WriteData(0x00);
      }
    }
  }
}

//*****************************************************************************
//
//! Initialize the backlight.
//!
//! This function initializes the backlight of the display
//!
//! \return None.
//
//*****************************************************************************
void Dogs102x64_backlightInit(void)
{
    // Turn on Backlight
    // Uses PWM to control brightness
    GPIO_setAsPeripheralModuleFunctionOutputPin(
        GPIO_PORT_P7,
        GPIO_PIN6
          );

    // start at full brightness (8)
    TIMER_B_initCompare(__MSP430_BASEADDRESS_T0B7__,
                      TIMER_B_CAPTURECOMPARE_REGISTER_4,
                      TIMER_B_CAPTURECOMPARE_INTERRUPT_DISABLE,
                      TIMER_B_OUTPUTMODE_RESET_SET,
                      (TIMER_B_getCaptureCompareCount(__MSP430_BASEADDRESS_T0B7__,
                                                   TIMER_B_CAPTURECOMPARE_REGISTER_0
                                                   ) >> 1 )

                      );

    TIMER_B_startUpMode(   __MSP430_BASEADDRESS_T0B7__,
        TIMER_B_CLOCKSOURCE_ACLK,
        TIMER_B_CLOCKSOURCE_DIVIDER_1,
        50,
        TIMER_B_TBIE_INTERRUPT_DISABLE,
        TIMER_B_CCIE_CCR0_INTERRUPT_DISABLE,
        TIMER_B_SKIP_CLEAR
        );

}

//*****************************************************************************
//
//! Sets the backlight.
//!
//! This function sets the backlight level of the display,
//! from a level of 0-11 where 11 is the brightest setting.
//!
//! \return None.
//
//*****************************************************************************
void Dogs102x64_setBacklight(unsigned char brightness)
{
    unsigned int dutyCycle = 0, i, dummy;

    if (brightness > 0)
    {
        TB0CCTL4 = OUTMOD_7;
        dummy = (TB0CCR0 >> 4);

        dutyCycle = 12;
        for (i = 0; i < brightness; i++)
            dutyCycle += dummy;

        TB0CCR4 = dutyCycle;

        //If the backlight was previously turned off, turn it on.
        if (!ucBacklightLevel)
            TB0CTL |= MC0;
    }
    else
    {
        TB0CCTL4 = 0;
        TB0CTL &= ~MC0;
    }
    ucBacklightLevel = brightness;
}

//*****************************************************************************
//
//! Sets the contrast.
//!
//! This function sets the contrast of the UC1701 display controller,
//! from a level of 0-31 where 31 is the darkest setting.
//!
//! \return None.
//
//*****************************************************************************
void Dogs102x64_setContrast(unsigned char newContrast)
{
    //check if parameter is in range
    if (newContrast > 0x1F)
    {
        newContrast = 0x1F;
    }

    WriteCommand(SET_ELECTRONIC_VOLUME_MSB);
    WriteCommand(SET_ELECTRONIC_VOLUME_LSB + newContrast);
    ucContrast = newContrast;
}

//*****************************************************************************
//
//! Draws a non-standard grlib image.
//!
//! This function draws an image to  the UC1701 display with
//! optimum speed for playing the LaunchPad Defender game
//!
//! \return None.
//
//*****************************************************************************
void Dogs102x64_DefenderDraw(const unsigned char *pucData, int lY, int lX)
{
    unsigned char ucHeight, ucWidth;
    unsigned int ulBufferLocation, ulTempWidth;
    unsigned int i = 0;

    ulBufferLocation = lY * 102 + lX;

    ucWidth = *pucData++;
    ucHeight = *pucData++;

    ulTempWidth = ucWidth;
    while(ucHeight--)
    {
      // Set Cursor Address
      SetInverseAddress(lX, lY + i);

      while(ulTempWidth--)
      {
          DOGS102x64Memory[ulBufferLocation] = *pucData++;
          WriteData(DOGS102x64Memory[ulBufferLocation++]);
      }

      i++;
      ulTempWidth = ucWidth;
    }
}

//*****************************************************************************
//
//! Draws a pixel on the screen.
//!
//! \param pvDisplayData is a pointer to the driver-specific data for this
//! display driver.
//! \param lX is the X coordinate of the pixel.
//! \param lY is the Y coordinate of the pixel.
//! \param ulValue is the color of the pixel.
//!
//! This function sets the given pixel to a particular color.  The coordinates
//! of the pixel are assumed to be within the extents of the display.
//!
//! \return None.
//
//*****************************************************************************
static void
Dogs102x64_UC1701PixelDraw(void *pvDisplayData, int lX, int lY,
                                   unsigned int ulValue)
{
  unsigned char ulPageAddress, ulPixelHeight;
  unsigned int ulBufferLocation;

  ulPageAddress = lY/8;
  ulPixelHeight = 0x01 << (lY & 0x07);
  ulBufferLocation = ulPageAddress * 102 + lX;

  // White Pixel
  if(ulValue)
  {
    DOGS102x64Memory[ulBufferLocation] |= ulPixelHeight;
  }
  // Black Pixel
  else
  {
    DOGS102x64Memory[ulBufferLocation] &= ~ulPixelHeight;
  }

  SetAddress(lX, ulPageAddress);
  WriteData(DOGS102x64Memory[ulBufferLocation]);
}

//*****************************************************************************
//
//! Draws a horizontal sequence of pixels on the screen.
//!
//! \param pvDisplayData is a pointer to the driver-specific data for this
//! display driver.
//! \param lX is the X coordinate of the first pixel.
//! \param lY is the Y coordinate of the first pixel.
//! \param lX0 is sub-pixel offset within the pixel data, which is valid for 1
//! or 4 bit per pixel formats.
//! \param lCount is the number of pixels to draw.
//! \param lBPP is the number of bits per pixel; must be 1, 4, or 8.
//! \param pucData is a pointer to the pixel data.  For 1 and 4 bit per pixel
//! formats, the most significant bit(s) represent the left-most pixel.
//! \param pucPalette is a pointer to the palette used to draw the pixels.
//!
//! This function draws a horizontal sequence of pixels on the screen, using
//! the supplied palette.  For 1 bit per pixel format, the palette contains
//! pre-translated colors; for 4 and 8 bit per pixel formats, the palette
//! contains 24-bit RGB values that must be translated before being written to
//! the display.
//!
//! \return None.
//
//*****************************************************************************
static void
Dogs102x64_UC1701PixelDrawMultiple(void *pvDisplayData, int lX,
                                           int lY, int lX0, int lCount,
                                           int lBPP,
                                           const unsigned char *pucData,
                                           const unsigned int *pucPalette)
{
  unsigned char ulPageAddress, ulPixelHeight;
  unsigned int ulBufferLocation;
  unsigned int Byte, ulValue;

  ulPageAddress = lY/8;
  ulPixelHeight = 0x01 << (lY & 0x07);
  ulBufferLocation = ulPageAddress * 102 + lX;

  //
  // Set the cursor increment to left to right, followed by top to bottom.
  //
  SetAddress(lX, ulPageAddress);

  //
  // Determine how to interpret the pixel data based on the number of bits
  // per pixel. Only 1BPP is supported on this display
  //
  if(lBPP == 1)
  {
      // The pixel data is in 1 bit per pixel format

      // Loop while there are more pixels to draw
      while(lCount > 0)
      {
        // Get the next byte of image data
        Byte = *pucData++;

        // Loop through the pixels in this byte of image data
        for(; (lX0 < 8) && lCount; lX0++, lCount--)
        {
          ulValue = ((unsigned int *)pucPalette)[(Byte >> (7 - lX0)) & 1];

          // White Pixel
          if(ulValue)
          {
            DOGS102x64Memory[ulBufferLocation] |= ulPixelHeight;
          }
          // Black Pixel
          else
          {
            DOGS102x64Memory[ulBufferLocation] &= ~ulPixelHeight;
          }
          WriteData(DOGS102x64Memory[ulBufferLocation++]);
        }

        // Start at the beginning of the next byte of image data
        lX0 = 0;
      }
      // The image data has been drawn
  }
}

//*****************************************************************************
//
//! Draws a horizontal line.
//!
//! \param pvDisplayData is a pointer to the driver-specific data for this
//! display driver.
//! \param lX1 is the X coordinate of the start of the line.
//! \param lX2 is the X coordinate of the end of the line.
//! \param lY is the Y coordinate of the line.
//! \param ulValue is the color of the line.
//!
//! This function draws a horizontal line on the display.  The coordinates of
//! the line are assumed to be within the extents of the display.
//!
//! \return None.
//
//*****************************************************************************
static void
Dogs102x64_UC1701LineDrawH(void *pvDisplayData, int lX1, int lX2,
                                   int lY, unsigned int ulValue)
{

  unsigned char ulPageAddress, ulPixelHeight;
  unsigned int ulBufferLocation;

  ulPageAddress = lY/8;
  ulPixelHeight = 0x01 << (lY & 0x07);
  ulBufferLocation = ulPageAddress * 102 + lX1;

  // Only need to set this address once,
  // the cursor is auto-incremented horizontally
  SetAddress(lX1, ulPageAddress);

  while(lX1++ <= lX2)
  {
    // White Pixels
    if(ulValue)
    {
      DOGS102x64Memory[ulBufferLocation] |= ulPixelHeight;
    }
    // Black Pixels
    else
    {
      DOGS102x64Memory[ulBufferLocation] &= ~ulPixelHeight;
    }
    WriteData(DOGS102x64Memory[ulBufferLocation++]);
  }
}

//*****************************************************************************
//
//! Draws a vertical line.
//!
//! \param pvDisplayData is a pointer to the driver-specific data for this
//! display driver.
//! \param lX is the X coordinate of the line.
//! \param lY1 is the Y coordinate of the start of the line.
//! \param lY2 is the Y coordinate of the end of the line.
//! \param ulValue is the color of the line.
//!
//! This function draws a vertical line on the display.  The coordinates of the
//! line are assumed to be within the extents of the display.
//!
//! \return None.
//
//*****************************************************************************
static void
Dogs102x64_UC1701LineDrawV(void *pvDisplayData, int iX, int iY1,
                                   int iY2, unsigned int ulValue)
{
  unsigned char ulPageAddress1, ulPageAddress2, ulPixelHeight1, ulPixelHeight2;
  unsigned int ulBufferLocation;

  ulPageAddress1 = iY1/8;
  ulPageAddress2 = iY2/8;

  ulPixelHeight1 = 0xFF << (iY1 & 0x07);
  ulPixelHeight2 = 0xFF >> (7 - (iY2 & 0x07));
  ulBufferLocation = ulPageAddress1 * 102 + iX;

  //Vertical Line spans more than 1 page on the LCD
  if(ulPageAddress1 != ulPageAddress2)
  {
    //Write First Page of vertical Line
    SetAddress(iX, ulPageAddress1);

    // White Pixels
    if(ulValue)
    {
      DOGS102x64Memory[ulBufferLocation] |= ulPixelHeight1;
    }
    // Black Pixels
    else
    {
      DOGS102x64Memory[ulBufferLocation] &= ~ulPixelHeight1;
    }
    WriteData(DOGS102x64Memory[ulBufferLocation]);

    // Skip to next page
    ulPageAddress1++;
    ulBufferLocation += 102;

    //Write Pages between First and Last Page
    while(ulPageAddress1 < ulPageAddress2)
    {
      SetAddress(iX, ulPageAddress1);

      // White Pixels
      if(ulValue)
      {
        DOGS102x64Memory[ulBufferLocation] |= 0xFF;
      }
      // Black Pixels
      else
      {
        DOGS102x64Memory[ulBufferLocation] &= ~0xFF;
      }
      WriteData(DOGS102x64Memory[ulBufferLocation]);

      // Skip to next page
      ulPageAddress1++;
      ulBufferLocation += 102;
    }

    //Write Last Page of vertical Line
    SetAddress(iX, ulPageAddress2);

    // White Pixels
    if(ulValue)
    {
      DOGS102x64Memory[ulBufferLocation] |= ulPixelHeight2;
    }
    // Black Pixels
    else
    {
      DOGS102x64Memory[ulBufferLocation] &= ~ulPixelHeight2;
    }
    WriteData(DOGS102x64Memory[ulBufferLocation]);
  }
  //Vertical line resides on one page on the LCD
  else
  {
    ulPixelHeight1 &= ulPixelHeight2;
    SetAddress(iX, ulPageAddress1);

    // White Pixels
    if(ulValue)
    {
      DOGS102x64Memory[ulBufferLocation] |= ulPixelHeight1;
    }
    // Black Pixels
    else
    {
      DOGS102x64Memory[ulBufferLocation] &= ~ulPixelHeight1;
    }
    WriteData(DOGS102x64Memory[ulBufferLocation]);
  }
}

//*****************************************************************************
//
//! Fills a rectangle.
//!
//! \param pvDisplayData is a pointer to the driver-specific data for this
//! display driver.
//! \param pRect is a pointer to the structure describing the rectangle.
//! \param ulValue is the color of the rectangle.
//!
//! This function fills a rectangle on the display.  The coordinates of the
//! rectangle are assumed to be within the extents of the display, and the
//! rectangle specification is fully inclusive (in other words, both sXMin and
//! sXMax are drawn, along with sYMin and sYMax).
//!
//! \return None.
//
//*****************************************************************************
static void
Dogs102x64_UC1701RectFill(void *pvDisplayData, const tRectangle *pRect,
                                  unsigned int ulValue)
{
  int x0 = pRect->sXMin;
  int x1 = pRect->sXMax;
  int y0 = pRect->sYMin;
  int y1 = pRect->sYMax;
  unsigned char ulPageAddress1, ulPageAddress2, ulPixelHeight1, ulPixelHeight2;
  unsigned int ulBufferLocation;

  ulPageAddress1 = y0/8;
  ulPageAddress2 = y1/8;

  ulPixelHeight1 = 0xFF << (y0 & 0x07);
  ulPixelHeight2 = 0xFF >> (7 - (y1 & 0x07));
  ulBufferLocation = ulPageAddress1 * 102 + x0;

  // Vertical Line spans more than 1 page on the LCD
  if(ulPageAddress1 != ulPageAddress2)
  {
    // Write First Page of vertical Line
    SetAddress(x0, ulPageAddress1);

    // White Pixels
    if(ulValue)
    {
      // Draw across page
      for(; x0 <= x1; x0++)
      {
        DOGS102x64Memory[ulBufferLocation] |= ulPixelHeight1;
        WriteData(DOGS102x64Memory[ulBufferLocation++]);
      }
    }
    // Black Pixels
    else
    {
      for(; x0 <= x1; x0++)
      {
        // Draw across page
        DOGS102x64Memory[ulBufferLocation] &= ~ulPixelHeight1;
        WriteData(DOGS102x64Memory[ulBufferLocation++]);
      }
    }

    // Skip to next page and reset variables
    ulPageAddress1++;
    x0 = pRect->sXMin;
    ulBufferLocation = ulPageAddress1 * 102 + x0;

    // Write Pages between First and Last Page
    while(ulPageAddress1 < ulPageAddress2)
    {
      SetAddress(x0, ulPageAddress1);

      // White Pixels
      if(ulValue)
      {
        // Draw across page
        for(; x0 <= x1; x0++)
        {
          DOGS102x64Memory[ulBufferLocation] = 0xFF;
          WriteData(DOGS102x64Memory[ulBufferLocation++]);
        }
      }
      // Black Pixels
      else
      {
        for(; x0 <= x1; x0++)
        {
          // Draw across page
          DOGS102x64Memory[ulBufferLocation] = 0x00;
          WriteData(DOGS102x64Memory[ulBufferLocation++]);
        }
      }

      // Skip to next page and reset variables
      ulPageAddress1++;
      x0 = pRect->sXMin;
      ulBufferLocation = ulPageAddress1 * 102 + x0;
    }

    //Write Last Page of vertical Line
    SetAddress(x0, ulPageAddress2);

    // White Pixels
    if(ulValue)
    {
      // Draw across page
      for(; x0 <= x1; x0++)
      {
        DOGS102x64Memory[ulBufferLocation] |= ulPixelHeight2;
        WriteData(DOGS102x64Memory[ulBufferLocation++]);
      }
    }
    // Black Pixels
    else
    {
      for(; x0 <= x1; x0++)
      {
        // Draw across page
        DOGS102x64Memory[ulBufferLocation] &= ~ulPixelHeight2;
        WriteData(DOGS102x64Memory[ulBufferLocation++]);
      }
    }
  }
  //Vertical line resides on one page on the LCD
  else
  {
    ulPixelHeight1 &= ulPixelHeight2;
    SetAddress(x0, ulPageAddress1);

    // White Pixels
    if(ulValue)
    {
      // Draw across page
      for(; x0 <= x1; x0++)
      {
        DOGS102x64Memory[ulBufferLocation] |= ulPixelHeight1;
        WriteData(DOGS102x64Memory[ulBufferLocation++]);
      }
    }
    // Black Pixels
    else
    {
      for(; x0 <= x1; x0++)
      {
        // Draw across page
        DOGS102x64Memory[ulBufferLocation] &= ~ulPixelHeight1;
        WriteData(DOGS102x64Memory[ulBufferLocation++]);
      }
    }
  }
}

//*****************************************************************************
//
//! Clear the screen
//!
//! This function "clears" by filling the entire screen with white pixels
//! and changes the corresponding pixels in the buffer as well
//!
//! \return None.
//
//*****************************************************************************
void Dogs102x64_UC1701ClearDisplay(void *pvDisplayData, unsigned char ucValue)
{
  int i, j;

  //Page Address
  for(i = 0; i < 8; i++)
  {
    SetAddress(0,i);
    //Column Address
    for(j = 0; j < 102; j++)
    {
      DOGS102x64Memory[i * 102 + j] = 0xFF;
      WriteData(0xFF);
    }
  }
}
//*****************************************************************************
//
//! Translates a 24-bit RGB color to a display driver-specific color.
//!
//! \param pvDisplayData is a pointer to the driver-specific data for this
//! display driver.
//! \param ulValue is the 24-bit RGB color.  The least-significant byte is the
//! blue channel, the next byte is the green channel, and the third byte is the
//! red channel.
//!
//! This function translates a 24-bit RGB color into a value that can be
//! written into the display's frame buffer in order to reproduce that color,
//! or the closest possible approximation of that color.
//!
//! \return Returns the display-driver specific color.
//
//*****************************************************************************
static unsigned int
Dogs102x64_UC1701ColorTranslate(void *pvDisplayData,
                                        unsigned long ulValue)
{
    //
    // Translate from a 24-bit RGB color to a Black or White 1BPP color.
    //
    return(DPYCOLORTRANSLATE(ulValue));
}

//*****************************************************************************
//
//! Flushes any cached drawing operations.
//!
//! \param pvDisplayData is a pointer to the driver-specific data for this
//! display driver.
//!
//! This functions flushes any cached drawing operations to the display.  This
//! is useful when a local frame buffer is used for drawing operations, and the
//! flush would copy the local frame buffer to the display. This function isn't
//! implemented because each driver operation updates the buffer and the display
//!
//! \return None.
//
//*****************************************************************************
static void
Dogs102x64_UC1701Flush(void *pvDisplayData)
{
  // No operation
}

//*****************************************************************************
//
//! The display structure that describes the driver for the DOGS
//! 102x64 LCD with an UC1701 controller.
//
//*****************************************************************************
const tDisplay g_sDogs102x64_UC1701 =
{
    sizeof(tDisplay),
    DOGS102x64Memory,
#if defined(PORTRAIT) || defined(PORTRAIT_FLIP)
    64,
    102,
#else
    102,
    64,
#endif
    Dogs102x64_UC1701PixelDraw,
    Dogs102x64_UC1701PixelDrawMultiple,
    Dogs102x64_UC1701LineDrawH,
    Dogs102x64_UC1701LineDrawV,
    Dogs102x64_UC1701RectFill,
    Dogs102x64_UC1701ColorTranslate,
    Dogs102x64_UC1701Flush,
    Dogs102x64_UC1701ClearDisplay
};

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
