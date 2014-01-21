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
//*****************************************************************************
//
// Dogs102x64_UC1701.h - Prototypes for the Dogs102x64 LCD
//                                     display driver with a UC1701
//                                     controller.
//
//*****************************************************************************

#ifndef __DOGS102X64_UC1701_H__
#define __DOGS102X64_UC1701_H__

//*****************************************************************************
//
// Include Files
//
//*****************************************************************************


//*****************************************************************************
//
// User Configuration for the LCD Driver
//
//*****************************************************************************

// Define LCD Screen Orientation Here
#define LANDSCAPE
//#define LANDSCAPE_FLIP

//*****************************************************************************
//
// Defines for the pins that are used to communicate with the UC1701
//
//*****************************************************************************
// Pins from MSP430 connected to LCD
#define CD              BIT6
#define CS              BIT4
#define RST             BIT7
#define BACKLT          BIT6
#define SPI_SIMO        BIT1
#define SPI_CLK         BIT3

// Ports
#define CD_DIR          P5DIR
#define CD_OUT          P5OUT
#define CS_DIR          P7DIR
#define CS_OUT          P7OUT

#define CD_RST_DIR      P5DIR
#define CD_RST_OUT      P5OUT
#define CS_BACKLT_DIR   P7DIR
#define CS_BACKLT_OUT   P7OUT
#define CS_BACKLT_SEL   P7SEL
#define SPI_SEL         P4SEL
#define SPI_DIR         P4DIR

//*****************************************************************************
//
// This driver operates in two different screen orientations.  They are:
//
// * Landscape - The screen is wider than it is tall, and there are more pins and the
//               white band on the top of the display. This is selected by defining
//               LANDSCAPE in the User Configuration section above.
//
// * Landscape flip - The screen is wider than it is tall, and there are more pins
//               and the white band on the bottom of the display. This is selected by
//               defining LANDSCAPE_FLIP in the User Configuration section above.
//
// These can also be imagined in terms of screen rotation; if landscape mode is 0 degrees
// of rotation, landscape flip is 180 degrees of rotation
//
// If no screen orientation is selected, "landscape" mode will be used.
//
//*****************************************************************************
#if ! defined(LANDSCAPE) && ! defined(LANDSCAPE_FLIP)
#define LANDSCAPE
#endif

//*****************************************************************************
//
// Various UC1701 command name labels and associated control bits
//
//*****************************************************************************

//
// Set SRAM column address. The column address (ca) must be set with 2 commands, the ca MSBs and LSBs
// In both commands the MSBs or LSBs are set with the last 4 bits of the command
//
#define SET_COLUMN_ADDRESS_MSB        0x10  // BIT0 - BIT4 = ca4 - ca7
#define SET_COLUMN_ADDRESS_LSB        0x00  // BIT0 - BIT4 = ca0 - ca3

//
// Sets the power control options. The booster, regulator, and follower can all be
// independently controlled with the last 3 bits of this command
//
#define SET_POWER_CONTROL             0x28
#define BOOSTER                       BIT0
#define REGULATOR                     BIT1
#define FOLLOWER                      BIT2

//
// Scroll image up by SL rows. SL = last 5 bits of this command. Range 0 - 63
//
#define SET_SCROLL_LINE               0x40

//
// Set SRAM page address. The page address (pa) is the last 4 bits of this command. Range 0 -7
//
#define SET_PAGE_ADDRESS              0xB0

//
// Set internal resistor ratio Rb/Ra to adjust contrast
// Internal resistor ratio is controlled by the last 3 bits of this command
//
#define SET_VLCD_RESISTOR_RATIO       0x20
#define INTERNAL_RESISTOR_RATIO       0x07  // Ratio = BIT0-BIT2

//
// Set electronic volume (PM) to adjust contrast. The electronic volume must be set with 2 commands
// SET_ELECTRONIC_VOLUME_MSB command is static. PM = last 5 bits of SET_ELECTRONIC_VOLUME_LSB command
//
#define SET_ELECTRONIC_VOLUME_MSB     0x81  //Static Command
#define SET_ELECTRONIC_VOLUME_LSB     0x00  //PM = BIT0-BIT4
#define ELECTRONIC_VOLUME_PM          0x0f

//
// Set all pixels on. This command does not affect SRAM memory
// This functionality is enabled/disabled by BIT0 of this command
//
#define SET_ALL_PIXEL_ON              0xA4
#define DISABLE                       0x00
#define ENABLE                        0x01

//
// Inverse the pixels displayed. This command does not affect SRAM memory, only how it is read
// Set to regular for each "1" in SRAM memory to represent a black pixel
// Set to inverse for each "1" in SRAM memory to represent a white pixel
// Pixel polarity is controlled by BIT0 of this command
//
#define SET_INVERSE_DISPLAY           0xA6
#define REGULAR                       0x00
#define INVERSE                       0x01

//
// Enable or disable the display. Enabling the display exits all sleep modes and restores power
// Enable/disable is controlled by BIT0 of this command
//
#define SET_DISPLAY_ENABLE            0xAE
#define DISABLE                       0x00
#define ENABLE                        0x01

//
// Set SEG direction (mirror X axis). Set SEG to mirror for 6:00 viewing because of the mirrored layout
// When SEG is set to mirror, column addresses range from 0 -101
// Set SEG to normal for 12:00 viewing
// When SEG is set to normal, column addresses range from 30-131
// SEG is controlled by BIT0 of this command
//
#define SET_SEG_DIRECTION             0xA0
#define SEG_MIRROR                    0x01
#define SEG_NORMAL                    0x00

//
// Set COM direction (mirror Y axis). Set COM to normal for 6:00 viewing
// When COm is set to normal Y pixels progress 0 - 63
// Set COM to mirror for 12:00 viewing
// When COM is set to mirror Y pixels progress 63 - 0
// COM is controlled by BIT3 of this command
//
#define SET_COM_DIRECTION             0xC0
#define COM_MIRROR                    0x08
#define COM_NORMAL                    0x00

//
// Reset the system. All control registers are reset, SRAM memory is not affected
//
#define SYSTEM_RESET                  0xE2

//
// No operation
//
#define NOP                           0xE3

//
// Set voltage bias ratio BR. BR is controlled by BIT0
// BR: 0 = 1/9; 1 = 1/7
//
#define SET_LCD_BIAS_RATIO            0xA2
#define NINTH                         0x00
#define SEVENTH                       0x01

//
// Advanced Program Controls. This 2 command sequence sets the temperature compensation
// and the cursor wrapping options. SET_ADV_PROGRAM_CONTROL0_MSB is a static value
// SET_ADV_PROGRAM_CONTROL0_LSB controls the temperature compensation with BIT7, the
// column wrap-around with BIT1 and the page wrap-around with BIT0
//
#define SET_ADV_PROGRAM_CONTROL0_MSB  0xFA  //Set temp. compensation curve to -0.11%/C
#define SET_ADV_PROGRAM_CONTROL0_LSB  0x10
#define TEMP_COMP_5                   0x00
#define TEMP_COMP_11                  0x80
#define COLUMN_WRAP_AROUND            0x02
#define PAGE_WRAP_AROUND              0x01

//*****************************************************************************
//
// Macros for the Display Driver
//
//*****************************************************************************

//
// Translates a 24-bit RGB color to a display driver-specific color.
//
// \param c is the 24-bit RGB color.  The least-significant byte is the blue
// channel, the next byte is the green channel, and the third byte is the red
// channel.
//
// This macro translates a 24-bit RGB color into a value that can be written
// into the display's frame buffer in order to reproduce that color, or the
// closest possible approximation of that color. This particular driver
// requires the 8-8-8 24 bit RGB color to convert into a 1BPP color.
// It works by adding together the seperate red, green, and blue components
// of the color and any total value over 255 corresponds to a white pixel
// written to the display. Colors with total value 255 or below are black pixels
//
// \return Returns the display-driver specific color

#define DPYCOLORTRANSLATE(c)   (((((c) & 0x00ff0000) >> 16) +              \
                                 (((c) & 0x0000ff00) >> 8)  +              \
                                 (((c) & 0x000000ff))) >> 8)


//
// Sets the cursor to coordinates X, Y.
//
// \param X and Y are the LCD pixel coordinates to place the cursor
//
// This macro sets the cursor location. The LCD is broken down into
// column address and page address. The page address (0 - 7) splits the
// 64 pixel tall display into 8 pages. The column address
// requires a 2 byte write for the 4 LSBs and the 4 MSBs
// For 6:00 viewing the range for column address is (0 - 101)
// For 12:00 viewing all operations are the same except ca range is (30 - 131)
//
// \return None
#if (defined LANDSCAPE)     // 6:00 viewing angle
#define SetAddress(X, Y)                                                          \
    WriteCommand(SET_PAGE_ADDRESS + (unsigned int)(Y));                           \
    WriteCommand(SET_COLUMN_ADDRESS_LSB + (unsigned int)((X) & 0x0F));            \
    WriteCommand(SET_COLUMN_ADDRESS_MSB + (unsigned int)(((X) & 0xF0) >> 4))
#else                       // 12:00 viewing angle
#define SetAddress(X, Y)                                                          \
    WriteCommand(SET_PAGE_ADDRESS + (unsigned int)(Y));                           \
    WriteCommand(SET_COLUMN_ADDRESS_LSB + (unsigned int)((X + 30) & 0x0F));       \
    WriteCommand(SET_COLUMN_ADDRESS_MSB + (unsigned int)(((X + 30) & 0xF0) >> 4))
#endif


//
// Sets the cursor to coordinates X, Y.
//
// \param X and Y are the LCD pixel coordinates to place the cursor
//
// This macro sets the cursor location. This macro is for use when SEG is mirrored
// in the Launchpad Defender game. This sets the inverse Y page address
// For 6:00 viewing the range for column address is (0 - 101)
// For 12:00 viewing all operations are the same except ca range is (30 - 131)
//
// \return None
#if (defined LANDSCAPE)     // 6:00 viewing angle
#define SetInverseAddress(X, Y)                                                   \
    WriteCommand(SET_PAGE_ADDRESS + (unsigned int)(7 - (Y)));                     \
    WriteCommand(SET_COLUMN_ADDRESS_LSB + (unsigned int)((X) & 0x0F));            \
    WriteCommand(SET_COLUMN_ADDRESS_MSB + (unsigned int)(((X) & 0xF0) >> 4))
#else                       // 12:00 viewing angle
#define SetInverseAddress(X, Y)                                                   \
    WriteCommand(SET_PAGE_ADDRESS + (unsigned int)(7 - (Y)));                     \
    WriteCommand(SET_COLUMN_ADDRESS_LSB + (unsigned int)((X + 30) & 0x0F));       \
    WriteCommand(SET_COLUMN_ADDRESS_MSB + (unsigned int)(((X + 30) & 0xF0) >> 4))
#endif


//*****************************************************************************
//
// Prototypes for the globals exported by this driver.
//
//*****************************************************************************
extern void Dogs102x64_UC1701Init(void);
extern void Dogs102x64_fillScreen(unsigned long ulValue);
extern void Dogs102x64_DefenderDraw(const unsigned char *pucData, int lX, int lY);
extern void Dogs102x64_setBacklight(unsigned char brightness);
extern void Dogs102x64_setContrast(unsigned char newContrast);
extern void Dogs102x64_DefenderInit(void);
extern void Dogs102x64_backlightInit(void);
extern void Dogs102x64_disable(void);
extern void Dogs102x64_InverseDisplay(void);
extern void Dogs102x64_ClearInverseDisplay(void);

extern const tDisplay g_sDogs102x64_UC1701;
extern unsigned char DOGS102x64Memory[];
extern unsigned char ucBacklightLevel;
extern unsigned char ucContrast;


#endif // __DOGS102X64_UC1701_H__
