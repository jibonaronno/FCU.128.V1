/*! \file lcd.h \brief Character LCD driver for HD44780/SED1278 displays. */
//*****************************************************************************
//
// File Name	: 'lcd.h'
// Title		: Character LCD driver for HD44780/SED1278 displays
//					(usable in mem-mapped, or I/O mode)
// Author		: Pascal Stang
// Created		: 11/22/2000
// Revised		: 4/30/2002
// Version		: 1.1
// Target MCU	: Atmel AVR series
// Editor Tabs	: 4
//
///	\ingroup driver_hw
/// \defgroup lcd Character LCD Driver for HD44780/SED1278-based displays (lcd.c)
/// \code #include "lcd.h" \endcode
/// \par Overview
///		This display driver provides an interface to the most common type of
///	character LCD, those based on the HD44780 or SED1278 controller chip
/// (about 90% of character LCDs use one of these chips).  The display driver
/// can interface to the display through the CPU memory bus, or directly via
/// I/O port pins.  When using the direct I/O port mode, no additional
/// interface hardware is needed except for a contrast potentiometer.
/// Supported functions include initialization, clearing, scrolling, cursor
/// positioning, text writing, and loading of custom characters or icons
/// (up to 8).  Although these displays are simple, clever use of the custom
/// characters can allow you to create animations or simple graphics.  The
/// "progress bar" function that is included in this driver is an example of
/// graphics using limited custom-chars.
///
/// \Note The driver now supports both 8-bit and 4-bit interface modes.
///
/// \Note For full text output functionality, you may wish to use the rprintf
/// functions along with this driver
//
// This code is distributed under the GNU Public License
//		which can be found at http://www.gnu.org/licenses/gpl.txt
//
//*****************************************************************************

#ifndef LCD2_H
#define LCD2_H

#include "global.h"

// include project-dependent configurations
#include "lcdconf.h"

// if LCD_DELAY is not defined, this definition sequence
// attempts to find a suitable LCD_DELAY given the F_CPU
#ifndef LCD_DELAY
#if F_CPU >= 16000000
#define LCD_DELAY	asm volatile ("nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n");
#else
#if FCPU >= 12000000
#define LCD_DELAY	asm volatile ("nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n");
#else
#if FCPU >= 8000000
#define LCD_DELAY	asm volatile ("nop\n nop\n nop\n nop\n nop\n nop\n nop\n");
#else
#if FCPU >= 4000000
#define LCD_DELAY	asm volatile ("nop\n nop\n nop\n nop\n nop\n");
#else
#define LCD_DELAY	asm volatile ("nop\n nop\n nop\n");
#endif
#endif
#endif
#endif
#endif

// HD44780 LCD controller command set (do not modify these)
// writing:
#define LCD_CLR             0      // DB0: clear display
#define LCD_HOME            1      // DB1: return to home position
#define LCD_ENTRY_MODE      2      // DB2: set entry mode
#define LCD_ENTRY_INC       1      //   DB1: increment
#define LCD_ENTRY_SHIFT     0      //   DB2: shift
#define LCD_ON_CTRL         3      // DB3: turn lcd/cursor on
#define LCD_ON_DISPLAY      2      //   DB2: turn display on
#define LCD_ON_CURSOR       1      //   DB1: turn cursor on
#define LCD_ON_BLINK        0      //   DB0: blinking cursor
#define LCD_MOVE            4      // DB4: move cursor/display
#define LCD_MOVE_DISP       3      //   DB3: move display (0-> move cursor)
#define LCD_MOVE_RIGHT      2      //   DB2: move right (0-> left)
#define LCD_FUNCTION        5      // DB5: function set
#define LCD_FUNCTION_8BIT   4      //   DB4: set 8BIT mode (0->4BIT mode)
#define LCD_FUNCTION_2LINES 3      //   DB3: two lines (0->one line)
#define LCD_FUNCTION_10DOTS 2      //   DB2: 5x10 font (0->5x7 font)
#define LCD_CGRAM           6      // DB6: set CG RAM address
#define LCD_DDRAM           7      // DB7: set DD RAM address
// reading:
#define LCD_BUSY            7      // DB7: LCD is busy

// Default LCD setup
// this default setup is loaded on LCD initialization
#ifdef LCD_DATA_4BIT
	#define LCD_FDEF_1			(0<<LCD_FUNCTION_8BIT)
#else
	#define LCD_FDEF_1			(1<<LCD_FUNCTION_8BIT)

#endif
#define LCD_FDEF_2				(1<<LCD_FUNCTION_2LINES)

//<CDTR>
#define LCD_FUNCTION_DEFAULT	((1<<LCD_FUNCTION) | LCD_FDEF_1 | LCD_FDEF_2)

#define LCD_MODE_DEFAULT		((1<<LCD_ENTRY_MODE) | (1<<LCD_ENTRY_INC))

// custom LCD characters


// progress bar defines
#define PROGRESSPIXELS_PER_CHAR	6


// ****** Low-level functions ******
// the following functions are the only ones which deal with the CPU
// memory or port pins directly.  If you decide to use a fundamentally
// different hardware interface to your LCD, only these functions need
// to be changed, after which all the high-level functions will
// work again.

// initializes I/O pins connected to LCD
void lcdInitHW(void);
// waits until LCD is not busy
void lcdBusyWait(void);
// writes a control command to the LCD
void lcdControlWrite(u08 data);
// read the control status from the LCD
u08 lcdControlRead(void);
// writes a data byte to the LCD screen at the current position
void lcdDataWrite(u08 data);
// reads the data byte on the LCD screen at the current position
u08 lcdDataRead(void);


// ****** High-levlel functions ******
// these functions provide the high-level control of the LCD
// such as clearing the display, setting cursor positions,
// displaying text and special characters

// initializes the LCD display (gets it ready for use)
void lcdInit(void);

// moves the cursor/position to Home (upper left corner)
void lcdHome(void);

// clears the LCD display
void lcdClear(void);

// moves the cursor/position to the row,col requested
// ** this may not be accurate for all displays
void lcdGotoXY(u08 row, u08 col);

// loads a special user-defined character into the LCD
// <lcdCustomCharArray> is a pointer to a ROM array containing custom characters
// <romCharNum> is the index of the character to load from lcdCustomCharArray
// <lcdCharNum> is the RAM location in the LCD (legal value: 0-7)
//void lcdLoadCustomChar(u08* lcdCustomCharArray, u08 romCharNum, u08 lcdCharNum);

// prints a series of bytes/characters to the display
void lcdPrintData(char* data, u08 nBytes);

// displays a horizontal progress bar at the current cursor location
// <progress> is the value the bargraph should indicate
// <maxprogress> is the value at the end of the bargraph
// <length> is the number of LCD characters that the bargraph should cover
void lcdProgressBar(u16 progress, u16 maxprogress, u08 length);


unsigned char LcdInited = 0;





/*************************************************************/
/********************** LOCAL FUNCTIONS **********************/
/*************************************************************/

void lcdInitHW(void)
{
	// initialize I/O ports
	// if I/O interface is in use
#ifdef LCD_PORT_INTERFACE
	// initialize LCD control lines
	cbi(LCD_CTRL_PORT, LCD_CTRL_RS);
	cbi(LCD_CTRL_PORT, LCD_CTRL_RW);
	cbi(LCD_CTRL_PORT, LCD_CTRL_E);
	// initialize LCD control lines to output

	_delay_ms(1);

	sbi(LCD_CTRL_DDR, LCD_CTRL_RS);
	sbi(LCD_CTRL_DDR, LCD_CTRL_RW);
	sbi(LCD_CTRL_DDR, LCD_CTRL_E);
	_delay_ms(1);
	// initialize LCD data port to input
	// initialize LCD data lines to pull-up
	#ifdef LCD_DATA_4BIT
//REM		outb(LCD_DATA_DDR, inb(LCD_DATA_DDR)&0xFF);		// set data I/O lines to input (4bit)
//REM		outb(LCD_DATA_POUT, inb(LCD_DATA_POUT)|0xF0);	// set pull-ups to on (4bit)
		_delay_ms(1);
	#else
		outb(LCD_DATA_DDR, 0x00);						// set data I/O lines to input (8bit)
		outb(LCD_DATA_POUT, 0xFF);						// set pull-ups to on (8bit)
	#endif
#else
	// enable external memory bus if not already enabled
	sbi(MCUCR, SRE);			// enable bus interface
#endif
}



void lcdControlWrite(u08 data) 
{
// write the control byte to the display controller

	_delay_ms(2);

	cbi(LCD_CTRL_PORT, LCD_CTRL_RS);			// set RS to "control"
	cbi(LCD_CTRL_PORT, LCD_CTRL_RW);			// set R/W to "write"
	
		// 4 bit write
		sbi(LCD_CTRL_PORT, LCD_CTRL_E);	// set "E" line

//		delay_ms(2);

		outb(LCD_DATA_DDR, inb(LCD_DATA_DDR)| 0b11110000);	// set data I/O lines to output (4bit)

		outb(LCD_DATA_POUT, (inb(LCD_DATA_POUT)&0x0F) | ((data)&0xF0) );

		cbi(LCD_CTRL_PORT, LCD_CTRL_E);	// clear "E" line

//		delay_ms(2);

		sbi(LCD_CTRL_PORT, LCD_CTRL_E);	// set "E" line

//		delay_ms(2);

		outb(LCD_DATA_POUT, (inb(LCD_DATA_POUT)&0x0F) | ((data<<4)&0xF0) );
		
		
		cbi(LCD_CTRL_PORT, LCD_CTRL_E);	// clear "E" line

//		delay_ms(2);


}



void lcdDataWrite(u08 data) 
{
// write a data byte to the display


		_delay_ms(1);

		LCD_DELAY;								// wait
		LCD_DELAY;								// wait

		sbi(LCD_CTRL_PORT, LCD_CTRL_RS);		// set RS to "data"
		cbi(LCD_CTRL_PORT, LCD_CTRL_RW);		// set R/W to "write"
	
		sbi(LCD_CTRL_PORT, LCD_CTRL_E);	// set "E" line
		LCD_DELAY;								// wait
//		LCD_DELAY;								// wait
		outb(LCD_DATA_DDR, inb(LCD_DATA_DDR) | 0b11110000);	// set data I/O lines to output (4bit)
		outb(LCD_DATA_POUT, (inb(LCD_DATA_POUT)&0x0F) | ((data)&0xF0) );	// output data, high 4 bits
		
		LCD_DELAY;								// wait
//		LCD_DELAY;								// wait

//		delay_ms(1);
		
		cbi(LCD_CTRL_PORT, LCD_CTRL_E);	// clear "E" line
		
//		delay_ms(1);
		
//		LCD_DELAY;								// wait
		LCD_DELAY;								// wait
		sbi(LCD_CTRL_PORT, LCD_CTRL_E);	// set "E" line
		LCD_DELAY;								// wait
//		LCD_DELAY;								// wait
		
		outb(LCD_DATA_POUT, (inb(LCD_DATA_POUT)&0x0F) | ((data<<4)&0xF0) );	// output data, low 4 bits

		LCD_DELAY;								// wait
//		LCD_DELAY;								// wait
		cbi(LCD_CTRL_PORT, LCD_CTRL_E);	// clear "E" line

//		LCD_DELAY;								// wait
		LCD_DELAY;								// wait

//		delay_ms(1);
	

}




/*************************************************************/
/********************* PUBLIC FUNCTIONS **********************/
/*************************************************************/

void lcdInit()
{
	// initialize hardware
	_delay_ms(1);
	lcdInitHW();
	//wdt_reset();					/////////Watchdog Reset
//wdt_enable(WDTO_2S);

	lcdControlWrite(0x00);

	// LCD function set
	_delay_ms(1);
	lcdControlWrite(LCD_FUNCTION_DEFAULT);
	// clear LCD
	lcdControlWrite(1<<LCD_CLR);
//	wdt_reset();					/////////Watchdog Reset
//wdt_enable(WDTO_2S);
	_delay_ms(100);
	// set entry mode
	_delay_ms(1);
	lcdControlWrite(1<<LCD_ENTRY_MODE | 1<<LCD_ENTRY_INC);
	// set display to on
	//lcdControlWrite(1<<LCD_ON_CTRL | 1<<LCD_ON_DISPLAY | 1<<LCD_ON_BLINK);
	_delay_ms(1);
	lcdControlWrite(1<<LCD_ON_CTRL | 1<<LCD_ON_DISPLAY );
	// move cursor to home
	_delay_ms(1);
////rem	lcdControlWrite(1<<LCD_HOME);
	// set data address to 0
	_delay_ms(1);
////rem	lcdControlWrite(1<<LCD_DDRAM | 0x00);

	_delay_ms(1);

}

void lcdHome(void)
{
	// move cursor to home
	lcdControlWrite(1<<LCD_HOME);
}

void lcdClear(void)
{
	// clear LCD
	lcdControlWrite(1<<LCD_CLR);
}

void lcdGotoXY(u08 x, u08 y)
{
	register u08 DDRAMAddr;

	// remap lines into proper order
	switch(y)
	{
	case 0: DDRAMAddr = LCD_LINE0_DDRAMADDR+x; break;
	case 1: DDRAMAddr = LCD_LINE1_DDRAMADDR+x; break;
	case 2: DDRAMAddr = LCD_LINE2_DDRAMADDR+x; break;
	case 3: DDRAMAddr = LCD_LINE3_DDRAMADDR+x; break;
	default: DDRAMAddr = LCD_LINE0_DDRAMADDR+x;
	}

	// set data address
	lcdControlWrite(1<<LCD_DDRAM | DDRAMAddr);
}

void lcdPrintData(char* data, u08 nBytes)
{
	register u08 i;

	// check to make sure we have a good pointer
	if (!data) return;

	// print data
	for(i=0; i<nBytes; i++)
	{
		lcdDataWrite(data[i]);
	}
}




#endif
