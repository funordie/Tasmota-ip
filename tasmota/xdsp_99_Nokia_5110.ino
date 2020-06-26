/*
  xdsp_99_Nokia_5110 - Display Nokia 5110 support for Tasmota

  Copyright (C) 2020  Gerhard Mutz and Theo Arends

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifdef USE_SPI
#ifdef USE_DISPLAY
#ifdef USE_DISPLAY_5110

#define XDSP_99                99

#define NOKIA5110_WIDTH  84
#define NOKIA5110_HEIGHT 48

#define SHOW_SPLASH

#include <renderer.h>
//#include <Adafruit_GFX.h>      // include adafruit graphics library
#include <Adafruit_PCD8544.h>  // include adafruit PCD8544 (Nokia 5110) library

Adafruit_PCD8544 * display;

void Nokia5110_InitDriver() {
	if (!Settings.display_model) {
		Settings.display_model = XDSP_99;
	}

	if (Settings.display_width != NOKIA5110_WIDTH) {
	  Settings.display_width = NOKIA5110_WIDTH;
	}
	if (Settings.display_height != NOKIA5110_HEIGHT) {
	  Settings.display_height = NOKIA5110_HEIGHT;
	}

	buffer=0;

	// init renderer
	if  (PinUsed(GPIO_SSPI_CS) && PinUsed(GPIO_SSPI_DC) && PinUsed(GPIO_SSPI_MOSI) && PinUsed(GPIO_SSPI_SCLK) && PinUsed(GPIO_OLED_RESET)){
		AddLog_P2(LOG_LEVEL_DEBUG, PSTR(D_LOG_DEBUG "%s: all gpio's are set"), __FUNCTION__);
		// Nokia 5110 LCD module connections (CLK, DIN, D/C, CS, RST)
		display = new Adafruit_PCD8544(Pin(GPIO_SSPI_SCLK), Pin(GPIO_SSPI_MOSI), Pin(GPIO_SSPI_DC), Pin(GPIO_SSPI_CS), Pin(GPIO_OLED_RESET));
	}
	else {
		AddLog_P2(LOG_LEVEL_ERROR, PSTR(D_LOG_DEBUG "%s: missing GPIO!!!!!!!!!!"), __FUNCTION__);
		return;
	}

	delay(100);
	SPI.begin();

	display->begin();
	// init done

    renderer = display;
    renderer->DisplayInit(DISPLAY_INIT_MODE,Settings.display_size,Settings.display_rotate,Settings.display_font);
    renderer->dim(Settings.display_dimmer);

#define NOKIA5110_BLACK       0x0000      /*   0,   0,   0 */
#define NOKIA5110_WHITE       0xFFFF      /* 255, 255, 255 */

#ifdef SHOW_SPLASH
    // Welcome text
    renderer->setTextFont(2);
    renderer->DrawStringAt(10, 60, "Nokia 5110", NOKIA5110_BLACK,0);
    delay(1000);
#endif
}

void Nokia5110_Refresh(void) {

}
/*********************************************************************************************/
/*********************************************************************************************\
 * Interface
\*********************************************************************************************/
bool Xdsp99(uint8_t function)
{
  bool result = false;

  if (FUNC_DISPLAY_INIT_DRIVER == function) {
	  Nokia5110_InitDriver();
  }
  else if (XDSP_99 == Settings.display_model) {
    switch (function) {
      case FUNC_DISPLAY_MODEL:
        result = true;
        break;
      case FUNC_DISPLAY_EVERY_SECOND:
        Nokia5110_Refresh();
        break;
    }
  }
  return result;
}
#endif  // USE_DISPLAY_5110
#endif  // USE_DISPLAY
#endif  // USE_SPI
