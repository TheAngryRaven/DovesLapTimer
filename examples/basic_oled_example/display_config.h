#ifndef _DOVES_DISPLAYCONFIG_H
#define _DOVES_DISPLAYCONFIG_H
  #ifndef I2C_DISPLAY_ADDRESS
    #define I2C_DISPLAY_ADDRESS 0x3C
  #endif

  #ifndef SCREEN_WIDTH
    #define SCREEN_WIDTH 128
  #endif

  #ifndef SCREEN_HEIGHT
    #define SCREEN_HEIGHT 64
  #endif

  #include <Adafruit_GFX.h>

  #ifdef USE_1306_DISPLAY
    #include <Adafruit_SSD1306.h>
    #define DISPLAY_TEXT_WHITE SSD1306_WHITE
    #define DISPLAY_TEXT_BLACK SSD1306_BLACK
    #define OLED_RESET 2 // reccomended to use reset pin, not required
    Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
  #else
    #include <Adafruit_SH110X.h>
    #define DISPLAY_TEXT_WHITE SH110X_WHITE
    #define DISPLAY_TEXT_BLACK SH110X_BLACK
    #define OLED_RESET -1
    Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
  #endif
#endif