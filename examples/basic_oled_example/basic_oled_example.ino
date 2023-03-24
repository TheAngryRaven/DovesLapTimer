/**
 * Notes: this display of data is not pretty at all, its a debug screen.... the pretty part is your job
 * 
 * Notes: assumes you have the following:
 *  1: a legit ublox GPS (in my case a SAM-M8Q, highly reccomended)
 *  2: a 128x64 1306|1106 i2c oled display
 * 
 * DovesLapTimer does not interface with your GPS directly,
 * you feed it data, and then check the state.
 *
 * For feeding data: Check out the "Setup()" "loop()" and "gpsLoop()"
 * For getting data: check out "displayStats()" towards the bottom of the file
 *
 * I like organizing code with defines.... dont shoot me
 */

//THIS DEFINES YOUR START/FINISH LINE
const double crossingPointALat = 00.00;
const double crossingPointALng = -00.00;
const double crossingPointBLat = 00.00;
const double crossingPointBLng = -00.00;

// reminder: this example pauses code until terminal connected
#define HAS_DEBUG
#define DEBUG_SERIAL Serial

#ifdef HAS_DEBUG
  #define debugln DEBUG_SERIAL.println
  #define debug DEBUG_SERIAL.print
#else
  void dummy_debug(...) {}
  #define debug dummy_debug
  #define debugln dummy_debug
#endif

#define GPS_SERIAL Serial1
#include <Adafruit_GPS.h>
Adafruit_GPS* gps = NULL;
// long series of ublox configuration commands
#include "gps_config.h"

// uses adafruit display libraries
#define USE_1306_DISPLAY // remove to use SH110X oled
#define I2C_DISPLAY_ADDRESS 0x3C
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#include "images.h"
#include "display_config.h"
int displayUpdateRateHz = 5;
unsigned long displayLastUpdate;

/**
* @brief Returns the GPS time since midnight in milliseconds
*
* @return unsigned long The time since midnight in milliseconds
*/
unsigned long getGpsTimeInMilliseconds() {
  return millis(); // hotfix, gps time doing a weird

  unsigned long timeInMillis = 0;
  timeInMillis += gps->hour * 3600000ULL;   // Convert hours to milliseconds
  timeInMillis += gps->minute * 60000ULL;   // Convert minutes to milliseconds
  timeInMillis += gps->seconds * 1000ULL;   // Convert seconds to milliseconds
  timeInMillis += gps->milliseconds;        // Add the milliseconds part

  return timeInMillis;
}

#define NMEA_LED_PIN LED_GREEN
int ledState = LOW;
unsigned long currentMillis;

#include <DovesLapTimer.h>
// initialize with internal debugger, and or crossingThreshold (default 10)
double crossingThresholdMeters = 10.0;
// DovesLapTimer lapTimer(crossingThresholdMeters, &DEBUG_SERIAL);
// DovesLapTimer lapTimer(crossingThresholdMeters);
DovesLapTimer lapTimer;

void setup() {
  #ifdef HAS_DEBUG
      Serial.begin(9600);
      while (!Serial);
  #endif

  // Initialize oled display
  displaySetup();
  // display borbo
  displayLoadPage();
  // initialize GPS and send configuration commands
  gpsSetup();
  // enabled built-in seeed NRF52840 led
  pinMode(NMEA_LED_PIN, OUTPUT);
  digitalWrite(NMEA_LED_PIN, ledState);

  // initialize laptimer class
  lapTimer.setStartFinishLine(crossingPointALat, crossingPointALng, crossingPointBLat, crossingPointBLng);
  // reset everything back to zero
  lapTimer.reset();

  debugln(F("GPS Lap Timer Started"));
}

void loop() {
  currentMillis = millis();
  gpsLoop();
  displayLoop();
}

void gpsLoop() {
    char c = gps->read();
    if (gps->newNMEAreceived() && gps->parse(gps->lastNMEA())) {
        // Toggle green LED every time valid NMEA is received
        ledState = ledState == LOW ? HIGH : LOW;
        digitalWrite(NMEA_LED_PIN, ledState);

        // try to always keep the time up to date
        if (gps->satellites >= 1) {
          lapTimer.updateCurrentTime(getGpsTimeInMilliseconds());
        }

        // update the timer loop everytime we have fixed data
        if (gps->fixquality > 0) {
          float altitudeMeters = gps->altitude;
          float speedKnots = gps->speed;
          lapTimer.loop(gps->latitudeDegrees, gps->longitudeDegrees, altitudeMeters, speedKnots);
        }
    }
}

#ifndef GPS_CONFIGURATION
  /**
   * @brief Sends a GPS configuration command stored in program memory to the GPS module via [GPS_SERIAL].
   *
   * This function reads a configuration command from PROGMEM (program memory) and sends it byte by byte to the GPS module using the [GPS_SERIAL] interface.
   * The function also prints the configuration command in hexadecimal format for debugging purposes.
   *
   * @note This function contains blocking code and should be used during setup only.
   *
   * @param Progmem_ptr Pointer to the PROGMEM (program memory) containing the GPS configuration command.
   * @param arraysize Size of the configuration command stored in PROGMEM.
   */
  void GPS_SendConfig(const uint8_t *Progmem_ptr, uint8_t arraysize) {
    uint8_t byteread, index;

    debug(F("GPSSend  "));

    for (index = 0; index < arraysize; index++)
    {
      byteread = pgm_read_byte_near(Progmem_ptr++);
      if (byteread < 0x10)
      {
        debug(F("0"));
      }
      debug(byteread, HEX);
      debug(F(" "));
    }

    debugln();
    //set Progmem_ptr back to start
    Progmem_ptr = Progmem_ptr - arraysize;

    for (index = 0; index < arraysize; index++)
    {
      byteread = pgm_read_byte_near(Progmem_ptr++);
      GPS_SERIAL.write(byteread);
    }
    delay(200);
  }

  void gpsSetup() {
    // first try serial at 9600 baud
    GPS_SERIAL.begin(9600);
    // wait for the GPS to boot
    delay(2250);
    if (GPS_SERIAL) {
      GPS_SendConfig(uart115200NmeaOnly, 28);
      GPS_SERIAL.end();
    }

    // reconnect at proper baud
    gps = new Adafruit_GPS(&GPS_SERIAL);
    GPS_SERIAL.begin(115200);
    // wait for the GPS to boot
    delay(2250);
    // Send GPS Configurations
    if (GPS_SERIAL) {
      GPS_SendConfig(NMEAVersion23, 28);
      GPS_SendConfig(FullPower, 16);

      GPS_SendConfig(GPGLLOff, 16);
      GPS_SendConfig(GPVTGOff, 16);
      GPS_SendConfig(GPGSVOff, 16);
      GPS_SendConfig(GPGSAOff, 16);
      // GPS_SendConfig(GPGGAOn5, 16); // for 10hz
      GPS_SendConfig(GPGGAOn10, 16); // for 18hz
      GPS_SendConfig(NavTypeAutomobile, 44);
      GPS_SendConfig(ENABLE_GPS_ONLY, 68);
      // GPS_SendConfig(Navrate10hz, 14);
      GPS_SendConfig(Navrate18hz, 14);
    } else {
      debugln("No GPS????");
    }
  }
#endif

// setup / loop / pages
#ifndef DISPLAY_FUNCTIONS
  void displaySetup() {
    debugln("SETTING UP DISPLAY");
    delay(250); // wait for the OLED to power up
    #ifdef USE_1306_DISPLAY
      display.begin(SSD1306_SWITCHCAPVCC, I2C_DISPLAY_ADDRESS);
    #else
      display.begin(I2C_DISPLAY_ADDRESS, true);
    #endif
    display.setTextColor(DISPLAY_TEXT_WHITE);
    display.setTextWrap(false);
    display.setFont();
    display.clearDisplay();
    display.display();
    displayLastUpdate = millis();
  }

  void displayLoop() {
    // Update Display
    if (currentMillis - displayLastUpdate > (1000 / displayUpdateRateHz)) {
      displayLastUpdate = millis();

      if (!lapTimer.getCrossing()) {
        displayStats();
      } else {
        displayCrossing();
      }
    }
  }

  bool calculatingFlip = false;
  void displayCrossing() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);

    // Draw bitmap on the screen
    calculatingFlip = calculatingFlip == true ? false : true;
    if (calculatingFlip) {
      display.drawBitmap(0, 0, image_data_calculating1, 128, 64, 1);
    } else {
      display.drawBitmap(0, 0, image_data_calculating2, 128, 64, 1);
    }

    display.display();
  }

  void displayLoadPage() {
    display.clearDisplay();
    display.drawBitmap(0, 0, image_data_bird2, 128, 64, 1);
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.println("");
    display.setTextSize(2);
    display.setTextColor(DISPLAY_TEXT_BLACK);
    display.println("Loading");
    display.setTextColor(DISPLAY_TEXT_WHITE);
    display.display();
  }

  bool activityFlasher = false;
  unsigned long lastCurTime = -1;
  unsigned long worstTimeDifference = 0;
  void displayStats() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);

    display.print("Q:");
    display.print(gps->fixquality);

    display.print(" H:");
    display.print(gps->HDOP, 2);

    // this little derp here just lets me know the device is still working when the GPS hangs for a moment
    display.print(" [");
    activityFlasher = activityFlasher == true ? false : true;
    display.print(activityFlasher == true ? "*" : " ");
    display.print("|");
    display.print(ledState == LOW ? "*" : " ");
    display.print("] ");

    // just a little debuggin with millis vs gps time
    unsigned long currentTime = getGpsTimeInMilliseconds();
    unsigned long tDiff = -1;
    if (lastCurTime > 0) {
      tDiff = currentTime - lastCurTime;
      if (tDiff > worstTimeDifference || tDiff < 0) {
        worstTimeDifference = tDiff;
      }
      display.print(worstTimeDifference);
    } else {
      display.print("0");  
    }
    lastCurTime = currentTime;

    display.println();

    display.print("St:");
    display.print(gps->satellites);
    display.print(" rs:");
    display.print(lapTimer.getRaceStarted() == true ? "T" : "F");
    display.print(" cr:");
    display.print(lapTimer.getCrossing() == true ? "T" : "F");
    display.print(" la:");
    display.print(lapTimer.getLaps());
    display.println();

    display.print("CLT: ");
    display.print(lapTimer.getCurrentLapTime() / 1000);
    display.print(".");
    display.print(lapTimer.getCurrentLapTime() % 1000);
    // display.print(", ");
    // display.print(lapTimer.lapTimer.getCurrentLapTime());
    
    // our wacky way of making sure the driver isnt off to the side and can "actually cross" the line
    bool idnl = lapTimer.isAcuteTriangle(
      gps->latitudeDegrees, gps->longitudeDegrees,
      crossingPointALat,
      crossingPointALng,
      crossingPointBLat,
      crossingPointBLng
    );
    display.print(" AT:");
    display.print(idnl == true ? "T" : "F");
    display.print(":");
    // our actual distance to the line reguardless
    double dtl = lapTimer.pointLineSegmentDistance(
      gps->latitudeDegrees, gps->longitudeDegrees,
      crossingPointALat,
      crossingPointALng,
      crossingPointBLat,
      crossingPointBLng
    );
    display.print(dtl);
    display.println();

    display.print("B:");
    display.print(lapTimer.getBestLapNumber());
    display.print("-");
    display.print(lapTimer.getBestLapTime() / 1000);
    display.print(".");
    display.print(lapTimer.getBestLapTime() % 1000);
    display.print(" L: ");
    display.print(lapTimer.getLastLapTime() / 1000);
    display.print(".");
    display.print(lapTimer.getLastLapTime() % 1000);

    // display.print("lapStart: ");
    // display.println(lapTimer.getCurrentLapStartTime());
    // display.print("CurTime: ");
    // display.println(getGpsTimeInMilliseconds());

    display.println();
    display.print("pace: ");
    display.print(lapTimer.paceDifference());
    display.print(" crD:");
    display.print(lapTimer.getCurrentLapDistance());

    display.println();
    display.print("AtM:");
    display.print(gps->altitude);
    display.print(" odo:");
    display.print(lapTimer.getTotalDistanceTraveled());

    display.println();   
    // display.print("BLD:");
    // display.print(lapTimer.getBestLapDistance());
    // display.print(" LLD:");
    // display.print(lapTimer.getLastLapDistance());
    display.print("BLP:");
    display.print(lapTimer.getBestLapTime() / lapTimer.getBestLapDistance());
    display.print(" CLP:");
    display.print(lapTimer.getCurrentLapTime() / lapTimer.getCurrentLapDistance());

    display.println();
    display.print("Time:");
    display.print(tDiff);
    display.print(":");
    display.println(getGpsTimeInMilliseconds());

    display.display();
  }
#endif