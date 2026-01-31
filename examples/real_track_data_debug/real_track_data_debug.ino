/**
 * Real debug using adafruit GPS library without a GPS connected
 *
 * Will probably be cleaned up... eventually... to replace current unit-test suite
 *
 * 3 different data-sets available
 */
#include <stdlib.h>
#include <string.h>

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

// were not actually using the GPS, we just need the library
#define GPS_SERIAL Serial1
#include <Adafruit_GPS.h>
Adafruit_GPS* gps = NULL;

#include <DovesLapTimer.h>
// probably dont change unless you know what you're doing
// might be better off making track dependant??
double crossingThresholdMeters = 7.0;
DovesLapTimer lapTimer(crossingThresholdMeters, &DEBUG_SERIAL);
// DovesLapTimer lapTimer(crossingThresholdMeters);

// orlando kart center
// Width of your crossing line is somewhat important
// Do not make the width of your crossing line too much larger than the track
// a tolerence is already included, more testing to be done tho

// these two are closer to turn 10 and make it better for testing the detection
const double crossingPointALat = 28.41270817056385;
const double crossingPointALng = -81.37973266418031;
const double crossingPointBLat = 28.41273038679321;
const double crossingPointBLng = -81.37957048753776;

// Actual data samples from a trip to the track
// #include "gps_race_data_lap.h" // normal track
#include "gps_race_data_2laps.h" // normal track
// #include "gps_race_data_long_lap.h" // long track

const int num_gps_logs = sizeof(gps_logs) / sizeof(gps_logs[0]);

// Define a static variable to keep track of the last processed line
static int last_processed_line = -1;

/**
 * Returns the next NMEA data string from a pre-defined array of GPS logs.
 * The function stores the logs in a global array and keeps track of the
 * last processed line using a static variable. Each time the function is
 * called, it returns the next line in the array until all the lines have
 * been processed. If there are no more lines to process, the function
 * returns NULL.
 *
 * @return A pointer to a static buffer containing the next NMEA data string
 *         from the array, or NULL if there are no more lines to process.
 *         The buffer is reused on each call, so copy data if needed.
 */
char *gpsLastFakeNMEA() {
    // Static buffer to avoid memory allocation - NMEA sentences are max 82 chars
    static char nmea_buffer[128];

    // Check if we've processed all the lines
    if (last_processed_line >= num_gps_logs - 1) {
      return NULL;
    }

    // Update the last processed line
    last_processed_line++;

    // Copy the NMEA data into the static buffer (safely)
    strncpy(nmea_buffer, gps_logs[last_processed_line], sizeof(nmea_buffer) - 1);
    nmea_buffer[sizeof(nmea_buffer) - 1] = '\0';  // Ensure null termination

    // Return the buffer
    return nmea_buffer;
}

/**
 * @brief Returns the GPS time since midnight in milliseconds
 *
 * @return unsigned long The time since midnight in milliseconds
 */
unsigned long getGpsTimeInMilliseconds() {
  unsigned long timeInMillis = 0;
  timeInMillis += gps->hour * 3600000ULL;   // Convert hours to milliseconds
  timeInMillis += gps->minute * 60000ULL;   // Convert minutes to milliseconds
  timeInMillis += gps->seconds * 1000ULL;   // Convert seconds to milliseconds
  timeInMillis += gps->milliseconds;        // Add the milliseconds part
  return timeInMillis;
}

void gpsLoop() {
    if (gps->parse(gpsLastFakeNMEA())) {
        // update the timer loop everytime we have fixed data
        if (gps->fix) {
          // Update current time
          lapTimer.updateCurrentTime(getGpsTimeInMilliseconds());
          // Update current posistional data
          float altitudeMeters = gps->altitude;
          float speedKnots = gps->speed;
          lapTimer.loop(gps->latitudeDegrees, gps->longitudeDegrees, altitudeMeters, speedKnots);
        }
    }
}

void setup() {
  #ifdef HAS_DEBUG
    Serial.begin(9600);
    while (!Serial);
  #endif
  debugln("Started");

  gps = new Adafruit_GPS(&GPS_SERIAL);

  // initialize laptimer class
  lapTimer.setStartFinishLine(crossingPointALat, crossingPointALng, crossingPointBLat, crossingPointBLng);
  lapTimer.forceLinearInterpolation();
  // lapTimer.forceCatmullRomInterpolation();

  // reset everything back to zero
  lapTimer.reset();

  debugln(F("GPS Lap Timer Started"));
}

bool done = false;
unsigned long lastLapTime = -1;
void loop() {
  if (last_processed_line >= num_gps_logs - 1 ) {
    if (!done) {
      done = true;
      debugln();
      debugln("~~~~~~~~Finished~~~~~~~~");
    }
    return;
  }

  // will process one loop at a time
  gpsLoop();

  if (last_processed_line > 0) {
    debug(".");

    if (lastLapTime != lapTimer.getLastLapTime()) {
      lastLapTime = lapTimer.getLastLapTime();
      debugln();
      debug("LastLapTime: ");
      debug(lastLapTime / 1000);
      debug(".");
      debugln(lastLapTime % 1000);
    }

    // if (lapTimer.getRaceStarted()) {
    //   debugln();
    //   debug("getCurrentLapTime: ");
    //   debug(lapTimer.getCurrentLapTime() / 1000);
    //   debug(".");
    //   debugln(lapTimer.getCurrentLapTime() % 1000);
    // }

    // for multilap datasets
    // if (lapTimer.getRaceStarted()) {
    //   debugln();
    //   debug("getPaceDifference: ");
    //   debug(lapTimer.getPaceDifference());
    // }

    // emulate realtime
    // delay(1000 / 25);
  }
}
