# Update
Full datalogger + STL case files dropped over at [https://github.com/TheAngryRaven/DovesDataLogger](https://github.com/TheAngryRaven/DovesDataLogger) (no readme yet)

Dataviewer: [https://github.com/TheAngryRaven/DovesDataViewer](https://github.com/TheAngryRaven/DovesDataViewer) (100% vibe code)

# Doves GPS Lap Timer
Library for Arduino for creating damned accurate lap-timings using GPS data, on par with other commercial solutions.
Once the driver is within a specified threshold of the line, it begins logging gps lat/lng/alt/speed.
Once past the threshold, using the 4 points closest to the line, creates a catmullrom spline to interpolate the exact crossing time.

## What's New in v4.0

* **Automatic Course Detection** - Drive a lap at any track and the library figures out which course layout you're on by matching driven distance against known courses. No manual selection needed.
* **Multi-Course Support** - Define up to 8 course layouts per track (different configurations, rental vs. pro layouts, etc). `CourseManager` orchestrates them all.
* **Direction Detection** - Automatically detects if you're driving the course forward or reverse based on which sector line you cross first.
* **"Lap Anything" Fallback** - If course detection fails after 3 attempts, falls back to `WaypointLapTimer` which drops a waypoint and uses proximity-based timing. Works on any track, anywhere - no pre-configured lines needed.
* **Configurable Thresholds** - Speed, proximity, and detection thresholds are now adjustable at runtime via setter methods.

## Supported Hardware: MCU
While this is technially an arduino library, this needs a device with a large amount of ram and processing power due to all the floating point math.
* Arduino Mega+
  * Technically appears to be working, really pushing it
* [Seed NRF52840 (Recommended)](https://www.amazon.com/Seeed-Studio-XIAO-nRF52840-Microcontroller/dp/B09T9VVQG7)
  * Has a dedicated high speed FPU for both floats and doubles
  * Fast enough to support GPS/Display/SDCard Logging
  * Really low power
    * 65mA~ with screen, gps, and bluetooth
  * 256KB RAM, 1MB Flash
  * Sense version **not** required


## Supported Hardware: GPS
  Getting GPS data is your job, not mine, but here are a couple I reccomend that work well with the Adafruit GPS library.

  >**Note:** The [Basic Oled Example](examples/basic_oled_example/basic_oled_example.ino) has an example on how to send ublox configuration commands while receiving only NMEA sentences.
  >
  >**Note:** If GPS is not an authentic UBLOX module, sending configuration commands might, fail but receiving data should probably still work.

* [Matek SAM-M10Q](https://www.amazon.com/SAM-M10Q-Supports-Concurrent-Reception-Multirotor/dp/B0BZ7931G7/)
  * 25hz GPS only
  * 16hz GPS+GALILEO+GLONASS
  * Uses NMEA or UBLOX commands (NMEA for all included examples)
* [Matek SAM-M8Q](https://www.amazon.com/Matek-Module-SAM-M8Q-GLONASS-Galileo/dp/B07Q2SGQQT)
  * 18hz GPS only
  * 10hz GPS+GLONASS
  * Uses NMEA or UBLOX commands (NMEA for all included examples)
* **Check your local/regional RC plane/drone resources for a serial compatible GPS!**
  * **US:** [Race Day Quads](https://www.racedayquads.com/)
  * **EU:** [GetFPV](https://www.getfpv.com/)


## Supported Hardware: Display
* [128x64 i2c 110X display](https://www.amazon.com/dp/B08V97FYD2)
  * **Note:** Only here for the included demo
  * **Note:** Demo also includes pre-compile switch for 1306 displays


## Supported Functions
* Current lap
  * Time
  * Distance
  * Number
* Last lap
  * Time
  * Distance
* Best lap
  * Time
  * Distance
  * Number
* Pace difference against current and best lap
* **Sector timing** (optional)
  * 3 sectors per lap (Sector 1, 2, and 3)
  * Best time for each sector
  * Current lap sector times
  * Optimal lap time (sum of best sectors)
  * Track which lap achieved best sector times
* **Direction detection** (v4.0)
  * Automatic forward/reverse detection based on sector crossing order
* **Automatic course detection** (v4.0)
  * Detects which course layout the driver is on by matching driven distance
  * Supports up to 8 course layouts per track
* **"Lap Anything" fallback** (v4.0)
  * Proximity-based lap timing when no course is detected
  * Works on any track without pre-configured crossing lines
* List lap times

## Architecture (v4.0)

The library is organized into a hierarchy of components:

```
CourseManager (orchestrator - optional, use for multi-course tracks)
├── DovesLapTimer[8]        # One per course layout, line-crossing detection
│   └── DirectionDetector   # Detects forward/reverse driving direction
├── CourseDetector           # State machine: speed → waypoint → distance match
├── WaypointLapTimer         # Fallback "Lap Anything" proximity-based timer
└── GeoMath.h                # Shared haversine distance functions
```

You can still use `DovesLapTimer` standalone if you only have one course and know your crossing lines up front. `CourseManager` is for when you want automatic course detection and multi-course support.

## API

### Option 1: DovesLapTimer (standalone, single course)

Use this if you know your track and just want lap timing. This is the original API.

See the source code, specifically the [DovesLapTimer.h](src/DovesLapTimer.h) file.
The code should have clarifying comments wherever there are any unclear bits.

#### Initialize
```c
  // Initialize with internal debugger, and or crossingThreshold (default 7)
  #define DEBUG_SERIAL Serial
  // Only change if you know what you're doing
  double crossingThresholdMeters = 7.0;
  DovesLapTimer lapTimer(crossingThresholdMeters, &DEBUG_SERIAL);
  DovesLapTimer lapTimer(crossingThresholdMeters);
  // default threshold is 7 meters, this is perfectly valid
  DovesLapTimer lapTimer;
```
#### Setup()
```c
  // define start/finish line
  lapTimer.setStartFinishLine(crossingPointALat, crossingPointALng, crossingPointBLat, crossingPointBLng);

  // OPTIONAL: define sector lines for split timing
  // Sector 1 = Start/Finish → Sector 2
  // Sector 2 = Sector 2 → Sector 3
  // Sector 3 = Sector 3 → Start/Finish
  lapTimer.setSector2Line(sector2PointALat, sector2PointALng, sector2PointBLat, sector2PointBLng);
  lapTimer.setSector3Line(sector3PointALat, sector3PointALng, sector3PointBLat, sector3PointBLng);

  // default interpolation method
  lapTimer.forceCatmullRomInterpolation();
  // Might be more accurate if your finishline is on a location you expect constant speed
  lapTimer.forceLinearInterpolation();
  // reset all counters back to zero
  lapTimer.reset();

```
#### Loop()->gpsLoop()
create a simple method with the signature `unsigned long getGpsTimeInMilliseconds();` to... as it says, get the current time from the gps in milliseconds.

Now inside of your gps loop, add something like the following

All of the lap timing magic is happening inside of `checkStartFinish` consider that our "timing loop".
```c
  // update the timer loop only when we have fully fixed data
  if (gps->fix) {
    // Update current time
    lapTimer.updateCurrentTime(getGpsTimeInMilliseconds());
    // Update current posistional data
    float altitudeMeters = gps->altitude;
    float speedKnots = gps->speed;
    lapTimer.loop(gps->latitudeDegrees, gps->longitudeDegrees, altitudeMeters, speedKnots);
  }
```

Here is an example `getGpsTimeInMilliseconds()`
```c
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
```

#### Retrieving Data
Now if you want any running information,  you have the following...
```c
  // Basic lap timing
  bool getRaceStarted() const; // True if the race has started, false otherwise (passed the line one time).
  bool getCrossing() const; // True if crossing the start/finish line, false otherwise.
  unsigned long getCurrentLapStartTime() const; // The current lap start time in milliseconds.
  unsigned long getCurrentLapTime() const; // The current lap time in milliseconds.
  unsigned long getLastLapTime() const; // The last lap time in milliseconds.
  unsigned long getBestLapTime() const; // The best lap time in milliseconds.
  float getPaceDifference() const; // Calculates the pace difference (in seconds...) between the current lap and the best lap.
  float getCurrentLapOdometerStart() const; // The distance traveled at the start of the current lap in meters.
  float getCurrentLapDistance() const; // The distance traveled during the current lap in meters.
  float getLastLapDistance() const; // The distance traveled during the last lap in meters.
  float getBestLapDistance() const; // The distance traveled during the best lap in meters.
  float getTotalDistanceTraveled() const; // The total distance traveled in meters.
  int getBestLapNumber() const; // The lap number of the best lap.
  int getLaps() const; // The total number of laps completed.

  // Sector timing (requires setSector2Line and setSector3Line to be called)
  bool areSectorLinesConfigured() const; // True if both sector lines are configured.
  int getCurrentSector() const; // Current sector (0=not started, 1/2/3=in sector).
  unsigned long getBestSector1Time() const; // Best sector 1 time in milliseconds.
  unsigned long getBestSector2Time() const; // Best sector 2 time in milliseconds.
  unsigned long getBestSector3Time() const; // Best sector 3 time in milliseconds.
  unsigned long getCurrentLapSector1Time() const; // Current lap sector 1 time in milliseconds.
  unsigned long getCurrentLapSector2Time() const; // Current lap sector 2 time in milliseconds.
  unsigned long getCurrentLapSector3Time() const; // Current lap sector 3 time in milliseconds.
  unsigned long getOptimalLapTime() const; // Sum of best sector times in milliseconds.
  int getBestSector1LapNumber() const; // Lap number that achieved best sector 1.
  int getBestSector2LapNumber() const; // Lap number that achieved best sector 2.
  int getBestSector3LapNumber() const; // Lap number that achieved best sector 3.

  // Direction detection (v4.0)
  int getDirection() const; // DIR_UNKNOWN (0), DIR_FORWARD (1), or DIR_REVERSE (2).
  bool isDirectionResolved() const; // True once direction has been determined.
```

### Option 2: CourseManager (multi-course, automatic detection)

Use this when your track has multiple course layouts and you want the library to figure out which one you're on automatically. The `CourseManager` feeds GPS data to all course timers simultaneously, uses `CourseDetector` to identify the course by driven distance, and falls back to `WaypointLapTimer` ("Lap Anything") if detection fails.

#### Define Your Track

```c
#include <CourseManager.h>

TrackConfig myTrack = {
  "Orlando Kart Center",  // longName
  "OKC",                  // shortName
  {
    // Course 1
    {
      "Pro Layout",          // name
      2100.0,                // lengthFt (used for course detection distance matching)
      // Start/Finish line (point A lat, lng, point B lat, lng)
      28.4192, -81.4301, 28.4193, -81.4300,
      // Sector 2 line
      28.4195, -81.4305, 28.4196, -81.4304,
      // Sector 3 line
      28.4190, -81.4298, 28.4191, -81.4297,
      true,  // hasSector2
      true   // hasSector3
    },
    // Course 2
    {
      "Rental Layout",
      1800.0,
      28.4192, -81.4301, 28.4193, -81.4300,
      0, 0, 0, 0,  // no sector 2
      0, 0, 0, 0,  // no sector 3
      false,
      false
    }
  },
  2  // courseCount
};
```

#### Initialize and Use

```c
CourseManager manager(myTrack, 7.0, &Serial);  // track config, crossing threshold, debug serial

void loop() {
  if (gps->fix) {
    manager.updateCurrentTime(getGpsTimeInMilliseconds());
    manager.loop(gps->latitudeDegrees, gps->longitudeDegrees, gps->altitude, gps->speed);

    if (manager.isDetectionComplete()) {
      if (manager.isLapAnythingActive()) {
        // No course matched, using Lap Anything fallback
        WaypointLapTimer* timer = manager.getLapAnythingTimer();
        // Use timer->getLaps(), timer->getLastLapTime(), etc.
      } else {
        // Course detected!
        DovesLapTimer* timer = manager.getActiveTimer();
        Serial.println(manager.getActiveCourseName());
        // Use timer->getLaps(), timer->getLastLapTime(), etc.
        // Full DovesLapTimer API available (sectors, direction, pace, etc.)
      }
    }
  }
}
```

#### CourseManager API

```c
  // Core loop (same interface as DovesLapTimer)
  void updateCurrentTime(unsigned long ms);
  int loop(double lat, double lng, float altMeters, float speedKnots);
  void reset();

  // Detection state
  bool isDetectionComplete() const;     // True if course detected or Lap Anything active.
  int getActiveCourseIndex() const;     // Index of detected course (-1 if none).
  const char* getActiveCourseName() const; // Name of detected course, or "Lap Anything".
  int getCourseCount() const;           // Number of configured courses.
  int getDetectionRejectionCount() const; // How many times detection has been rejected.

  // Timer access
  DovesLapTimer* getActiveTimer();      // Pointer to detected course's timer (NULL if none).
  WaypointLapTimer* getLapAnythingTimer(); // Pointer to fallback timer.
  bool isLapAnythingActive() const;     // True if using Lap Anything fallback.

  // Track metadata
  const char* getTrackName() const;     // Track long name.
  const char* getShortName() const;     // Track short name.

  // Memory management
  void pruneInactiveCourses();          // Deactivate non-detected timers to save RAM.

  // Threshold setters (adjustable at runtime)
  void setSpeedThresholdMph(float mph);           // Speed to trigger detection (default 20 mph).
  void setWaypointProximityMeters(float meters);   // Proximity for Lap Anything (default 30m).
  void setDetectionProximityMeters(float meters);  // Proximity for course detection (default 10m).
```

### WaypointLapTimer ("Lap Anything")

The `WaypointLapTimer` is the fallback that kicks in when no pre-configured course matches. It can also be used standalone if you just want proximity-based timing without any crossing lines.

**How it works:**
1. Wait for speed >= 20 mph, drop a waypoint at that position
2. Drive away (minimum 100m traveled)
3. When you return near the waypoint (within 30m), it buffers approach points
4. On exit from the proximity zone, it uses the closest-approach point's time as the lap split
5. Repeat for subsequent laps

```c
  // Same timing getters as DovesLapTimer (duck-typed)
  bool getRaceStarted() const;
  int getLaps() const;
  unsigned long getCurrentLapTime() const;
  unsigned long getLastLapTime() const;
  unsigned long getBestLapTime() const;
  float getPaceDifference() const;
  float getCurrentLapDistance() const;
  float getTotalDistanceTraveled() const;
  // ... etc.

  // Waypoint info
  bool hasWaypoint() const;
  double getWaypointLat() const;
  double getWaypointLng() const;

  // Sector getters exist but return 0 (sectors not supported in proximity mode)
```

### How Course Detection Works

The `CourseDetector` is a state machine that runs inside `CourseManager`:

1. **IDLE** - Waiting to start
2. **WAITING_FOR_SPEED** - Waiting for driver to reach 20 mph
3. **WAYPOINT_SET** - Speed threshold hit, waypoint dropped. Now waiting for the driver to travel 200m+ and return within 10m of the waypoint
4. **CANDIDATES_READY** - Driver returned. Driven distance is compared to each course's `lengthFt` (within 25% tolerance). Ranked candidates are built
5. **DETECTED** - `CourseManager` validated a candidate (the course's timer saw `raceStarted = true`)

If no candidates match or validation fails 3 times, `CourseManager` activates "Lap Anything" mode.

### Direction Detection

When sector lines are configured, the library automatically detects whether you're driving the course forward or in reverse:

* After the start/finish line is first crossed, the first sector line you cross determines direction
* Sector 2 first = **forward** (`DIR_FORWARD`)
* Sector 3 first = **reverse** (`DIR_REVERSE`)
* Once resolved, direction is locked and sector lines are remapped internally so timing stays correct

```c
  int getDirection() const;        // DIR_UNKNOWN (0), DIR_FORWARD (1), DIR_REVERSE (2)
  bool isDirectionResolved() const; // True once direction is known
```

## Examples
* [WokWi Emulator (basic oled example)](https://wokwi.com/projects/367029104171726849)
  * Includes 4 laps of data
  * Custom Chip included in repo [./wokwi/](wokwi/)
    * in-browser demo does not include/support uBlox configuration commands
* [Basic Oled Example](examples/basic_oled_example/basic_oled_example.ino)
  * Shows all basic functionality, along with a simple display literally showing all basic functionality.
  * Assumes adafruit compatible [authentic ublox GPS](https://www.amazon.com/Matek-Module-SAM-M8Q-GLONASS-Galileo/dp/B07Q2SGQQT)
    * If not authentic, sending configuration commands might fail but receiving data should probably still work.
  * Originally for [Seed NRF52840](https://www.amazon.com/Seeed-Studio-XIAO-nRF52840-Microcontroller/dp/B09T9VVQG7)
    * Might need to remove/change LED_GREEN blinker
  * [128x64 i2c 110X display](https://www.amazon.com/dp/B08V97FYD2).
    * Display is NOT PRETTY, it is an EXAMPLE / DEBUG SCREEN.
    * Too tired to make serial only logger, but you can very easily remove it.
  * Borb load screen
* [Sector Timing Example](examples/sector_timing_example/sector_timing_example.ino)
  * Demonstrates sector split timing functionality
  * Shows how to configure sector 2 and sector 3 lines
  * Displays best sector times and optimal lap calculation
  * Tracks which lap achieved each best sector time
  * Serial output only (easy to integrate into existing projects)
* [Real Track Data Debug](examples/real_track_data_debug/real_track_data_debug.ino)
  * **REQUIRES A LOT OF RAM TO STORE SAMPLE DATA**
  * **Serial Only** No GPS Required
  * Simple test using data recorded at [Orlando Kart Center](https://orlandokartcenter.com/)
    * MyLaps    : 1:08:807 (magnetic/official)
    * DovesTimer: 1:08:748 (LINEAR)
    * DovesTimer: 1:08.745 (CATMULLROM)

## Memory Usage

During course detection, the library uses up to ~24 KB of RAM (8 course timers running simultaneously). After detection completes, call `pruneInactiveCourses()` to deactivate unused timers and drop to ~5 KB.

If using `DovesLapTimer` standalone (no `CourseManager`), memory usage is much lower - just the single timer instance with its crossing point buffer (100 entries on boards with >3KB RAM, 25 entries otherwise).

## License

This library is [licensed](LICENSE) under the [GNU General Public License v3.0](https://www.gnu.org/licenses/gpl-3.0.en.html).

## Dependencies
* Auto-Included/Required
  * [ArxTypeTraits](https://github.com/hideakitai/ArxTypeTraits/actions)
* Nice To Use / Used in examples
  * [Adafruit_GPS](https://github.com/adafruit/Adafruit_GPS)
    * My go-to gps library
  * [Adafruit-GFX-Library](https://github.com/adafruit/Adafruit-GFX-Library)
  * [Adafruit_SSD1306](https://github.com/adafruit/Adafruit_SSD1306)
  * [Adafruit_SH110x](https://github.com/wonho-maker/Adafruit_SH110x)

## More features?
If you want more features, go and download this dudes app RaceChrono (available on both iPhone and Android), and send the data to your phone, or log it and send it after the race.

RaceChrono is not a sponsor or affiliated, I just really enjoy the app, but don't like keeping my phone in a go-kart.
If you are looking for a "proper racing solution", you can log canbus data through the NRF52840, to the RaceChrono app. This will allow you to use a much more affordable GPS module, and have a fully fledged data logger.
You can also send this data back to another(or the same) BLE device to create custom digital gauge clusters!

Paid version required for DIY loggers and importing NMEA logs, worth every penny.

[RaceChrono Website](https://racechrono.com/) | [RaceChrono iPhone](https://apps.apple.com/us/app/racechrono-pro/id1129429340) | [RaceChrono Android](https://play.google.com/store/apps/details?id=com.racechrono.pro&pli=1)

Source code for: `can-bus logger/gps logger/digital gauges`
[https://github.com/aollin/racechrono-ble-diy-device](https://github.com/aollin/racechrono-ble-diy-device)

Pairs wonderfully with the previously mentioned [Seed NRF52840](https://www.amazon.com/Seeed-Studio-XIAO-nRF52840-Microcontroller/dp/B09T9VVQG7)
