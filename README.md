


# Doves GPS Lap Timer
Library for Arduino for creating mostly accurate lap-timings using GPS data.
Once the driver is within a specified threshold of the line, it begins logging gps lat/lng/alt/speed.
Once past the threshold, using the 4 points closest to the line, creates a catmullrom spline to interpolate the exact crossing time.

🤖✨ Crafted with love & a sprinkle of ChatGPT magic! ✨🤖

## Supported Hardware

* literally anything, but fair warning lots of floating point math.
	* The [Seed NRF52840](https://www.amazon.com/Seeed-Studio-XIAO-nRF52840-Microcontroller/dp/B09T9VVQG7) has a dedicated high speed FPU for both floats and doubles
	* The [Matek SAM-M8Q](https://www.amazon.com/Matek-Module-SAM-M8Q-GLONASS-Galileo/dp/B07Q2SGQQT) GPS is another wonderful addition

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

#### TODO
* Convert gps time into unix time
* Split timings
* Add `updateCurrentSpeedKnts` function (default gps speed)
* Refine interpolation method
* Better code formatting

## API

See the source code, specifically the [DovesLapTimer.h](src/DovesLapTimer.h) file.
The code should have clarifying comments wherever there are any unclear bits.

#### Setup()
Currently only supports one split line, the main start/finish.
```c
  // initialize laptimer class
  lapTimer.setStartFinishLine(crossingPointALat, crossingPointALng, crossingPointBLat, crossingPointBLng);
  // reset everything back to zero
  lapTimer.reset();
```
#### Loop()->gpsLoop()
create a simple function called `getGpsTimeInMilliseconds()` to... as it says, get the current time from the gps in milliseconds.

Now inside of your gps loop, add something like the following

All of the lap timing magic is happening inside of `checkStartFinish` consider that our "timing loop".
```c
  // always update current speed, speed is accurate with as little as 2 sats
  lapTimer.updateCurrentSpeedKmh(gps->speed * 1.852);
  // always keep the time up to date, gps can keep time somewhat well once synced, or via battery backup
  lapTimer.updateCurrentTime(getGpsTimeInMilliseconds());

  if (gps->fixquality > 0) {
    float altitude = 50; // gps->altitude // flat earth for testing
    // must update odometer every refresh (ONLY WHEN FIX VALID)
    lapTimer.updateOdometer(gps->latitudeDegrees, gps->longitudeDegrees, gps->altitude);
    // check if we are crossing start/finish line (ONLY WHEN FIX VALID)
    lapTimer.checkStartFinish(gps->latitudeDegrees, gps->longitudeDegrees);
  }
```
#### Retrieving Data
Now if you want any running information,  you have the following...
```c
  bool getRaceStarted() const; // True if the race has started, false otherwise (passed the line one time).
  bool getCrossing() const; // True if crossing the start/finish line, false otherwise.
  unsigned long getCurrentLapStartTime() const; // The current lap start time in milliseconds.
  unsigned long getCurrentLapTime() const; // The current lap time in milliseconds.
  float getCurrentLapDistance() const; // The current lap time in milliseconds.
  unsigned long getLastLapTime() const; // The last lap time in milliseconds.
  unsigned long getBestLapTime() const; // The best lap time in milliseconds.
  float getCurrentLapOdometerStart() const; // The distance traveled at the start of the current lap in meters.
  float getLastLapDistance() const; // The distance traveled during the last lap in meters.
  float getBestLapDistance() const; // The distance traveled during the best lap in meters.
  int getBestLapNumber() const; // The lap number of the best lap.
  int getLaps() const; // The total number of laps completed.
  float getTotalDistanceTraveled() const; // The total distance traveled in meters.
```

#### Compile-time Configs
```c
// Force the algo to use the 2 closest points to the line and nothing more
// if not defined, defaults to catmullrom spline using 4 points nearest to the line
#define DOVES_LAP_TIMER_FORCE_LINEAR
// enable debug logging of the library to serial
#define DOVES_LAP_TIMER_DEBUG
// Change the distance to which we start logging datapoints
// note: the buffer is configured to only store 300 points at this time
#define DOVES_LAP_CROSSING_THRESHOLD_METERS 10
```

## Examples

* [Basic Oled Example](examples/basic_oled_example/basic_oled_example.ino)
	* Shows all basic functionality, along with a simple display literally showing all basic functionality.
	* assumes adafruit compatible [authentic ublox GPS ](https://www.amazon.com/Matek-Module-SAM-M8Q-GLONASS-Galileo/dp/B07Q2SGQQT) 
		* if not authentic, commands might fail but should probably still work.
	* Originally for [Seed NRF52840](https://www.amazon.com/Seeed-Studio-XIAO-nRF52840-Microcontroller/dp/B09T9VVQG7), might need to remove LED_GREEN blinker
	* [128x64 i2c 110X display](https://www.amazon.com/dp/B08V97FYD2). Display is NOT PRETTY, it is a DEBUG SCREEN.
		* Too tired to make serial only logger, but you can very easily remove it.
		* \+ most arduinos and the like only have a single serial.
	* borb load screen

## License

This library is [licensed](LICENSE) under the [MIT Licence](http://en.wikipedia.org/wiki/MIT_License).

## More features?
If you want more features, go and download this dudes app RaceChrono (available on both iPhone and Android), and send the data to your phone.

Paid version required for DIY loggers and importing NMEA logs, worth every penny.

[RaceChrono Website](https://racechrono.com/)
[RaceChrono iPhone](https://apps.apple.com/us/app/racechrono-pro/id1129429340)
[RaceChrono Android](https://play.google.com/store/apps/details?id=com.racechrono.pro&pli=1)
[https://github.com/aollin/racechrono-ble-diy-device](https://github.com/aollin/racechrono-ble-diy-device)
Pairs wonderfully with the previously mentioned [Seed NRF52840](https://www.amazon.com/Seeed-Studio-XIAO-nRF52840-Microcontroller/dp/B09T9VVQG7)