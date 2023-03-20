

# Doves GPS Lap Timer
Library for Arduino for creating mostly accurate lap-timings using GPS data.
Once the driver is within a specified threshold of the line, it begins logging gps lat/lng/alt/speed.
Once past the threshold, using the 4 points closest to the line, creates a catmullrom spline to interpolate the exact crossing time.

🤖✨ Crafted with love & a sprinkle of ChatGPT magic! ✨🤖

## Supported Hardware

* literally anything, but fair warning lots of floating point math.
	* The nrf52840 has a dedicated high speed FPU for both floats and doubles

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

## Examples

* [Basic Oled Example](examples/basic_oled_example/basic_oled_example.ino)
	* Shows all basic functionality, along with a simple display literally showing all basic functionality.
	* assumes adafruit compatible authentic ublox GPS 
		* if not authentic, commands might fail but should probably still work.
	* Originally for nrf52840, might need to remove LED_GREEN blinker
	* 128x64 i2c 110X display. Display is NOT PRETTY, it is a DEBUG SCREEN.
		* Too tired to make serial only logger, but you can very easily remove it.
		* \+ most arduinos and the like only have a single serial.
	* borb load screen

## License

This library is [licensed](LICENSE) under the [MIT Licence](http://en.wikipedia.org/wiki/MIT_License).
