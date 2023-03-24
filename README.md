
# Doves GPS Lap Timer
Library for Arduino for creating mostly accurate lap-timings using GPS data.
Once the driver is within a specified threshold of the line, it begins logging gps lat/lng/alt/speed.
Once past the threshold, using the 4 points closest to the line, creates a catmullrom spline to interpolate the exact crossing time.

🤖✨ Crafted with love & a sprinkle of ChatGPT magic! ✨🤖

## Supported Hardware

* literally anything, but fair warning lots of floating point math.
  * The [Seed NRF52840](https://www.amazon.com/Seeed-Studio-XIAO-nRF52840-Microcontroller/dp/B09T9VVQG7) has a dedicated high speed FPU for both floats and doubles
  * The [Matek SAM-M8Q](https://www.amazon.com/Matek-Module-SAM-M8Q-GLONASS-Galileo/dp/B07Q2SGQQT) GPS is another wonderful addition
  * Display data with a[128x64 i2c 110X display](https://www.amazon.com/dp/B08V97FYD2)

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

## Planned Functions
* List lap times
* Splits
  * "Optimal" Lap
Yea let me be real here, I just want the screen to flash when I have a good sector, and check my times in qualifying before the race.
If you want literally any other feature, use the [RaceChrono Android|iPhone App](https://racechrono.com/) or make it yourself and submit a pull-request.

#### TODO
* ~~unit tests~~
* Split timings
* Better code formatting

## API
See the source code, specifically the [DovesLapTimer.h](src/DovesLapTimer.h) file.
The code should have clarifying comments wherever there are any unclear bits.

#### Initialize
```c
  // initialize with internal debugger, and or crossingThreshold (default 10)
  #define DEBUG_SERIAL Serial
  double crossingThresholdMeters = 10.0;
  DovesLapTimer lapTimer(crossingThresholdMeters, &DEBUG_SERIAL);
  DovesLapTimer lapTimer(crossingThresholdMeters);
  DovesLapTimer lapTimer;
```
#### Setup()
Currently only supports one split line, the main start/finish.
```c
  // define start/finish line
  lapTimer.setStartFinishLine(crossingPointALat, crossingPointALng, crossingPointBLat, crossingPointBLng);
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
  // try to always keep the time up to date for pace calculations
  // can bundle with the others if you dont mind slower pace updates
  if (gps->satellites >= 1) {
    lapTimer.updateCurrentTime(getGpsTimeInMilliseconds());
  }
  // update the timer loop everytime we have fixed data
  if (gps->fixquality > 0) {
    float altitudeMeters = gps->altitude;
    float speedKnots = gps->speed;
    lapTimer.loop(gps->latitudeDegrees, gps->longitudeDegrees, altitudeMeters, speedKnots);
  }
```
#### Retrieving Data
Now if you want any running information,  you have the following...
```c
  bool getRaceStarted() const; // True if the race has started, false otherwise (passed the line one time).
  bool getCrossing() const; // True if crossing the start/finish line, false otherwise.
  unsigned long getCurrentLapStartTime() const; // The current lap start time in milliseconds.
  unsigned long getCurrentLapTime() const; // The current lap time in milliseconds.
  unsigned long getLastLapTime() const; // The last lap time in milliseconds.
  unsigned long getBestLapTime() const; // The best lap time in milliseconds.
  float getCurrentLapOdometerStart() const; // The distance traveled at the start of the current lap in meters.
  float getCurrentLapDistance() const; // The current lap time in milliseconds.
  float getLastLapDistance() const; // The distance traveled during the last lap in meters.
  float getBestLapDistance() const; // The distance traveled during the best lap in meters.
  float getTotalDistanceTraveled() const; // The total distance traveled in meters.
  int getBestLapNumber() const; // The lap number of the best lap.
  int getLaps() const; // The total number of laps completed.
```

#### Compile-time Configs
Inside [DovesLapTimer.h](src/DovesLapTimer.h)
```c
// Will eventually remove with better tests
#define DOVES_UNIT_TEST
```

## Examples

* [Basic Oled Example](examples/basic_oled_example/basic_oled_example.ino)
  * Shows all basic functionality, along with a simple display literally showing all basic functionality.
  * assumes adafruit compatible [authentic ublox GPS](https://www.amazon.com/Matek-Module-SAM-M8Q-GLONASS-Galileo/dp/B07Q2SGQQT) 
    * if not authentic, commands might fail but should probably still work.
  * Originally for [Seed NRF52840](https://www.amazon.com/Seeed-Studio-XIAO-nRF52840-Microcontroller/dp/B09T9VVQG7), might need to remove LED_GREEN blinker
  * [128x64 i2c 110X display](https://www.amazon.com/dp/B08V97FYD2). Display is NOT PRETTY, it is an EXAMPLE / DEBUG SCREEN.
    * Too tired to make serial only logger, but you can very easily remove it.
  * borb load screen
* [Unit Tests](examples/unit_test/unit_test.ino)
  * Code fully covered 34 tests
  * I believe, these results should suffice at 10-18hz below 130mph

## License

This library is [licensed](LICENSE) under the [MIT Licence](http://en.wikipedia.org/wiki/MIT_License).

## More features?
If you want more features, go and download this dudes app RaceChrono (available on both iPhone and Android), and send the data to your phone, or log it and send it after the race.

RaceChrono is not a sponsor or affiliated, i just really enjoy the app, but don't like keeping my phone in a go-kart.
If you are looking for a "proper racing solution", you can log canbus data through the NRF52840, to the RaceChrono app. This will allow you to use a much more affordable GPS module, and have a fully fledged data logger.
You can also send this data back to another(or the same) BLE device to create custom digital gauge clusters!

Paid version required for DIY loggers and importing NMEA logs, worth every penny.

[RaceChrono Website](https://racechrono.com/) | [RaceChrono iPhone](https://apps.apple.com/us/app/racechrono-pro/id1129429340) | [RaceChrono Android](https://play.google.com/store/apps/details?id=com.racechrono.pro&pli=1)

Source code for: `can-bus logger/gps logger/digital gauges`
[https://github.com/aollin/racechrono-ble-diy-device](https://github.com/aollin/racechrono-ble-diy-device)

Pairs wonderfully with the previously mentioned [Seed NRF52840](https://www.amazon.com/Seeed-Studio-XIAO-nRF52840-Microcontroller/dp/B09T9VVQG7)


###### Might as well plug my youtube here as well  :^) bunch of 360 karting videos [https://www.youtube.com/shorts/r0rKZCIl5Zw](https://www.youtube.com/shorts/r0rKZCIl5Zw)
