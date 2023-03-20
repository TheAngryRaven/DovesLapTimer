# Doves GPS Lap Timer
Library for Arduino for creating mostly accurate lap-timings using GPS data.
Once the driver is within a specified threshold of the line, it begins logging gps lat/lng/alt/speed.
Once past the threshold, using the 4 points closest to the line, creates a catmullrom spline to interpolate the exact crossing time.

Written with love ... and whatever you want to call chatGPT

## Supported Hardware

* literally anything, but fair warning lots of floating point math

## Supported Functions
* Current lap time / distance
* last lap time / distance
* best lap time / number / distance
* pace difference agaisnt best lap

#### TODO
* Split timings

## API

See the source code, specifically the [DovesLapTimer.h](src/DovesLapTimer.h) file.
The code should have clarifying comments wherever there are any unclear bits.

TODO: write actual documentation

## Examples

See [examples](examples) folder.
note: example was using a seed nrf52840, so you might need to change the serial/disable the LED
if you want to just remove the display code look at `displayStats()` to see what information can be pulled from the library

## License

This library is [licensed](LICENSE) under the [MIT Licence](http://en.wikipedia.org/wiki/MIT_License).
