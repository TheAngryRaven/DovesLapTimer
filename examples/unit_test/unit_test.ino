/*
 * unit tests only require a compatible microcontroller that can handle this math
 * No GPS, screen, or buttons required, only a serial out
 *
 * TODO: clean all this crap up
 */
#include <Wire.h>
#include <Math.h>

const double EARTH_RADIUS = 6371.0 * 1000;

//Automatically generate points for unit-tests
double crossingPointALat =  28.538336; // Orlando, florida
double crossingPointALng = -81.379234;
double crossingPointBLat = 00.00;
double crossingPointBLng = -00.00;

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

// MUST ALSO DEFINE THIS FLAG IN LIBRARY HEADER TO RUN FULL SUITE
// #define DOVES_UNIT_TEST
#include <DovesLapTimer.h>
#define CROSSING_THRESHOLD_METERS 10
// DovesLapTimer lapTimer(CROSSING_THRESHOLD_METERS, &DEBUG_SERIAL);
DovesLapTimer lapTimer(CROSSING_THRESHOLD_METERS);

// Function array with corresponding names
typedef bool (*TestFunction)();
struct Test {
  TestFunction function;
  const char* name;
};

struct GpsCords {
  double lat;
  double lng;
};

bool testIsObtuseTriangle1();
bool testIsObtuseTriangle2();
bool testIsObtuseTriangle3();
bool testPointOnSideOfLine1();
bool testPointOnSideOfLine2();
bool testPointOnSideOfLine3();
bool testHaversine1();
bool testHaversine2();
bool testHaversine3();
bool testPointLineSegmentDistance1();
bool testPointLineSegmentDistance2();
bool testPointLineSegmentDistance3();
bool testPointLineSegmentDistance4();
bool testNotCrossingNextToLine1();
bool testNotCrossingNextToLine2();
bool testNotCrossingNextToLine3();
bool testNotCrossingNextToLine4();
bool testNotCrossingNextToLine5();
bool testNotCrossingNextToLine6();
bool testInCrossingThreshold();
bool testRaceStarted();
bool testLapDetection();
#ifdef DOVES_UNIT_TEST
bool testCatmullRom1();
bool testCatmullRom2();
bool testCatmullRom3();
bool testInterpolateWeight1();
bool testInterpolateWeight2();
bool testInterpolateWeight3();
bool testInterpolateWeight4();
bool testInterpolationOnCurve1();
bool testInterpolationOnCurve2();
bool testInterpolationOnCurve3();
bool testInterpolationOnCurve4();
bool testInterpolationLinear1();
bool testInterpolationLinear2();
bool testInterpolationLinear3();
bool testInterpolationLinear4();
#endif

Test tests[] = {
  
  {testIsObtuseTriangle1, "testIsObtuseTriangle1"},
  {testIsObtuseTriangle2, "testIsObtuseTriangle2"},
  {testIsObtuseTriangle3, "testIsObtuseTriangle3"},
  {testHaversine1, "testHaversine1"},
  {testHaversine2, "testHaversine2"},
  {testHaversine3, "testHaversine3"},
  {testPointLineSegmentDistance1, "testPointLineSegmentDistance1"},
  {testPointLineSegmentDistance2, "testPointLineSegmentDistance2"},
  {testPointLineSegmentDistance3, "testPointLineSegmentDistance3"},
  {testPointLineSegmentDistance4, "testPointLineSegmentDistance4"},
  {testNotCrossingNextToLine1, "testNotCrossingNextToLine1"},
  {testNotCrossingNextToLine2, "testNotCrossingNextToLine2"},
  {testNotCrossingNextToLine3, "testNotCrossingNextToLine3"},
  {testNotCrossingNextToLine4, "testNotCrossingNextToLine4"},
  {testNotCrossingNextToLine5, "testNotCrossingNextToLine5"},
  {testNotCrossingNextToLine6, "testNotCrossingNextToLine6"},
  {testInCrossingThreshold, "testInCrossingThreshold"},
  
  #ifdef DOVES_UNIT_TEST
  {testCatmullRom1, "testCatmullRom1"},
  {testCatmullRom2, "testCatmullRom2"},
  {testCatmullRom3, "testCatmullRom3"},
  {testInterpolateWeight1, "testInterpolateWeight1"},
  {testInterpolateWeight2, "testInterpolateWeight2"},
  {testInterpolateWeight3, "testInterpolateWeight3"},
  {testInterpolateWeight4, "testInterpolateWeight4"},
  {testInterpolationOnCurve1, "testInterpolationOnCurve1"},
  {testInterpolationOnCurve2, "testInterpolationOnCurve2"},
  {testInterpolationOnCurve3, "testInterpolationOnCurve3"},
  {testInterpolationOnCurve4, "testInterpolationOnCurve4"},
  {testInterpolationLinear1, "testInterpolationLinear1"},
  {testInterpolationLinear2, "testInterpolationLinear2"},
  {testInterpolationLinear3, "testInterpolationLinear3"},
  {testInterpolationLinear4, "testInterpolationLinear4"},
  #endif

  {testRaceStarted, "testRaceStarted"},
  {testLapDetection, "testLapDetection"}
  /*
    TODO:
      catmullrom / interpolationWeight
        Need someone to double check these
      edgecases:
        what happens if someone enters the threshold but never crosses the line?
        what happens if we start to close to the line?
      pace:
        2-3 simple pace tests
      interpolation:
        Currently a few primitive tests in place, would love to add more
        I believe, these results should suffice at 10-18hz below 130mph
  */
};


// these functions should only be used for testing
// Helper function to convert degrees to radians
double degreesToRadians(double degrees) {
  return degrees * (M_PI / 180.0);
}

// Helper function to convert radians to degrees
double radiansToDegrees(double radians) {
  return radians * (180.0 / M_PI);
}

GpsCords midpoint(double lat1, double lng1, double lat2, double lng2) {
  GpsCords newCords;

  // Convert the input coordinates to radians
  lat1 = degreesToRadians(lat1);
  lng1 = degreesToRadians(lng1);
  lat2 = degreesToRadians(lat2);
  lng2 = degreesToRadians(lng2);

  // Calculate the midpoint using spherical interpolation
  double Bx = cos(lat2) * cos(lng2 - lng1);
  double By = cos(lat2) * sin(lng2 - lng1);

  double lat3 = atan2(sin(lat1) + sin(lat2), sqrt((cos(lat1) + Bx) * (cos(lat1) + Bx) + By * By));
  double lng3 = lng1 + atan2(By, cos(lat1) + Bx);

  // Convert the result back to degrees
  newCords.lat = radiansToDegrees(lat3);
  newCords.lng = radiansToDegrees(lng3);

  return newCords;
}

GpsCords moveNorth(GpsCords cords, double distanceInMeters) {
  GpsCords newCords;
  double lat1Rad = degreesToRadians(cords.lat);
  double dLatRad = distanceInMeters / EARTH_RADIUS;
  
  newCords.lat = radiansToDegrees(lat1Rad + dLatRad);
  newCords.lng = cords.lng;
  return newCords;
}

GpsCords moveSouth(GpsCords cords, double distanceInMeters) {
  return moveNorth(cords, -distanceInMeters);
}

GpsCords moveWest(GpsCords cords, double distanceInMeters) {
  GpsCords newCords;
  double lat1Rad = degreesToRadians(cords.lat);
  double lng1Rad = degreesToRadians(cords.lng);
  double dLngRad = -distanceInMeters / (EARTH_RADIUS * cos(lat1Rad));

  newCords.lat = cords.lat;
  newCords.lng = radiansToDegrees(lng1Rad + dLngRad);
  return newCords;
}

GpsCords moveEast(GpsCords cords, double distanceInMeters) {
  return moveWest(cords, -distanceInMeters);
}

const int CROSSING_LINE_LENGTH = 5;
unsigned long currentMillis;
int failedTests = 0;
GpsCords finishLineMidPoint;

void setup() {
  #if defined(HAS_DEBUG) || defined(DOVES_LAP_TIMER_DEBUG)
    DEBUG_SERIAL.begin(9600);
    while (!DEBUG_SERIAL);
  #endif

  debugln("Lap Timer Unit Tests Started\n");
  GpsCords crossingPointA;
  crossingPointA.lat = crossingPointALat;
  crossingPointA.lng = crossingPointALng;

  // generate start line from single point
  GpsCords crossingPointB = moveEast(crossingPointA, CROSSING_LINE_LENGTH);

  // figure out mid-point for later use
  finishLineMidPoint = midpoint(crossingPointA.lat, crossingPointA.lng, crossingPointB.lat, crossingPointB.lng);

  // make sure these are set to reference later
  crossingPointALat = crossingPointA.lat;
  crossingPointALng = crossingPointA.lng;
  crossingPointBLat = crossingPointB.lat;
  crossingPointBLng = crossingPointB.lng;

  for (int i = 0; i < (sizeof(tests) / sizeof(tests[0])); i++) {
    unsigned long testStart = micros();
    // re-initialize before each test
    lapTimer.setStartFinishLine(crossingPointALat, crossingPointALng, crossingPointB.lat, crossingPointB.lng);
    lapTimer.reset();
    lapTimer.updateCurrentTime(millis());
    lapTimer.forceCatmullRomInterpolation();

    bool result = tests[i].function();
    if (!result) {
      debug("Test failed: ");
      debugln(tests[i].name);
      failedTests++;
    } else {
      // debug("Test passed: ");
      // debugln(tests[i].name);
      // debug("Test took: ");
      // debug(micros() - testStart);
      // debugln(" microseconds");
    }
  }

  if (failedTests > 0) {
    debugln("\n!! TESTS COMPLETED WITH FAILURES !!");
    debug(failedTests);
    debug(" out of ");
    debug((sizeof(tests) / sizeof(tests[0])));
    debugln(" tests failed!!");
  } else {
    debug((sizeof(tests) / sizeof(tests[0])));
    debugln(" tests completed successfully!");
  }
}

// bool allTestsPassed = false;
int testsFailed = 0;
void loop() {
  currentMillis = millis();
}

const double EPSILON = 1e-6;
bool doubleEquals(double a, double b, double epsilon = EPSILON) {
  return abs(a - b) <= epsilon * abs(a);
}


// Test case 1: Acute triangle
bool testIsObtuseTriangle1() {
  bool result = lapTimer.isObtuseTriangle(0.0, 0.0, 1.0, 0.0, 0.5, 1.0);
  if (!result) {
    return false;
  }
  return true;
}
// Test case 2: Right triangle
bool testIsObtuseTriangle2() {
  bool result = lapTimer.isObtuseTriangle(0.0, 0.0, 0.0, 3.0, 2.236068, 0.0);
  if (!result) {
    return false;
  }
  return true;
}
// Test case 3: Obtuse triangle
bool testIsObtuseTriangle3() {
  bool result = lapTimer.isObtuseTriangle(0.0, 0.0, 1.0, 0.0, 2.0, 2.0);
  if (result) {
    return false;
  }
  return true;
}


// Test case 1: Same point (distance should be 0)
bool testHaversine1() {
  double result = lapTimer.haversine(40.7128, -74.0060, 40.7128, -74.0060);
  if (!doubleEquals(result, 0.0)) {
    return false;
  }
  return true;
}
// Test case 2: Short distance
bool testHaversine2() {
  double result = lapTimer.haversine(0.0, 0.000000, 0.08993211, 0.0);
  if (!doubleEquals(result, 10000)) {
    return false;
  }
  return true;
}
// Test case 3: Long distance
bool testHaversine3() {
  double result = lapTimer.haversine(0.0, 0.0, 0.89932150, 0.0);
  if (!doubleEquals(result, 100000)) {
    return false;
  }
  return true;
}


// Test case 1: Driver is on one side of the line
bool testPointOnSideOfLine1() {
  int result = lapTimer.pointOnSideOfLine(0.0, 0.0, 1.0, 1.0, 2.0, 2.0);
  if (result != 1) {
    return false;
  }
  return true;
}
// Test case 2: Driver is on the other side of the line
bool testPointOnSideOfLine2() {
  int result = lapTimer.pointOnSideOfLine(1.0, 0.0, 0.0, 1.0, 2.0, 1.0);
  if (result != -1) {
    return false;
  }
  return true;
}
// Test case 3: Driver is exactly on the line
bool testPointOnSideOfLine3() {
  int result = lapTimer.pointOnSideOfLine(1.5, 1.5, 1.0, 1.0, 2.0, 2.0);
  if (result != 0) {
    return false;
  }
  return true;
}


// Test case 1: Line segment is a point
bool testPointLineSegmentDistance1() {
  double result = lapTimer.pointLineSegmentDistance(0.0, 0.0, 1.0, 1.0, 1.0, 1.0);
  if (!doubleEquals(result, lapTimer.haversine(0.0, 0.0, 1.0, 1.0))) {
    return false;
  }
  return true;
}
// Test case 2: Point is on the line segment
bool testPointLineSegmentDistance2() {
  double result = lapTimer.pointLineSegmentDistance(2.0, 2.0, 1.0, 1.0, 3.0, 3.0);
  if (!doubleEquals(result, 0.0)) {
    return false;
  }
  return true;
}
// Test case 3: Point is closest to the start point of the line segment
bool testPointLineSegmentDistance3() {
  double result = lapTimer.pointLineSegmentDistance(0.0, 0.0, 1.0, 1.0, 3.0, 3.0);
  if (!doubleEquals(result, lapTimer.haversine(0.0, 0.0, 1.0, 1.0))) {
    return false;
  }
  return true;
}
// Test case 4: Point is closest to the end point of the line segment
bool testPointLineSegmentDistance4() {
  double result = lapTimer.pointLineSegmentDistance(4.0, 4.0, 1.0, 1.0, 3.0, 3.0);
  if (!doubleEquals(result, lapTimer.haversine(4.0, 4.0, 3.0, 3.0))) {
    return false;
  }
  return true;
}


// automates some of the loop for testing purposes
void lapTimerTestLoop(GpsCords cords, float altitudeMeters, float speedKnots) {
  lapTimer.updateCurrentTime(millis());
  lapTimer.loop(cords.lat, cords.lng, altitudeMeters, speedKnots);
}
// 0 = north | 1 = east | 2 = south | 3 = west
void incrementTimerLoop(GpsCords &cords, double metersPerMove, int moveCount, int direction = 0, float speedKnots = 10, float altitudeMeters = 50) {
  for (int i = 0; i < moveCount; i++) {
  
    switch(direction) {
      case 0:
        cords = moveNorth(cords, metersPerMove);
        break;
      case 1:
        cords = moveEast(cords, metersPerMove);
        break;
      case 2:
        cords = moveSouth(cords, metersPerMove);
        break;
      case 3:
        cords = moveWest(cords, metersPerMove);
        break;
      default:
        cords = moveNorth(cords, metersPerMove);
        break;
    };

    lapTimerTestLoop(cords, altitudeMeters, speedKnots);
  }
}


bool testNotCrossingNextToLine1() {
  GpsCords testPoint = moveWest(finishLineMidPoint, 10);
  lapTimerTestLoop(testPoint, 50, 5);
  if (lapTimer.getCrossing()) {
    return false;
  }
  return true;
}

bool testNotCrossingNextToLine2() {
  GpsCords testPoint = moveEast(finishLineMidPoint, 10);
  lapTimerTestLoop(testPoint, 50, 5);
  if (lapTimer.getCrossing()) {
    return false;
  }
  return true;
}

bool testNotCrossingNextToLine3() {
  GpsCords testPoint = moveNorth(finishLineMidPoint, 10);
  lapTimerTestLoop(testPoint, 50, 5);
  if (lapTimer.getCrossing()) {
    return false;
  }
  return true;
}

bool testNotCrossingNextToLine4() {
  GpsCords testPoint = moveSouth(finishLineMidPoint, CROSSING_THRESHOLD_METERS + 0.25);
  lapTimerTestLoop(testPoint, 50, 5);
  if (lapTimer.getCrossing()) {
    return false;
  }
  return true;
}

bool testNotCrossingNextToLine5() {
  GpsCords testPoint = moveNorth(finishLineMidPoint, CROSSING_THRESHOLD_METERS + 1);
  lapTimerTestLoop(testPoint, 50, 5);
  if (lapTimer.getCrossing()) {
    return false;
  }
  return true;
}

bool testNotCrossingNextToLine6() {
  GpsCords testPoint = moveSouth(finishLineMidPoint, CROSSING_THRESHOLD_METERS + 1);
  lapTimerTestLoop(testPoint, 50, 5);
  if (lapTimer.getCrossing()) {
    return false;
  }
  return true;
}

bool testInCrossingThreshold() {
  GpsCords testPoint = moveSouth(finishLineMidPoint, CROSSING_THRESHOLD_METERS - 1);
  lapTimerTestLoop(testPoint, 50, 5);
  if (!lapTimer.getCrossing()) {
    return false;
  }
  return true;
}


bool testRaceStarted() {

  // first one outside of threshold
  GpsCords testPoint = moveSouth(finishLineMidPoint, CROSSING_THRESHOLD_METERS + 1);
  lapTimerTestLoop(testPoint, 50, 5);

  // increment point north 1 meter, 15 times
  incrementTimerLoop(testPoint, 1, 15);

  // last one outside of theshold
  testPoint = moveNorth(finishLineMidPoint, CROSSING_THRESHOLD_METERS + 1);
  lapTimerTestLoop(testPoint, 50, 5);

  if (!lapTimer.getRaceStarted()) {
    return false;
  }

  return true;
}

bool testLapDetection() {
  // first one outside of threshold
  GpsCords testPoint = moveSouth(finishLineMidPoint, CROSSING_THRESHOLD_METERS + 1);
  lapTimerTestLoop(testPoint, 50, 5);

  // increment point north 1 meter, 15 times
  incrementTimerLoop(testPoint, 1, 15);

  // last one outside of theshold
  testPoint = moveNorth(finishLineMidPoint, CROSSING_THRESHOLD_METERS + 1);
  lapTimerTestLoop(testPoint, 50, 5);

  // race started, now do another lap
  // first one outside of threshold
  testPoint = moveSouth(finishLineMidPoint, CROSSING_THRESHOLD_METERS + 1);
  lapTimerTestLoop(testPoint, 50, 5);

  // increment point north 1 meter, 15 times
  incrementTimerLoop(testPoint, 1, 15);

  // last one outside of theshold
  testPoint = moveNorth(finishLineMidPoint, CROSSING_THRESHOLD_METERS + 1);
  lapTimerTestLoop(testPoint, 50, 5);

  if (lapTimer.getLaps() <= 0) {
    return false;
  }

  return true;
}

#ifdef DOVES_UNIT_TEST
// Test case 1: t = 0
bool testCatmullRom1() {
  double result1 = lapTimer.catmullRom(1.0, 2.0, 3.0, 4.0, 0.0);
  if (abs(result1 - 2.0) > 1e-6) {
    return false;
  }
  return true;
}
// Test case 2: t = 1
bool testCatmullRom2() {
  double result2 = lapTimer.catmullRom(1.0, 2.0, 3.0, 4.0, 1.0);
  if (abs(result2 - 3.0) > 1e-6) {
    return false;
  }
  return true;
}
// Test case 3: t = 0.5 (intermediate value)
bool testCatmullRom3() {
  double result3 = lapTimer.catmullRom(1.0, 2.0, 3.0, 4.0, 0.5);
  double expected3 = 2.5; // Expected result calculated manually
  if (abs(result3 - expected3) > 1e-6) {
    return false;
  }
  return true;
}


// Test case 1: Equal distances and speeds
bool testInterpolateWeight1() {
  double result = lapTimer.interpolateWeight(10.0, 10.0, 30.0, 30.0);
  if (!doubleEquals(result, 0.5)) {
    return false;
  }
  return true;
}
// Test case 2: Unequal distances and equal speeds
bool testInterpolateWeight2() {
  double result = lapTimer.interpolateWeight(20.0, 10.0, 30.0, 30.0);
  if (!doubleEquals(result, 0.6666666666666666)) {
    return false;
  }
  return true;
}
// Test case 3: Equal distances and unequal speeds
bool testInterpolateWeight3() {
  double result = lapTimer.interpolateWeight(10.0, 10.0, 20.0, 40.0);
  if (!doubleEquals(result, 0.6666666666666666)) {
    return false;
  }
  return true;
}
// Test case 4: Unequal distances and speeds
bool testInterpolateWeight4() {
  double result = lapTimer.interpolateWeight(10.0, 20.0, 20.0f, 40.0f);  
  if (!doubleEquals(result, 0.5)) {
    return false;
  }
  return true;
}


// Help build buffer array for interpolation testing
void buildBuffer(crossingPointBufferEntry *testBuffer, const int bufferSize = 10, bool alterDistance = false, bool alterSpeed = false) {
  // keep track of "current" stats
  float metersToMove = 2;
  float multiplier = 0.175;

  float currentSpeed = 20;
  float currentOdometer = 1000;
  unsigned long currentTime = 10000;

  // define starting point for mock data
  GpsCords currentPoint = moveSouth(finishLineMidPoint, (bufferSize / metersToMove) + metersToMove + (bufferSize * multiplier));
  for (int i = 0; i < bufferSize; i++) {
    // debug(currentPoint.lat, 10);
    // debug(", ");
    // debugln(currentPoint.lng, 10);
    testBuffer[i] = {currentPoint.lat, currentPoint.lng, currentTime, currentOdometer, currentSpeed};

    // update for next loop
    currentPoint = moveNorth(currentPoint, metersToMove);
    currentOdometer += metersToMove;
    currentTime += 100;
    if (alterDistance) {
      metersToMove += metersToMove * multiplier; 
    }
    if (alterSpeed) {
      currentSpeed += currentSpeed * multiplier;
    }
  }
}

// constant distance constant speed
bool testInterpolationOnCurve1() {
  // Populate the crossingPointBuffer with test data
  const int bufferSize = 10;
  crossingPointBufferEntry testBuffer[bufferSize];
  buildBuffer(testBuffer, bufferSize, false, false);
  
  // Set the buffer for the lapTimer
  for (int i = 0; i < sizeof(testBuffer) / sizeof(testBuffer[0]); i++) {
    lapTimer.crossingPointBuffer[i] = testBuffer[i];
    int crossingPointSideOfLine = lapTimer.pointOnSideOfLine(testBuffer[i].lat, testBuffer[i].lng, crossingPointALat, crossingPointALng, crossingPointBLat, crossingPointBLng);
    double crossingPointDistanceToLine = lapTimer.pointLineSegmentDistance(testBuffer[i].lat, testBuffer[i].lng, crossingPointALat, crossingPointALng, crossingPointBLat, crossingPointBLng);
    // debug("(");
    // debug(crossingPointSideOfLine);
    // debug(") ");
    // debug("crossingPointDistanceToLine: ");
    // debugln(crossingPointDistanceToLine, 12);
  }
  lapTimer.crossingPointBufferIndex = bufferSize;
  lapTimer.crossingPointBufferFull = false;

  // Variables to store the crossing point's latitude, longitude, time, and odometer
  double crossingLat;
  double crossingLng;
  unsigned long crossingTime;
  double crossingOdometer;

  // Call the function
  lapTimer.interpolateCrossingPoint(crossingLat, crossingLng, crossingTime, crossingOdometer, crossingPointALat, crossingPointALng, crossingPointBLat, crossingPointBLng);

  // Check if the function returns the expected values (tolerance can be adjusted)
  double tolerance = 1e-6;
  bool testsPassed = true;
  double crossingPointDistanceToLine = lapTimer.pointLineSegmentDistance(crossingLat, crossingLng, crossingPointALat, crossingPointALng, crossingPointBLat, crossingPointBLng);
  if (fabs(crossingPointDistanceToLine) > tolerance) {
    debug("crossingPointDistanceToLine: ");
    debugln(crossingPointDistanceToLine, 8);
    testsPassed = false;
  }

  return testsPassed;
}
// constant distance changing speed
bool testInterpolationOnCurve2() {
  // Populate the crossingPointBuffer with test data
  const int bufferSize = 10;
  crossingPointBufferEntry testBuffer[bufferSize];
  buildBuffer(testBuffer, bufferSize, false, true);
  
  // Set the buffer for the lapTimer
  for (int i = 0; i < sizeof(testBuffer) / sizeof(testBuffer[0]); i++) {
    lapTimer.crossingPointBuffer[i] = testBuffer[i];

    int crossingPointSideOfLine = lapTimer.pointOnSideOfLine(testBuffer[i].lat, testBuffer[i].lng, crossingPointALat, crossingPointALng, crossingPointBLat, crossingPointBLng);
    double crossingPointDistanceToLine = lapTimer.pointLineSegmentDistance(testBuffer[i].lat, testBuffer[i].lng, crossingPointALat, crossingPointALng, crossingPointBLat, crossingPointBLng);
    // debug("(");
    // debug(crossingPointSideOfLine);
    // debug(") ");
    // debug("crossingPointDistanceToLine: ");
    // debugln(crossingPointDistanceToLine, 12);
  }
  lapTimer.crossingPointBufferIndex = bufferSize;
  lapTimer.crossingPointBufferFull = false;

  // Variables to store the crossing point's latitude, longitude, time, and odometer
  double crossingLat;
  double crossingLng;
  unsigned long crossingTime;
  double crossingOdometer;

  // Call the function
  lapTimer.interpolateCrossingPoint(crossingLat, crossingLng, crossingTime, crossingOdometer, crossingPointALat, crossingPointALng, crossingPointBLat, crossingPointBLng);

  // Check if the function returns the expected values (tolerance can be adjusted)
  double tolerance = 0.08;
  bool testsPassed = true;

  double crossingPointDistanceToLine = lapTimer.pointLineSegmentDistance(crossingLat, crossingLng, crossingPointALat, crossingPointALng, crossingPointBLat, crossingPointBLng);
  if (fabs(crossingPointDistanceToLine) > tolerance) {
    debug("crossingPointDistanceToLine: ");
    debugln(crossingPointDistanceToLine, 8);
    testsPassed = false;
  }

  return testsPassed;
}
// changing distance constant speed
bool testInterpolationOnCurve3() {
  // Populate the crossingPointBuffer with test data
  const int bufferSize = 10;
  crossingPointBufferEntry testBuffer[bufferSize];
  buildBuffer(testBuffer, bufferSize, true, false);
  
  // Set the buffer for the lapTimer
  for (int i = 0; i < sizeof(testBuffer) / sizeof(testBuffer[0]); i++) {
    lapTimer.crossingPointBuffer[i] = testBuffer[i];
    // int crossingPointSideOfLine = lapTimer.pointOnSideOfLine(testBuffer[i].lat, testBuffer[i].lng, crossingPointALat, crossingPointALng, crossingPointBLat, crossingPointBLng);
    // double crossingPointDistanceToLine = lapTimer.pointLineSegmentDistance(testBuffer[i].lat, testBuffer[i].lng, crossingPointALat, crossingPointALng, crossingPointBLat, crossingPointBLng);
    // debug("(");
    // debug(crossingPointSideOfLine);
    // debug(") ");
    // debug("crossingPointDistanceToLine: ");
    // debugln(crossingPointDistanceToLine, 12);
  }
  lapTimer.crossingPointBufferIndex = bufferSize;
  lapTimer.crossingPointBufferFull = false;

  // Variables to store the crossing point's latitude, longitude, time, and odometer
  double crossingLat;
  double crossingLng;
  unsigned long crossingTime;
  double crossingOdometer;

  // Call the function
  lapTimer.interpolateCrossingPoint(crossingLat, crossingLng, crossingTime, crossingOdometer, crossingPointALat, crossingPointALng, crossingPointBLat, crossingPointBLng);

  // Check if the function returns the expected values (tolerance can be adjusted)
  double tolerance = 0.08;
  bool testsPassed = true;

  double crossingPointDistanceToLine = lapTimer.pointLineSegmentDistance(crossingLat, crossingLng, crossingPointALat, crossingPointALng, crossingPointBLat, crossingPointBLng);
  if (fabs(crossingPointDistanceToLine) > tolerance) {
    debug("crossingPointDistanceToLine: ");
    debugln(crossingPointDistanceToLine, 8);
    testsPassed = false;
  }

  return testsPassed;
}
// changing distance changing speed
bool testInterpolationOnCurve4() {
  // Populate the crossingPointBuffer with test data
  const int bufferSize = 10;
  crossingPointBufferEntry testBuffer[bufferSize];
  buildBuffer(testBuffer, bufferSize, true, true);
  
  // Set the buffer for the lapTimer
  for (int i = 0; i < sizeof(testBuffer) / sizeof(testBuffer[0]); i++) {
    lapTimer.crossingPointBuffer[i] = testBuffer[i];
    // int crossingPointSideOfLine = lapTimer.pointOnSideOfLine(testBuffer[i].lat, testBuffer[i].lng, crossingPointALat, crossingPointALng, crossingPointBLat, crossingPointBLng);
    // double crossingPointDistanceToLine = lapTimer.pointLineSegmentDistance(testBuffer[i].lat, testBuffer[i].lng, crossingPointALat, crossingPointALng, crossingPointBLat, crossingPointBLng);
    // debug("(");
    // debug(crossingPointSideOfLine);
    // debug(") ");
    // debug("crossingPointDistanceToLine: ");
    // debugln(crossingPointDistanceToLine, 12);
  }
  lapTimer.crossingPointBufferIndex = bufferSize;
  lapTimer.crossingPointBufferFull = false;

  // Variables to store the crossing point's latitude, longitude, time, and odometer
  double crossingLat;
  double crossingLng;
  unsigned long crossingTime;
  double crossingOdometer;

  // Call the function
  lapTimer.interpolateCrossingPoint(crossingLat, crossingLng, crossingTime, crossingOdometer, crossingPointALat, crossingPointALng, crossingPointBLat, crossingPointBLng);

  // Check if the function returns the expected values (tolerance can be adjusted)
  double tolerance = 0.08;
  bool testsPassed = true;

  double crossingPointDistanceToLine = lapTimer.pointLineSegmentDistance(crossingLat, crossingLng, crossingPointALat, crossingPointALng, crossingPointBLat, crossingPointBLng);
  if (fabs(crossingPointDistanceToLine) > tolerance) {
    debug("crossingPointDistanceToLine: ");
    debugln(crossingPointDistanceToLine, 8);
    testsPassed = false;
  }

  return testsPassed;
}


// constant distance constant speed
bool testInterpolationLinear1() {
  lapTimer.forceLinearInterpolation();
  // Populate the crossingPointBuffer with test data
  const int bufferSize = 10;
  crossingPointBufferEntry testBuffer[bufferSize];
  buildBuffer(testBuffer, bufferSize, false, false);
  
  // Set the buffer for the lapTimer
  for (int i = 0; i < sizeof(testBuffer) / sizeof(testBuffer[0]); i++) {
    lapTimer.crossingPointBuffer[i] = testBuffer[i];
    int crossingPointSideOfLine = lapTimer.pointOnSideOfLine(testBuffer[i].lat, testBuffer[i].lng, crossingPointALat, crossingPointALng, crossingPointBLat, crossingPointBLng);
    double crossingPointDistanceToLine = lapTimer.pointLineSegmentDistance(testBuffer[i].lat, testBuffer[i].lng, crossingPointALat, crossingPointALng, crossingPointBLat, crossingPointBLng);
    // debug("(");
    // debug(crossingPointSideOfLine);
    // debug(") ");
    // debug("crossingPointDistanceToLine: ");
    // debugln(crossingPointDistanceToLine, 12);
  }
  lapTimer.crossingPointBufferIndex = bufferSize;
  lapTimer.crossingPointBufferFull = false;

  // Variables to store the crossing point's latitude, longitude, time, and odometer
  double crossingLat;
  double crossingLng;
  unsigned long crossingTime;
  double crossingOdometer;

  // Call the function
  lapTimer.interpolateCrossingPoint(crossingLat, crossingLng, crossingTime, crossingOdometer, crossingPointALat, crossingPointALng, crossingPointBLat, crossingPointBLng);

  // Check if the function returns the expected values (tolerance can be adjusted)
  double tolerance = 1e-6;
  bool testsPassed = true;
  double crossingPointDistanceToLine = lapTimer.pointLineSegmentDistance(crossingLat, crossingLng, crossingPointALat, crossingPointALng, crossingPointBLat, crossingPointBLng);
  if (fabs(crossingPointDistanceToLine) > tolerance) {
    debug("crossingPointDistanceToLine: ");
    debugln(crossingPointDistanceToLine, 8);
    testsPassed = false;
  }

  return testsPassed;
}
// constant distance changing speed
bool testInterpolationLinear2() {
  lapTimer.forceLinearInterpolation();
  // Populate the crossingPointBuffer with test data
  const int bufferSize = 10;
  crossingPointBufferEntry testBuffer[bufferSize];
  buildBuffer(testBuffer, bufferSize, false, true);
  
  // Set the buffer for the lapTimer
  for (int i = 0; i < sizeof(testBuffer) / sizeof(testBuffer[0]); i++) {
    lapTimer.crossingPointBuffer[i] = testBuffer[i];

    int crossingPointSideOfLine = lapTimer.pointOnSideOfLine(testBuffer[i].lat, testBuffer[i].lng, crossingPointALat, crossingPointALng, crossingPointBLat, crossingPointBLng);
    double crossingPointDistanceToLine = lapTimer.pointLineSegmentDistance(testBuffer[i].lat, testBuffer[i].lng, crossingPointALat, crossingPointALng, crossingPointBLat, crossingPointBLng);
    // debug("(");
    // debug(crossingPointSideOfLine);
    // debug(") ");
    // debug("crossingPointDistanceToLine: ");
    // debugln(crossingPointDistanceToLine, 12);
  }
  lapTimer.crossingPointBufferIndex = bufferSize;
  lapTimer.crossingPointBufferFull = false;

  // Variables to store the crossing point's latitude, longitude, time, and odometer
  double crossingLat;
  double crossingLng;
  unsigned long crossingTime;
  double crossingOdometer;

  // Call the function
  lapTimer.interpolateCrossingPoint(crossingLat, crossingLng, crossingTime, crossingOdometer, crossingPointALat, crossingPointALng, crossingPointBLat, crossingPointBLng);

  // Check if the function returns the expected values (tolerance can be adjusted)
  double tolerance = 0.08;
  bool testsPassed = true;

  double crossingPointDistanceToLine = lapTimer.pointLineSegmentDistance(crossingLat, crossingLng, crossingPointALat, crossingPointALng, crossingPointBLat, crossingPointBLng);
  if (fabs(crossingPointDistanceToLine) > tolerance) {
    debug("crossingPointDistanceToLine: ");
    debugln(crossingPointDistanceToLine, 8);
    testsPassed = false;
  }

  return testsPassed;
}
// changing distance constant speed
bool testInterpolationLinear3() {
  lapTimer.forceLinearInterpolation();
  // Populate the crossingPointBuffer with test data
  const int bufferSize = 10;
  crossingPointBufferEntry testBuffer[bufferSize];
  buildBuffer(testBuffer, bufferSize, true, false);
  
  // Set the buffer for the lapTimer
  for (int i = 0; i < sizeof(testBuffer) / sizeof(testBuffer[0]); i++) {
    lapTimer.crossingPointBuffer[i] = testBuffer[i];
    // int crossingPointSideOfLine = lapTimer.pointOnSideOfLine(testBuffer[i].lat, testBuffer[i].lng, crossingPointALat, crossingPointALng, crossingPointBLat, crossingPointBLng);
    // double crossingPointDistanceToLine = lapTimer.pointLineSegmentDistance(testBuffer[i].lat, testBuffer[i].lng, crossingPointALat, crossingPointALng, crossingPointBLat, crossingPointBLng);
    // debug("(");
    // debug(crossingPointSideOfLine);
    // debug(") ");
    // debug("crossingPointDistanceToLine: ");
    // debugln(crossingPointDistanceToLine, 12);
  }
  lapTimer.crossingPointBufferIndex = bufferSize;
  lapTimer.crossingPointBufferFull = false;

  // Variables to store the crossing point's latitude, longitude, time, and odometer
  double crossingLat;
  double crossingLng;
  unsigned long crossingTime;
  double crossingOdometer;

  // Call the function
  lapTimer.interpolateCrossingPoint(crossingLat, crossingLng, crossingTime, crossingOdometer, crossingPointALat, crossingPointALng, crossingPointBLat, crossingPointBLng);

  // Check if the function returns the expected values (tolerance can be adjusted)
  double tolerance = 1e-6;
  bool testsPassed = true;

  double crossingPointDistanceToLine = lapTimer.pointLineSegmentDistance(crossingLat, crossingLng, crossingPointALat, crossingPointALng, crossingPointBLat, crossingPointBLng);
  if (fabs(crossingPointDistanceToLine) > tolerance) {
    debug("crossingPointDistanceToLine: ");
    debugln(crossingPointDistanceToLine, 8);
    testsPassed = false;
  }

  return testsPassed;
}
// changing distance changing speed
bool testInterpolationLinear4() {
  lapTimer.forceLinearInterpolation();
  // Populate the crossingPointBuffer with test data
  const int bufferSize = 10;
  crossingPointBufferEntry testBuffer[bufferSize];
  buildBuffer(testBuffer, bufferSize, true, true);
  
  // Set the buffer for the lapTimer
  for (int i = 0; i < sizeof(testBuffer) / sizeof(testBuffer[0]); i++) {
    lapTimer.crossingPointBuffer[i] = testBuffer[i];
    // int crossingPointSideOfLine = lapTimer.pointOnSideOfLine(testBuffer[i].lat, testBuffer[i].lng, crossingPointALat, crossingPointALng, crossingPointBLat, crossingPointBLng);
    // double crossingPointDistanceToLine = lapTimer.pointLineSegmentDistance(testBuffer[i].lat, testBuffer[i].lng, crossingPointALat, crossingPointALng, crossingPointBLat, crossingPointBLng);
    // debug("(");
    // debug(crossingPointSideOfLine);
    // debug(") ");
    // debug("crossingPointDistanceToLine: ");
    // debugln(crossingPointDistanceToLine, 12);
  }
  lapTimer.crossingPointBufferIndex = bufferSize;
  lapTimer.crossingPointBufferFull = false;

  // Variables to store the crossing point's latitude, longitude, time, and odometer
  double crossingLat;
  double crossingLng;
  unsigned long crossingTime;
  double crossingOdometer;

  // Call the function
  lapTimer.interpolateCrossingPoint(crossingLat, crossingLng, crossingTime, crossingOdometer, crossingPointALat, crossingPointALng, crossingPointBLat, crossingPointBLng);

  // Check if the function returns the expected values (tolerance can be adjusted)
  double tolerance = 0.15;
  bool testsPassed = true;

  double crossingPointDistanceToLine = lapTimer.pointLineSegmentDistance(crossingLat, crossingLng, crossingPointALat, crossingPointALng, crossingPointBLat, crossingPointBLng);
  if (fabs(crossingPointDistanceToLine) > tolerance) {
    debug("crossingPointDistanceToLine: ");
    debugln(crossingPointDistanceToLine, 8);
    testsPassed = false;
  }

  return testsPassed;
}
#endif