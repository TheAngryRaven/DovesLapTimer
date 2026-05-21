/**
 * Sector Timing Example for DovesLapTimer
 *
 * This example demonstrates 3-sector split timing using a SYNTHETIC GPS
 * track — no GPS hardware required. The library is driven through three
 * simulated laps around a 100m × 100m square loop so you can see sector
 * times, best times, and optimal-lap calculation light up on Serial.
 *
 * Sector layout:
 * - Sector 1: Start/Finish -> Sector 2 line
 * - Sector 2: Sector 2 line -> Sector 3 line
 * - Sector 3: Sector 3 line -> Start/Finish
 *
 * To adapt for a real track:
 *   1. Replace the four pairs of crossing-line coordinates below with
 *      your own (Google Maps right-click -> copy lat,lng).
 *   2. Delete generateSyntheticFix() and the LAPS_TO_RUN stop condition.
 *   3. In loop(), replace the synthetic GPS block with reads from your
 *      GPS module: feed lat, lng, altitude (m), and speed (knots) into
 *      lapTimer.loop(), and ms-since-midnight into lapTimer.updateCurrentTime().
 */

#include <DovesLapTimer.h>

// Adafruit nRF52 BSP (e.g. Seeed XIAO nRF52840) routes Serial through USB CDC
// via TinyUSB. The BSP defines USE_TINYUSB when that stack is selected; the
// header below pulls in the TinyUSB implementation so Serial links.
// No-op on AVR / ESP32 / mbed cores.
#ifdef USE_TINYUSB
  #include <Adafruit_TinyUSB.h>
#endif

// =========================================================================
// Synthetic track configuration
// =========================================================================
//
// 100m × 100m square loop traced counter-clockwise from the SW corner.
// Driver heading on each side: south -> east, east -> north, north -> west,
// west -> south. Each lap crosses S/F, S2, S3, then S/F again to close.
//
//   NW ─────── S3 line ─────── NE
//    │                          │
//    │                          S
//    W       (counter-          2
//    │        clockwise)        │
//    │                          │
//   SW ─────── S/F line ─────── SE

const double TRACK_SOUTH_LAT = 28.41265;
const double TRACK_NORTH_LAT = 28.41355;
const double TRACK_WEST_LNG  = -81.37970;
const double TRACK_EAST_LNG  = -81.37870;

const unsigned int  STEPS_PER_LAP   = 80;
const unsigned long SIM_MS_PER_STEP = 200;     // sim-time GPS @ 5Hz
const float         SIM_SPEED_KNOTS = 48.0f;   // ~90 km/h
const float         SIM_ALT_METERS  = 50.0f;
const unsigned int  LAPS_TO_RUN     = 3;

// Crossing lines positioned so a CCW driver hits S/F -> S2 -> S3 -> S/F.

// Start/finish: south side, perpendicular to east-bound travel
const double startFinishPointALat = 28.41255;
const double startFinishPointALng = -81.37920;
const double startFinishPointBLat = 28.41275;
const double startFinishPointBLng = -81.37920;

// Sector 2: east side, perpendicular to north-bound travel
const double sector2PointALat = 28.41310;
const double sector2PointALng = -81.37860;
const double sector2PointBLat = 28.41310;
const double sector2PointBLng = -81.37880;

// Sector 3: north side, perpendicular to west-bound travel
const double sector3PointALat = 28.41345;
const double sector3PointALng = -81.37925;
const double sector3PointBLat = 28.41365;
const double sector3PointBLng = -81.37925;

// =========================================================================
// Lap timer + simulation state
// =========================================================================

const double crossingThresholdMeters = 7.0;
DovesLapTimer lapTimer(crossingThresholdMeters, &Serial);

unsigned long stepCount = 0;
unsigned long simTimeMs = 3600000UL;  // start sim clock at 01:00 AM

// Compute (lat, lng) at a given step along the synthetic CCW loop.
void generateSyntheticFix(unsigned long step, double &lat, double &lng) {
  const unsigned int sideSteps = STEPS_PER_LAP / 4;
  const unsigned int phase = step % STEPS_PER_LAP;
  const double t = (phase % sideSteps) / (double)sideSteps;

  if (phase < sideSteps) {                  // south side, east-bound
    lat = TRACK_SOUTH_LAT;
    lng = TRACK_WEST_LNG + t * (TRACK_EAST_LNG - TRACK_WEST_LNG);
  } else if (phase < 2 * sideSteps) {       // east side, north-bound
    lat = TRACK_SOUTH_LAT + t * (TRACK_NORTH_LAT - TRACK_SOUTH_LAT);
    lng = TRACK_EAST_LNG;
  } else if (phase < 3 * sideSteps) {       // north side, west-bound
    lat = TRACK_NORTH_LAT;
    lng = TRACK_EAST_LNG + t * (TRACK_WEST_LNG - TRACK_EAST_LNG);
  } else {                                   // west side, south-bound
    lat = TRACK_NORTH_LAT + t * (TRACK_SOUTH_LAT - TRACK_NORTH_LAT);
    lng = TRACK_WEST_LNG;
  }
}

void displaySectorInfo();

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("DovesLapTimer - Sector Timing Example");
  Serial.println("======================================");
  Serial.println("Driving a synthetic CCW square track. No GPS hardware needed.");
  Serial.println();

  lapTimer.setStartFinishLine(startFinishPointALat, startFinishPointALng,
                              startFinishPointBLat, startFinishPointBLng);
  lapTimer.setSector2Line(sector2PointALat, sector2PointALng,
                          sector2PointBLat, sector2PointBLng);
  lapTimer.setSector3Line(sector3PointALat, sector3PointALng,
                          sector3PointBLat, sector3PointBLng);

  Serial.println("Sector lines configured. Driving...");
  Serial.println();
}

void loop() {
  // --- In a real application: replace this block with reads from your GPS ---
  double lat, lng;
  generateSyntheticFix(stepCount, lat, lng);
  lapTimer.updateCurrentTime(simTimeMs);
  lapTimer.loop(lat, lng, SIM_ALT_METERS, SIM_SPEED_KNOTS);
  stepCount++;
  simTimeMs += SIM_MS_PER_STEP;
  // --------------------------------------------------------------------------

  // Print only when lap or sector changes — keeps Serial output readable.
  static int lastLap = -1;
  static int lastSector = -1;
  const int curLap = (int)lapTimer.getLaps();
  const int curSector = lapTimer.getCurrentSector();
  if (curLap != lastLap || curSector != lastSector) {
    displaySectorInfo();
    lastLap = curLap;
    lastSector = curSector;
  }

  // Stop once the demo has produced enough data so output doesn't loop forever.
  if (curLap >= (int)LAPS_TO_RUN) {
    Serial.println();
    Serial.println("=== Demo complete ===");
    displaySectorInfo();
    while (true) {
      delay(1000);
    }
  }

  delay(50);  // real-time pacing
}

void displaySectorInfo() {
  if (!lapTimer.getRaceStarted()) {
    Serial.println("Waiting to cross start/finish line...");
    return;
  }

  // Display current lap information
  Serial.println("=== Current Lap ===");
  Serial.print("Lap: ");
  Serial.println(lapTimer.getLaps());
  Serial.print("Current Sector: ");
  Serial.println(lapTimer.getCurrentSector());
  Serial.print("Lap Time: ");
  Serial.print(lapTimer.getCurrentLapTime() / 1000.0, 3);
  Serial.println(" sec");

  // Display current lap sector times
  if (lapTimer.getCurrentLapSector1Time() > 0) {
    Serial.print("  Sector 1: ");
    Serial.print(lapTimer.getCurrentLapSector1Time() / 1000.0, 3);
    Serial.println(" sec");
  }
  if (lapTimer.getCurrentLapSector2Time() > 0) {
    Serial.print("  Sector 2: ");
    Serial.print(lapTimer.getCurrentLapSector2Time() / 1000.0, 3);
    Serial.println(" sec");
  }
  if (lapTimer.getCurrentLapSector3Time() > 0) {
    Serial.print("  Sector 3: ");
    Serial.print(lapTimer.getCurrentLapSector3Time() / 1000.0, 3);
    Serial.println(" sec");
  }

  Serial.println();

  // Display best lap and sector information
  if (lapTimer.areSectorLinesConfigured()) {
    Serial.println("=== Best Times ===");
    Serial.print("Best Lap: ");
    Serial.print(lapTimer.getBestLapTime() / 1000.0, 3);
    Serial.print(" sec (Lap ");
    Serial.print(lapTimer.getBestLapNumber());
    Serial.println(")");

    // Best sector times
    if (lapTimer.getBestSector1Time() > 0) {
      Serial.print("  Best Sector 1: ");
      Serial.print(lapTimer.getBestSector1Time() / 1000.0, 3);
      Serial.print(" sec (Lap ");
      Serial.print(lapTimer.getBestSector1LapNumber());
      Serial.println(")");
    }

    if (lapTimer.getBestSector2Time() > 0) {
      Serial.print("  Best Sector 2: ");
      Serial.print(lapTimer.getBestSector2Time() / 1000.0, 3);
      Serial.print(" sec (Lap ");
      Serial.print(lapTimer.getBestSector2LapNumber());
      Serial.println(")");
    }

    if (lapTimer.getBestSector3Time() > 0) {
      Serial.print("  Best Sector 3: ");
      Serial.print(lapTimer.getBestSector3Time() / 1000.0, 3);
      Serial.print(" sec (Lap ");
      Serial.print(lapTimer.getBestSector3LapNumber());
      Serial.println(")");
    }

    // Optimal lap time
    if (lapTimer.getOptimalLapTime() > 0) {
      Serial.print("  Optimal Lap: ");
      Serial.print(lapTimer.getOptimalLapTime() / 1000.0, 3);
      Serial.print(" sec");

      // Show difference from best lap
      if (lapTimer.getBestLapTime() > 0) {
        long delta = lapTimer.getOptimalLapTime() - lapTimer.getBestLapTime();
        Serial.print(" (");
        if (delta > 0) {
          Serial.print("+");
        }
        Serial.print(delta / 1000.0, 3);
        Serial.println(" sec from best)");
      } else {
        Serial.println();
      }
    }
  }

  Serial.println();
  Serial.println("================================");
  Serial.println();
}
