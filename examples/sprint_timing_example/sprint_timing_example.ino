/**
 * Sprint (Point-to-Point) Timing Example for DovesLapTimer's SprintTimer
 *
 * Demonstrates sprint-mode run timing — autocross / hillclimb style: a
 * START line and a SEPARATE FINISH line, no laps — using a SYNTHETIC GPS
 * course, no GPS hardware required. The timer is driven through three
 * simulated runs down a 500 m straight (loop back on a parallel return
 * leg between runs) so you can watch run times, split segments, best-run
 * and optimal tracking light up on Serial.
 *
 * Course layout (x = meters along the course):
 *
 *   0m ── START(50m) ── S2(150m) ── S3(300m) ── FINISH(450m) ── 500m
 *          |------------- timed run: 400m --------------|
 *   return leg 40m north of the course, outside every crossing zone
 *
 * Sprint semantics demonstrated:
 *   - a start-line crossing begins a run; a finish-line crossing ends it
 *   - crossing the start again MID-RUN cancels and restarts (botched course)
 *   - finish crossings with no active run are ignored (the return loop)
 *   - splits are optional: zero, one, or two lines are all legal
 *
 * To adapt for a real course:
 *   1. Replace the line coordinates below with your own
 *      (Google Maps right-click -> copy lat,lng).
 *   2. Delete the synthetic-drive block in loop().
 *   3. Feed real GPS: lat, lng, altitude (m), speed (knots) into
 *      sprintTimer.loop(), and ms-since-midnight into
 *      sprintTimer.updateCurrentTime().
 *
 * RAM note: each configured line owns an independent crossing buffer
 * (~2.9 KB each on 32-bit targets). SprintTimer targets modern MCUs
 * (nRF52840, ESP32, ...) — it does not fit small AVRs.
 */

#include <DovesLapTimer.h>
#include <SprintTimer.h>

// Adafruit nRF52 BSP (e.g. Seeed XIAO nRF52840) routes Serial through USB CDC
// via TinyUSB. The BSP defines USE_TINYUSB when that stack is selected; the
// header below pulls in the TinyUSB implementation so Serial links.
// No-op on ESP32 / mbed cores.
#ifdef USE_TINYUSB
  #include <Adafruit_TinyUSB.h>
#endif

// =========================================================================
// Synthetic course configuration
// =========================================================================

const double BASE_LAT = 28.40000;
const double BASE_LNG = -81.40000;
// meters -> degrees at BASE_LAT (haversine-consistent, R = 6371 km)
const double M_PER_DEG_LAT = 111194.9266;
const double M_PER_DEG_LNG = 111194.9266 * 0.879649;  // cos(28.4 deg)

double latAt(double yMeters) { return BASE_LAT + yMeters / M_PER_DEG_LAT; }
double lngAt(double xMeters) { return BASE_LNG + xMeters / M_PER_DEG_LNG; }

const double START_X  = 50.0;
const double S2_X     = 150.0;
const double S3_X     = 300.0;
const double FINISH_X = 450.0;
const double COURSE_END_X = 500.0;
const double RETURN_Y = 40.0;          // return-leg offset, outside all zones
const double LINE_HALF_WIDTH = 10.0;   // lines span y = -10m .. +10m

const double        STEP_METERS     = 5.0;
const unsigned long SIM_MS_PER_STEP = 200;    // sim-time GPS @ 5 Hz
const float         SIM_SPEED_KNOTS = 48.0f;  // ~90 km/h
const float         SIM_ALT_METERS  = 50.0f;
const unsigned int  RUNS_TO_DO      = 3;

SprintTimer sprintTimer(7.0);

// Synthetic drive state
double simX = 0, simY = 0;
unsigned long simTimeMs = 0;
unsigned int runsSeen = 0;
int simPhase = 0;  // 0 = out (east along course), 1..3 = return loop legs
bool done = false;

void feedFix() {
  sprintTimer.updateCurrentTime(simTimeMs);
  sprintTimer.loop(latAt(simY), lngAt(simX), SIM_ALT_METERS, SIM_SPEED_KNOTS);
  simTimeMs += SIM_MS_PER_STEP;
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 4000) { }

  Serial.println(F("=== SprintTimer point-to-point example ==="));

  // Start line: a run begins when it is crossed (even mid-run: that
  // cancels the current run and starts a fresh one).
  sprintTimer.setStartLine(latAt(-LINE_HALF_WIDTH), lngAt(START_X),
                           latAt(LINE_HALF_WIDTH),  lngAt(START_X));
  // Finish line: completes the run (ignored when no run is active).
  sprintTimer.setFinishLine(latAt(-LINE_HALF_WIDTH), lngAt(FINISH_X),
                            latAt(LINE_HALF_WIDTH),  lngAt(FINISH_X));
  // Optional splits — configure zero, one, or both.
  sprintTimer.setSector2Line(latAt(-LINE_HALF_WIDTH), lngAt(S2_X),
                             latAt(LINE_HALF_WIDTH),  lngAt(S2_X));
  sprintTimer.setSector3Line(latAt(-LINE_HALF_WIDTH), lngAt(S3_X),
                             latAt(LINE_HALF_WIDTH),  lngAt(S3_X));
  sprintTimer.forceLinearInterpolation();

  Serial.println(F("Course: start@50m  S2@150m  S3@300m  finish@450m (400m run)"));
  Serial.println();
}

void printRun() {
  Serial.print(F("Run "));
  Serial.print(sprintTimer.getRuns());
  Serial.print(F(" complete: "));
  Serial.print(sprintTimer.getLastRunTime());
  Serial.print(F(" ms  ("));
  Serial.print(sprintTimer.getLastRunDistance());
  Serial.println(F(" m)"));

  Serial.print(F("  segments: "));
  Serial.print(sprintTimer.getCurrentLapSector1Time());
  Serial.print(F(" / "));
  Serial.print(sprintTimer.getCurrentLapSector2Time());
  Serial.print(F(" / "));
  Serial.println(sprintTimer.getCurrentLapSector3Time());

  Serial.print(F("  best run: "));
  Serial.print(sprintTimer.getBestRunTime());
  Serial.print(F(" ms (run "));
  Serial.print(sprintTimer.getBestRunNumber());
  Serial.print(F(")  optimal: "));
  Serial.print(sprintTimer.getOptimalLapTime());
  Serial.println(F(" ms"));
  Serial.println();
}

void loop() {
  if (done) return;

  // ---- Synthetic drive: out along the course, then loop back north ----
  if (simPhase == 0) {          // out: east along y=0, through every line
    simX += STEP_METERS;
    if (simX >= COURSE_END_X) simPhase = 1;
  } else if (simPhase == 1) {   // north to the return leg
    simY += STEP_METERS;
    if (simY >= RETURN_Y) simPhase = 2;
  } else if (simPhase == 2) {   // west back to the course start
    simX -= STEP_METERS;
    if (simX <= 0) simPhase = 3;
  } else {                      // south, rejoin the course
    simY -= STEP_METERS;
    if (simY <= 0) simPhase = 0;
  }
  feedFix();
  // ---- End synthetic drive ----

  if ((unsigned int)sprintTimer.getRuns() > runsSeen) {
    runsSeen = sprintTimer.getRuns();
    printRun();
    if (runsSeen >= RUNS_TO_DO) {
      Serial.println(F("=== Done. ==="));
      done = true;
    }
  }

  delay(2);  // keep the sim snappy but readable on a scope
}
