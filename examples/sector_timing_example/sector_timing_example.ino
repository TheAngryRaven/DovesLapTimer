/**
 * Sector Timing Example for DovesLapTimer
 *
 * This example demonstrates the sector split timing feature.
 * Configure your start/finish line and two sector lines to get
 * sector times and calculate an optimal lap time.
 *
 * Sector layout:
 * - Sector 1: Start/Finish → Sector 2 line
 * - Sector 2: Sector 2 line → Sector 3 line
 * - Sector 3: Sector 3 line → Start/Finish
 */

#include <DovesLapTimer.h>

// Configure your start/finish line coordinates
// Use Google Maps: right-click → copy coordinates
const double startFinishPointALat = 28.41270817056385;
const double startFinishPointALng = -81.37973266418031;
const double startFinishPointBLat = 28.41273038679321;
const double startFinishPointBLng = -81.37957048753776;

// Configure sector 2 line coordinates
// This should be roughly 1/3 of the way around the track
const double sector2PointALat = 28.41300000000000;  // CHANGE THESE
const double sector2PointALng = -81.38000000000000;  // TO YOUR
const double sector2PointBLat = 28.41302000000000;  // TRACK
const double sector2PointBLng = -81.37998000000000;  // COORDINATES

// Configure sector 3 line coordinates
// This should be roughly 2/3 of the way around the track
const double sector3PointALat = 28.41280000000000;  // CHANGE THESE
const double sector3PointALng = -81.37900000000000;  // TO YOUR
const double sector3PointBLat = 28.41282000000000;  // TRACK
const double sector3PointBLng = -81.37898000000000;  // COORDINATES

// Crossing threshold in meters
double crossingThresholdMeters = 7.0;

// Create lap timer instance
DovesLapTimer lapTimer(crossingThresholdMeters, &Serial);

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("DovesLapTimer - Sector Timing Example");
  Serial.println("======================================");

  // Configure start/finish line
  lapTimer.setStartFinishLine(startFinishPointALat, startFinishPointALng,
                               startFinishPointBLat, startFinishPointBLng);

  // Configure sector lines
  lapTimer.setSector2Line(sector2PointALat, sector2PointALng,
                          sector2PointBLat, sector2PointBLng);
  lapTimer.setSector3Line(sector3PointALat, sector3PointALng,
                          sector3PointBLat, sector3PointBLng);

  Serial.println("Sector lines configured!");
  Serial.println("Waiting for GPS data...");
  Serial.println();
}

void loop() {
  // In a real application, you would:
  // 1. Read GPS data from your GPS module
  // 2. Update the lap timer with GPS coordinates
  // 3. Display sector times and optimal lap

  // Example GPS update (you would replace this with real GPS data):
  // double currentLat = gps.latitude;
  // double currentLng = gps.longitude;
  // float currentAlt = gps.altitude;
  // float currentSpeed = gps.speed;
  // unsigned long gpsTime = getGpsTimeInMilliseconds();

  // lapTimer.updateCurrentTime(gpsTime);
  // lapTimer.loop(currentLat, currentLng, currentAlt, currentSpeed);

  // Display sector timing information
  displaySectorInfo();

  delay(1000);
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
