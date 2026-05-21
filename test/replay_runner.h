/**
 * Layer-3 NMEA replay helper.
 *
 * Parses raw NMEA strings ($GPGGA + $GPRMC) into the (lat, lng, alt,
 * speed, time) tuples that DovesLapTimer.loop() expects, and drives a
 * replay over the full fixture. Returns the list of completed lap times
 * so each test_nmea_*.cpp can assert against the golden values documented
 * in the corresponding gps_race_data_*.h header.
 *
 * No Adafruit_GPS dependency — the parser only handles the two sentence
 * types our fixtures contain, which is enough.
 */

#ifndef REPLAY_RUNNER_H
#define REPLAY_RUNNER_H

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include "../src/DovesLapTimer.h"

struct NmeaState {
  double        lat = 0.0;
  double        lng = 0.0;
  float         alt_m = 0.0f;       // last known from any GGA
  float         speed_knots = 0.0f;
  unsigned long time_ms = 0;
  bool          fix = false;
};

// NMEA coord format: DDMM.MMMM or DDDMM.MMMM (degrees + decimal minutes).
// Two digits before the decimal are always the minutes.
static inline double parseNmeaCoord(const char* s, char hemi) {
  if (!s || !*s) return 0.0;
  const char* dot = strchr(s, '.');
  if (!dot) return 0.0;
  const int minIntDigits = 2;
  const int degDigits = (int)(dot - s) - minIntDigits;
  if (degDigits < 0) return 0.0;

  char degBuf[8] = {0};
  strncpy(degBuf, s, degDigits < 7 ? degDigits : 7);
  double degrees = atof(degBuf);
  double minutes = atof(s + degDigits);
  double decimal = degrees + minutes / 60.0;
  if (hemi == 'S' || hemi == 'W') decimal = -decimal;
  return decimal;
}

// HHMMSS.sss -> milliseconds since midnight.
static inline unsigned long parseNmeaTime(const char* s) {
  if (!s || strlen(s) < 6) return 0;
  unsigned long h = (s[0] - '0') * 10 + (s[1] - '0');
  unsigned long m = (s[2] - '0') * 10 + (s[3] - '0');
  double secs = atof(s + 4);
  return h * 3600000UL + m * 60000UL + (unsigned long)(secs * 1000.0);
}

// Tokenize a NMEA sentence (one line, no trailing newline) into up to N
// comma-separated fields. Leading '$' is skipped. Returns the count.
static inline int tokenize(const char* line, char* buf, size_t bufSize,
                           char** tokens, int maxTokens) {
  if (!line) return 0;
  const char* src = (line[0] == '$') ? line + 1 : line;
  strncpy(buf, src, bufSize - 1);
  buf[bufSize - 1] = '\0';
  // Strip the *XX checksum trailer
  char* star = strchr(buf, '*');
  if (star) *star = '\0';

  int n = 0;
  char* save = nullptr;
  for (char* tok = strtok_r(buf, ",", &save);
       tok && n < maxTokens;
       tok = strtok_r(nullptr, ",", &save)) {
    tokens[n++] = tok;
  }
  return n;
}

// Parse one NMEA line into state. Returns true if the line was a sentence
// we care about (GGA or RMC), even if no fix.
static inline bool parseNmeaLine(const char* line, NmeaState& state) {
  if (!line) return false;
  char buf[160];
  char* tokens[16] = {0};

  if (strncmp(line, "$GPGGA,", 7) == 0) {
    // GPGGA,time,lat,N/S,lng,E/W,fix,sats,hdop,alt,M,...
    int n = tokenize(line, buf, sizeof(buf), tokens, 16);
    if (n < 10) return true;
    if (tokens[6][0] == '0') {
      state.fix = false;
      return true;
    }
    state.fix = true;
    state.time_ms = parseNmeaTime(tokens[1]);
    state.lat = parseNmeaCoord(tokens[2], tokens[3][0]);
    state.lng = parseNmeaCoord(tokens[4], tokens[5][0]);
    state.alt_m = (float)atof(tokens[9]);
    return true;
  }

  if (strncmp(line, "$GPRMC,", 7) == 0) {
    // GPRMC,time,status,lat,N/S,lng,E/W,speed_knots,course,date,...
    int n = tokenize(line, buf, sizeof(buf), tokens, 16);
    if (n < 8) return true;
    if (tokens[2][0] != 'A') {
      state.fix = false;
      return true;
    }
    state.fix = true;
    state.time_ms = parseNmeaTime(tokens[1]);
    state.lat = parseNmeaCoord(tokens[3], tokens[4][0]);
    state.lng = parseNmeaCoord(tokens[5], tokens[6][0]);
    state.speed_knots = (float)atof(tokens[7]);
    return true;
  }

  return false;
}

struct ReplayConfig {
  double sfA_lat, sfA_lng;
  double sfB_lat, sfB_lng;
  double crossingThresholdMeters = 7.0;
  bool   useCatmullRom = false;
};

struct ReplayResult {
  std::vector<unsigned long> lapTimes;
  bool   raceStarted = false;
  int    totalLaps = 0;
};

// Run a full replay through the lap timer. Returns recorded lap times.
static inline ReplayResult runNmeaReplay(const char* const* gpsLogs,
                                         int numLogs,
                                         const ReplayConfig& cfg) {
  DovesLapTimer timer(cfg.crossingThresholdMeters);
  timer.setStartFinishLine(cfg.sfA_lat, cfg.sfA_lng, cfg.sfB_lat, cfg.sfB_lng);
  if (cfg.useCatmullRom) timer.forceCatmullRomInterpolation();
  else                   timer.forceLinearInterpolation();

  NmeaState state;
  ReplayResult result;
  int lastLap = 0;

  for (int i = 0; i < numLogs; i++) {
    if (!parseNmeaLine(gpsLogs[i], state)) continue;
    if (!state.fix) continue;

    timer.updateCurrentTime(state.time_ms);
    timer.loop(state.lat, state.lng, state.alt_m, state.speed_knots);

    int curLap = timer.getLaps();
    if (curLap > lastLap) {
      result.lapTimes.push_back(timer.getLastLapTime());
      lastLap = curLap;
    }
  }

  result.raceStarted = timer.getRaceStarted();
  result.totalLaps   = timer.getLaps();
  return result;
}

#endif
