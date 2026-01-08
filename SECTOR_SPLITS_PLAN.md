# Game Plan: Adding Sector Split Timing to DovesLapTimer

## Current State Analysis

### Existing Architecture
- **Line Detection**: Single start/finish line defined by two lat/long pairs
- **Detection Logic**: Sophisticated threshold-based detection using:
  - `insideLineThreshold()` - Triangle method to create detection zone
  - `pointLineSegmentDistance()` - Perpendicular distance calculation
  - Hysteresis (7m to enter, 8m to exit)
  - GPS point buffering during crossing
  - Interpolation to find exact crossing moment
- **Data Storage**: Simple variables for current/last/best lap times
- **State Machine**: `crossing` flag, `raceStarted` flag

### Key Files
- `src/DovesLapTimer.h` - Class definition (lines 330-369 have timing state)
- `src/DovesLapTimer.cpp` - Implementation (lines 63-217 have crossing logic)

## Requirements

### Sector Line Configuration
- Add **2 additional timing lines** (Sector 2 and Sector 3)
- Each line configured like start/finish: two lat/long pairs
- Three sectors total:
  - **Sector 1**: Start/Finish → Sector 2 line
  - **Sector 2**: Sector 2 line → Sector 3 line
  - **Sector 3**: Sector 3 line → Start/Finish

### Data Storage
- Store **best time for each sector** (3 values)
- Do NOT store all sector times (memory constrained)
- Calculate **optimal lap** = sum of best sector times

### API Functions
- `getBestSector1Time()` / `getBestSector2Time()` / `getBestSector3Time()`
- `getOptimalLapTime()` - calculated from best sectors
- Sector configuration setters (similar to `setStartFinishLine()`)

## Implementation Plan

### Phase 1: Data Structure Extensions

#### 1.1 Add Sector Line Configuration Variables (DovesLapTimer.h)
```cpp
// Sector 2 line coordinates
double sector2PointALat;
double sector2PointALng;
double sector2PointBLat;
double sector2PointBLng;

// Sector 3 line coordinates
double sector3PointALat;
double sector3PointALng;
double sector3PointBLat;
double sector3PointBLng;

// Flags to track if sector lines are configured
bool sector2LineConfigured = false;
bool sector3LineConfigured = false;
```

#### 1.2 Add Sector Timing State Variables (DovesLapTimer.h)
```cpp
// Current sector tracking
int currentSector = 0;  // 0=not started, 1/2/3=in sector
unsigned long currentSectorStartTime = 0;  // When current sector started

// Current lap sector times (reset each lap)
unsigned long currentLapSector1Time = 0;
unsigned long currentLapSector2Time = 0;
unsigned long currentLapSector3Time = 0;

// Best sector times (persistent)
unsigned long bestSector1Time = 0;
unsigned long bestSector2Time = 0;
unsigned long bestSector3Time = 0;

// Which lap achieved best sectors
int bestSector1LapNumber = 0;
int bestSector2LapNumber = 0;
int bestSector3LapNumber = 0;
```

#### 1.3 Add Sector Line Crossing State (DovesLapTimer.h)
```cpp
// Crossing state for each line
bool crossingSector2 = false;
bool crossingSector3 = false;
// (crossingStartFinish renamed from 'crossing')
```

### Phase 2: Line Detection Refactoring

#### 2.1 Create Generic Line Crossing Detection
**Challenge**: Current code is hardcoded for start/finish line

**Solution**: Extract crossing detection into a reusable structure

**Option A - Shared Buffer Approach** (RECOMMENDED):
- Use single `crossingPointBuffer` for all lines
- Simpler state management
- Lower memory usage
- Assumes driver can't cross multiple lines simultaneously (valid for racing)

**Option B - Separate Buffers**:
- Each line has own buffer and state
- Higher memory usage (~600 bytes × 3 = 1.8KB)
- More complex but handles edge cases

**Recommendation**: Option A (Shared Buffer) because:
1. Physical impossibility of crossing 2+ lines simultaneously at racing speeds
2. Memory efficiency critical on microcontrollers
3. Simpler state machine

#### 2.2 Refactor Detection Logic
Create new enum and helper functions:

```cpp
enum LineType {
  LINE_START_FINISH = 0,
  LINE_SECTOR2 = 1,
  LINE_SECTOR3 = 2
};

// Generalized distance check
double getDistanceToLine(LineType lineType, double driverLat, double driverLon);

// Check if inside threshold for any line
bool isInsideLineThreshold(LineType lineType, double driverLat, double driverLon);

// Get line coordinates by type
void getLineCoordinates(LineType lineType, double& pointALat, double& pointALng,
                        double& pointBLat, double& pointBLng);
```

#### 2.3 Multi-Line State Machine
Modify `update()` method (lines 63-217) to check all configured lines:

```cpp
void update(double lat, double lng, ...) {
  // ... existing GPS processing ...

  // Check each line in order
  checkLineCrossing(LINE_START_FINISH, lat, lng);
  if (sector2LineConfigured) {
    checkLineCrossing(LINE_SECTOR2, lat, lng);
  }
  if (sector3LineConfigured) {
    checkLineCrossing(LINE_SECTOR3, lat, lng);
  }
}
```

### Phase 3: Sector Timing Logic

#### 3.1 Handle Line Crossing Events
When a line is crossed, determine which sector transition occurred:

```cpp
void handleLineCrossing(LineType lineType, unsigned long crossingTime) {
  if (lineType == LINE_START_FINISH) {
    // Finishing sector 3, starting sector 1
    if (raceStarted && currentSector == 3) {
      currentLapSector3Time = crossingTime - currentSectorStartTime;
      checkAndUpdateBestSectors();
      completeLap(crossingTime);  // Existing lap completion logic
    }
    currentSector = 1;
    currentSectorStartTime = crossingTime;
    raceStarted = true;

  } else if (lineType == LINE_SECTOR2) {
    // Finishing sector 1, starting sector 2
    if (currentSector == 1) {
      currentLapSector1Time = crossingTime - currentSectorStartTime;
      currentSector = 2;
      currentSectorStartTime = crossingTime;
    } else {
      // Out of order - invalidate lap
      invalidateCurrentLap();
    }

  } else if (lineType == LINE_SECTOR3) {
    // Finishing sector 2, starting sector 3
    if (currentSector == 2) {
      currentLapSector2Time = crossingTime - currentSectorStartTime;
      currentSector = 3;
      currentSectorStartTime = crossingTime;
    } else {
      // Out of order - invalidate lap
      invalidateCurrentLap();
    }
  }
}
```

#### 3.2 Best Sector Updates
```cpp
void checkAndUpdateBestSectors() {
  if (bestSector1Time == 0 || currentLapSector1Time < bestSector1Time) {
    bestSector1Time = currentLapSector1Time;
    bestSector1LapNumber = laps + 1;
  }

  if (bestSector2Time == 0 || currentLapSector2Time < bestSector2Time) {
    bestSector2Time = currentLapSector2Time;
    bestSector2LapNumber = laps + 1;
  }

  if (bestSector3Time == 0 || currentLapSector3Time < bestSector3Time) {
    bestSector3Time = currentLapSector3Time;
    bestSector3LapNumber = laps + 1;
  }
}
```

#### 3.3 Invalid Lap Handling
```cpp
void invalidateCurrentLap() {
  // Reset sector times for current lap
  currentLapSector1Time = 0;
  currentLapSector2Time = 0;
  currentLapSector3Time = 0;
  currentSector = 0;  // Invalid state
}
```

### Phase 4: Public API

#### 4.1 Configuration Methods
```cpp
void setSector2Line(double pointALat, double pointALng,
                    double pointBLat, double pointBLng);
void setSector3Line(double pointALat, double pointALng,
                    double pointBLat, double pointBLng);

// Optional: Clear sector configuration
void clearSectorLines();
```

#### 4.2 Getter Methods
```cpp
// Best sector times
unsigned long getBestSector1Time() const;
unsigned long getBestSector2Time() const;
unsigned long getBestSector3Time() const;

// Current lap sector times (live timing)
unsigned long getCurrentLapSector1Time() const;
unsigned long getCurrentLapSector2Time() const;
unsigned long getCurrentLapSector3Time() const;

// Optimal lap time (sum of best sectors)
unsigned long getOptimalLapTime() const;

// Delta to optimal lap (current lap total vs optimal)
long getDeltaToOptimal() const;

// Which lap achieved best sectors
int getBestSector1LapNumber() const;
int getBestSector2LapNumber() const;
int getBestSector3LapNumber() const;

// Current sector driver is in
int getCurrentSector() const;

// Check if sector lines are configured
bool areSectorLinesConfigured() const;
```

### Phase 5: Edge Cases & Safety

#### 5.1 Backwards Compatibility
- Sector timing is **optional** - works without sector lines configured
- Existing API unchanged (no breaking changes)
- Sector getters return 0 if not configured

#### 5.2 GPS Signal Loss
- If GPS signal lost mid-sector, sector time invalid
- Check for time discontinuities (gap > threshold)
- Invalidate current lap if detected

#### 5.3 Out-of-Order Crossings
- Detect using `currentSector` state
- Invalidate lap if wrong line crossed
- Reset to valid state on next start/finish crossing

#### 5.4 Memory Constraints
- Minimal memory overhead (~100 bytes for sector state)
- Reuse existing buffer and interpolation code
- No sector history stored

#### 5.5 Race Start Behavior
- Sector timing only valid after first start/finish crossing
- First lap: sector 1 and sector 2 valid, sector 3 completes lap
- Handle "formation lap" scenarios

## Implementation Order

### Step 1: Core Refactoring (Non-Breaking)
1. Extract line coordinate storage into generic helpers
2. Create `getLineCoordinates()` helper
3. Add distance calculation helpers for any line type
4. **Test**: Ensure existing lap timing still works

### Step 2: Add Sector Configuration
1. Add sector line variables to header
2. Implement `setSector2Line()` and `setSector3Line()`
3. Add basic getters that return 0
4. **Test**: Configuration methods work, no crashes

### Step 3: Multi-Line Detection
1. Refactor `update()` to check all configured lines
2. Implement shared buffer approach with line type tracking
3. Add crossing state flags for each line
4. **Test**: Can detect crossings on all 3 lines independently

### Step 4: Sector Timing Logic
1. Implement `handleLineCrossing()` with sector state machine
2. Add sector time calculation
3. Implement best sector updates
4. **Test**: Sector times calculated correctly

### Step 5: API Completion
1. Implement all getter methods
2. Add optimal lap calculation
3. Add delta calculations
4. **Test**: Full integration test with 3 sector lines

### Step 6: Edge Case Hardening
1. Add out-of-order detection
2. Add GPS signal loss detection
3. Test backwards compatibility (no sectors configured)
4. **Test**: Stress test with real GPS data

### Step 7: Documentation & Examples
1. Update README.md with sector timing documentation
2. Create example sketch with sector configuration
3. Document optimal lap calculation methodology

## Testing Strategy

### Unit Testing (using real_track_data_debug example)
1. Load recorded GPS data from Orlando Kart Center
2. Configure sector 2 and sector 3 lines
3. Verify sector times sum to total lap time
4. Verify best sector detection
5. Verify optimal lap calculation

### Integration Testing
1. Test with sector lines disabled (backwards compat)
2. Test with only sector 2 configured (partial config)
3. Test with all sectors configured
4. Test out-of-order crossings (drive backwards)
5. Test GPS signal loss scenarios

### Memory Testing
1. Verify no memory leaks during sector updates
2. Check total RAM usage with sector feature enabled
3. Test on minimum hardware (non-high RAM boards)

## Open Questions

### Q1: Should sector times be included in lap completion callback?
**Consideration**: Current library doesn't have callbacks, but might be useful

**Recommendation**: Add later if needed, start with polling API

### Q2: Should we validate sector line positions?
**Consideration**: Sector lines could be configured incorrectly (overlapping, wrong order)

**Recommendation**: Add basic validation:
- Warn if sector lines too close to start/finish
- Warn if lines intersect
- Add `validateSectorConfiguration()` method

### Q3: Should we track "purple sectors" (best in current session)?
**Consideration**: Common in motorsports to highlight best ongoing performance

**Recommendation**: Yes, current design supports this - best sectors update immediately

### Q4: What about rolling start vs standing start?
**Consideration**: First lap might not have valid sector 1/2 times

**Recommendation**: Document that first lap after start/finish crossing is lap 1, sectors valid from that point

## Memory Impact Analysis

### New Variables (~100 bytes)
- Sector line coordinates: 8 doubles × 8 bytes = 64 bytes
- Sector timing state: 10 unsigned longs × 4 bytes = 40 bytes
- Sector flags and state: ~10 bytes
- **Total**: ~114 bytes

### Code Size (~2-3 KB)
- Multi-line detection logic: ~1 KB
- Sector timing logic: ~1 KB
- API methods: ~500 bytes
- **Total**: ~2.5 KB flash

### Buffer Impact
- No change (shared buffer approach)

**Conclusion**: Minimal impact, well within constraints of target hardware (256KB RAM)

## Performance Impact

### CPU Overhead
- 3× distance calculations per GPS update instead of 1×
- Negligible on 64MHz+ ARM processors
- ~0.3ms worst case at 25Hz GPS update rate

### GPS Update Rate Support
- No impact to 18-25Hz support
- State machine remains simple (shared buffer)

## Success Criteria

✅ Existing lap timing functionality unchanged
✅ Can configure 0, 1, or 2 sector lines independently
✅ Sector times calculated accurately (±100ms vs real timing)
✅ Best sector times tracked correctly
✅ Optimal lap calculation accurate
✅ No memory leaks or crashes
✅ Works on target hardware (Seeed NRF52840)
✅ Code well documented and maintainable

## Next Steps

**Review this plan** and confirm approach, then proceed with Step 1 implementation.

**Key Decision Points for Discussion:**
1. ✅ Shared buffer vs separate buffers? → Shared (recommended)
2. Should sector lines be required or optional? → Optional (recommended)
3. Should we validate sector configuration? → Basic validation (recommended)
4. Testing approach - use existing real track data? → Yes

