# Layer-2 host-native unit tests for DovesLapTimer.
#
# Each test_*.cpp builds into build/<name> and is run in sequence. Any
# non-zero exit code fails the whole run.
#
# Usage:
#   make            # build all tests
#   make run        # build + run all tests
#   make clean      # remove build artifacts
#
# Mock headers in mock/ are found before any (host-absent) Arduino headers
# via the -Imock include path, so the library's #include <Arduino.h> /
# #include "ArxTypeTraits.h" resolve to host-friendly stubs.

CXX      ?= g++
# -include mock/Arduino.h injects Arduino.h at the top of every TU, mirroring
# what the Arduino IDE does automatically for .ino sketches and library .cpp
# files. Without this, library sources that use F() / Stream / memset without
# an explicit include fail to compile on the host.
CXXFLAGS  = -std=c++14 -Wall -Wextra -O0 -g -Imock -I../src -include mock/Arduino.h
LDFLAGS   =

# Library sources we link into every test (small enough that always-link
# costs nothing and we don't have to per-test bookkeep dependencies).
LIB_SRCS = ../src/DovesLapTimer.cpp \
           ../src/WaypointLapTimer.cpp \
           ../src/CourseDetector.cpp \
           ../src/CourseManager.cpp

TEST_SRCS = $(wildcard test_*.cpp)
TEST_BINS = $(TEST_SRCS:%.cpp=build/%)

# Every header any test could pull in. Listing them as prerequisites is
# deliberately coarse (any header edit rebuilds everything — the suite
# builds in seconds) but it closes the false-green hole where `make run`
# passed on binaries that were stale against ../src/*.h, replay_runner.h,
# or the NMEA fixture headers.
HEADERS = $(wildcard ../src/*.h) \
          $(wildcard ../examples/real_track_data_debug/*.h) \
          $(wildcard mock/*.h) \
          test_runner.h replay_runner.h

all: $(TEST_BINS)

build/%: %.cpp $(LIB_SRCS) $(HEADERS)
	@mkdir -p build
	$(CXX) $(CXXFLAGS) -o $@ $< $(LIB_SRCS) $(LDFLAGS)

run: all
	@failed=0; \
	for bin in $(TEST_BINS); do \
		echo ""; \
		echo "Running $$bin"; \
		echo "================================"; \
		if ! $$bin; then failed=$$((failed + 1)); fi; \
	done; \
	echo ""; \
	if [ $$failed -gt 0 ]; then \
		echo "FAILED: $$failed test binary/binaries reported failures."; \
		exit 1; \
	else \
		echo "All test binaries passed."; \
	fi

clean:
	rm -rf build/ coverage/ *.gcno *.gcda *.gcov coverage-badge.json coverage-summary.md

# ----- Coverage -----
# Instruments the library sources, runs every test binary (gcda accumulates
# across them), and summarizes line coverage of ../src/ with gcovr. Fails if
# below COVERAGE_GATE percent. The gate sits just below measured coverage
# (84.5% line as of 2026-06-11) so any change that meaningfully drops
# coverage fails CI — keep ratcheting it up alongside new tests.
# Override on the command line: `make coverage COVERAGE_GATE=40`.
COVERAGE_GATE ?= 80
COV_DIR        = coverage
COV_BINS       = $(TEST_SRCS:%.cpp=$(COV_DIR)/%)

$(COV_DIR)/%: %.cpp $(LIB_SRCS) $(HEADERS)
	@mkdir -p $(COV_DIR)
	$(CXX) $(CXXFLAGS) --coverage -o $@ $< $(LIB_SRCS) $(LDFLAGS)

coverage: $(COV_BINS)
	@for bin in $(COV_BINS); do \
		echo "Running $$bin"; \
		$$bin > /dev/null || exit 1; \
	done
	@echo ""
	gcovr --root .. --filter '\.\./src/' \
	      --fail-under-line $(COVERAGE_GATE) \
	      --print-summary

# Emit a shields.io endpoint JSON describing line coverage. Used by CI to
# publish the README badge. Writes coverage-badge.json in the test dir.
coverage-badge: $(COV_BINS)
	@for bin in $(COV_BINS); do $$bin > /dev/null || exit 1; done
	@gcovr --root .. --filter '\.\./src/' --json-summary-pretty -o /tmp/cov.json >/dev/null
	@python3 -c "import json; \
d=json.load(open('/tmp/cov.json')); \
p=round(d['line_percent']); \
c='brightgreen' if p>=90 else 'green' if p>=75 else 'yellowgreen' if p>=60 else 'yellow' if p>=40 else 'orange' if p>=20 else 'red'; \
json.dump({'schemaVersion':1,'label':'coverage','message':f'{p}%','color':c}, open('coverage-badge.json','w'))"
	@echo "wrote coverage-badge.json:" && cat coverage-badge.json && echo ""

# Emit a Markdown coverage report (overall + per-file) for posting as a
# per-PR comment. Writes coverage-summary.md in the test dir.
coverage-summary: $(COV_BINS)
	@for bin in $(COV_BINS); do $$bin > /dev/null || exit 1; done
	@gcovr --root .. --filter '\.\./src/' --markdown coverage-summary.md >/dev/null
	@echo "wrote coverage-summary.md"

.PHONY: all run clean coverage coverage-badge coverage-summary
