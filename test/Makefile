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

all: $(TEST_BINS)

build/%: %.cpp $(LIB_SRCS) test_runner.h mock/Arduino.h mock/ArxTypeTraits.h
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
	rm -rf build/

.PHONY: all run clean
