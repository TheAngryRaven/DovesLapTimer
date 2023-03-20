/**
 * Originally intended to use for gokarting this library offers a simple way to get basic lap timing information from a GPS based system.
 * This library does NOT interface with your GPS, simply feed it data and check the state.
 * Right now this only offers a single split time around the "start/finish" and would not work for many other purposes without modification.
 * 
 * The development of this library has been overseen, and all documentation has been generated using chatGPT4.
 */

#ifndef _DOVES_LAP_TIMER_H
#define _DOVES_LAP_TIMER_H

#include <cfloat>
#include <math.h>
#include <cstring>
#include "Arduino.h"

class DovesLapTimer {
public:
  DovesLapTimer();

  /**
   * @brief Calculates the pace difference between the current lap and the best lap.
   *
   * This function computes the pace for both the current lap and the best lap, and returns the difference.
   * A positive value indicates that the current lap's pace is slower than the best lap's pace, while a negative
   * value indicates that the current lap's pace is faster.
   */
  double paceDifference();
  /**
   * @brief Checks if the kart is crossing the start/finish line and calculates lap time and crossing point.
   *
   * This function is responsible for detecting when the kart is crossing the start/finish line. It compares
   * the current position to the start/finish line and, if it is within a specified threshold distance,
   * starts saving GPS data to a buffer. When the kart moves away from the line, the function calls
   * interpolateCrossingPoint() to calculate the precise point at which the kart crossed the line,
   * and computes the lap time.
   *
   * @param currentLat Latitude of the current position in decimal degrees.
   * @param currentLng Longitude of the current position in decimal degrees.
   */
  void checkStartFinish(double currentLat, double currentLng);
  /**
   * @brief Calculates the angle between two 2D vectors in degrees.
   *
   * This function takes the components of two 2D vectors and calculates the angle between them in degrees.
   * It uses the dot product formula and the arccosine function to compute the angle.
   *
   * @param vector1X The x-component of the first vector
   * @param vector1Y The y-component of the first vector
   * @param vector2X The x-component of the second vector
   * @param vector2Y The y-component of the second vector
   * @return double The angle between the two vectors in degrees
   */
  double angleBetweenVectors(double vector1X, double vector1Y, double vector2X, double vector2Y);
  /**
   * @brief Determines if the triangle formed by three GPS coordinates is an acute triangle.
   * @param lat1 Latitude of the first coordinate.
   * @param lon1 Longitude of the first coordinate.
   * @param lat2 Latitude of the second coordinate.
   * @param lon2 Longitude of the second coordinate.
   * @param lat3 Latitude of the third coordinate.
   * @param lon3 Longitude of the third coordinate.
   * @return true if the triangle is acute, false otherwise.
   */
  bool isAcuteTriangle(double lat1, double lon1, double lat2, double lon2, double lat3, double lon3);
  /**
   * @brief Determines if the driver is near the given line or not.
   *
   * This function takes the latitude and longitude of the driver's current position and calculates
   * if the driver is near the given line. It forms a triangle with the line as the base and the
   * driver's position as the third point. The function then checks if both lines formed are less than the base of the line-segment.
   *
   * @param driverLat Latitude of the driver's current position in decimal degrees
   * @param driverLng Longitude of the driver's current position in decimal degrees
   * @param pointALat Latitude of the first point of the line in decimal degrees
   * @param pointALng Longitude of the first point of the line in decimal degrees
   * @param pointBLat Latitude of the second point of the line in decimal degrees
   * @param pointBLng Longitude of the second point of the line in decimal degrees
   * @return bool True if the driver is near the line, false otherwise
   */
  bool isDriverNearLine(double driverLat, double driverLng, double pointALat, double pointALng, double pointBLat, double pointBLng);
  /**
   * @brief Determines which side of a line a driver is on.
   *
   * Given a driver's position and two points defining a line segment, this function computes
   * the side of the line the driver is on. The line is treated as infinite for the side determination.
   * 
   * @param driverLat The latitude of the driver's position.
   * @param driverLng The longitude of the driver's position.
   * @param pointALat The latitude of the first point of the line.
   * @param pointALng The longitude of the first point of the line.
   * @param pointBLat The latitude of the second point of the line.
   * @param pointBLng The longitude of the second point of the line.
   * @return Returns 1 if the driver is on one side of the line, -1 if the driver is on the other side, and 0 if the driver is exactly on the line.
   */
  int driverSideOfLine(double driverLat, double driverLng, double pointALat, double pointALng, double pointBLat, double pointBLng);
  /**
   * @brief Calculate the shortest distance between a point and a line segment.
   *
   * This function takes the coordinates of a point (pointX, pointY) and a line segment
   * defined by two endpoints (startX, startY) and (endX, endY), and returns the shortest
   * distance between the point and the line segment. The distance is calculated
   * in the same unit as the input coordinates (e.g., degrees for latitude and
   * longitude values).
   *
   * @param pointX The x-coordinate of the point.
   * @param pointY The y-coordinate of the point.
   * @param startX The x-coordinate of the first endpoint of the line segment.
   * @param startY The y-coordinate of the first endpoint of the line segment.
   * @param endX The x-coordinate of the second endpoint of the line segment.
   * @param endY The y-coordinate of the second endpoint of the line segment.
   * @return The shortest distance between the point and the line segment.
   */
  double pointLineSegmentDistance(double pointX, double pointY, double startX, double startY, double endX, double endY);
  /**
   * @brief Calculates the great-circle distance between two points on the Earth's surface using the Haversine formula.
   *
   * This function takes the latitude and longitude of two points in decimal degrees and returns the distance between
   * them in meters. The Haversine formula is used to account for the Earth's curvature, providing accurate results
   * for relatively short distances (up to a few thousand kilometers).
   *
   * Note: This function assumes that the Earth is a perfect sphere with a radius of 6,371 kilometers.
   *
   * @param lat1 Latitude of the first point in decimal degrees
   * @param lon1 Longitude of the first point in decimal degrees
   * @param lat2 Latitude of the second point in decimal degrees
   * @param lon2 Longitude of the second point in decimal degrees
   * @return double The great-circle distance between the two points in meters
   */
  double haversine(double lat1, double lon1, double lat2, double lon2);
  /**
   * @brief Calculates the distance between two GPS points, including altitude difference.
   *
   * This function computes the distance between two GPS points using the haversine formula,
   * and takes into account the altitude difference between the points. The resulting distance
   * is the true 3D distance between the points, rather than just the 2D distance on the Earth's surface.
   *
   * @param prevLat Latitude of the first GPS point in decimal degrees.
   * @param prevLng Longitude of the first GPS point in decimal degrees.
   * @param prevAlt Altitude of the first GPS point in meters.
   * @param currentLat Latitude of the second GPS point in decimal degrees.
   * @param curentLng Longitude of the second GPS point in decimal degrees.
   * @param currentAlt Altitude of the second GPS point in meters.
   * @return The 3D distance between the two GPS points in meters.
   */
  double haversineAltitude(double prevLat, double prevLng, double prevAlt, double currentLat, double curentLng, double currentAlt);

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  /**
   * @brief Reset all parameters back to 0
   */
  void reset();
  /**
   * @brief Sets the start/finish line using two points (A and B).
   *
   * @param pointALat Latitude of point A in decimal degrees.
   * @param pointALng Longitude of point A in decimal degrees.
   * @param pointBLat Latitude of point B in decimal degrees.
   * @param pointBLng Longitude of point B in decimal degrees.
   */
  void setStartFinishLine(double pointALat, double pointALng, double pointBLat, double pointBLng);
  /**
   * @brief Updates the total distance traveled by calculating the distance between the current position and the previous position using the haversineAltitude() function.
   *
   * This function takes the current latitude, longitude, and altitude as arguments, and computes the distance between the current position and the previous position,
   * which is stored in the class member variables positionPrevLat, positionPrevLng, and positionPrevAlt.
   * It then updates the totalDistanceTraveled member variable by adding the computed distance.
   *
   * @param currentLat Latitude of the current position in decimal degrees.
   * @param currentLng Longitude of the current position in decimal degrees.
   * @param currentAltitudeMeters Altitude of the current position in meters.
   */
  void updateOdometer(double currentLat, double currentLng, double currentAltitudeMeters);
  /**
   * @brief Updates the current GPS time since midnight.
   *
   * @param currentTimeMilliseconds The current time in milliseconds.
   */
  void updateCurrentTime(unsigned long currentTimeMilliseconds);
  /**
   * @brief Updates current speed in kilometers per hour
   *
   * @param currentSpeed The current speed in kilometers per hour
   */
  void updateCurrentSpeedKmh(float currentSpeedkmh);
  /**
   * @brief Gets the race started status (passed the line one time).
   *
   * @return True if the race has started, false otherwise.
   */
  bool getRaceStarted() const;
  /**
   * @brief Gets the crossing status.
   *
   * @return True if crossing the start/finish line, false otherwise.
   */
  bool getCrossing() const;
  /**
   * @brief Gets the current lap start time.
   *
   * @return The current lap start time in milliseconds.
   */
  unsigned long getCurrentLapStartTime() const;
  /**
   * @brief Gets the current lap time.
   *
   * @return The current lap time in milliseconds.
   */
  unsigned long getCurrentLapTime() const;
  /**
   * @brief Gets the last lap time.
   *
   * @return The last lap time in milliseconds.
   */
  unsigned long getLastLapTime() const;
  /**
   * @brief Gets the best lap time.
   *
   * @return The best lap time in milliseconds.
   */
  unsigned long getBestLapTime() const;
  /**
   * @brief Gets the current lap odometer start.
   *
   * @return The distance traveled at the start of the current lap in meters.
   */
  float getCurrentLapOdometerStart() const;
  /**
   * @brief Gets the current lap time.
   *
   * @return The current lap time in milliseconds.
   */
  float getCurrentLapDistance() const;
  /**
   * @brief Gets the last lap distance.
   *
   * @return The distance traveled during the last lap in meters.
   */
  float getLastLapDistance() const;
  /**
   * @brief Gets the best lap distance.
   *
   * @return The distance traveled during the best lap in meters.
   */
  float getBestLapDistance() const;
  /**
   * @brief Gets the total distance traveled.
   *
   * @return The total distance traveled in meters.
   */
  float getTotalDistanceTraveled() const;
  /**
   * @brief Gets the best lap number.
   *
   * @return The lap number of the best lap.
   */
  int getBestLapNumber() const;
  /**
   * @brief Gets the total number of laps completed.
   *
   * @return The total number of laps completed.
   */
  int getLaps() const;

private:
  /**
   * @brief Computes the interpolation weight based on distances and speeds.
   * 
   * @param distA Distance from point A to the line.
   * @param distB Distance from point B to the line.
   * @param speedA Speed (in km/h) at point A.
   * @param speedB Speed (in km/h) at point B.
   * @return Interpolation weight factor for point A.
   */
  double interpolateWeight(double distA, double distB, float speedA, float speedB);

  #ifdef DOVES_LAP_TIMER_FORCE_LINEAR
  /**
   * @brief Calculates the crossing point's latitude, longitude, and time based on the buffer points and the line defined by two points.
   *
   * This function iterates through the buffer of GPS points and finds the best pair of consecutive points
   * with the smallest sum of distances to the line defined by two points (pointALat, pointALng) and (pointBLat, pointBLng).
   * It then interpolates the crossing point's latitude, longitude, and time using these best pair of points.
   *
   * @param crossingLat Reference to the variable that will store the crossing point's latitude.
   * @param crossingLng Reference to the variable that will store the crossing point's longitude.
   * @param crossingTime Reference to the variable that will store the crossing point's time.
   * @param crossingOdometer Reference to the variable that will store the crossing point's odometer.
   * @param pointALat Latitude of the first point of the line in decimal degrees.
   * @param pointALng Longitude of the first point of the line in decimal degrees.
   * @param pointBLat Latitude of the second point of the line in decimal degrees.
   * @param pointBLng Longitude of the second point of the line in decimal degrees.
   */
  void interpolateCrossingPoint(double& crossingLat, double& crossingLng, unsigned long& crossingTime, double& crossingOdometer, double pointALat, double pointALng, double pointBLat, double pointBLng);
  #else
  /**
   * @brief Catmull-Rom spline interpolation between two points
   *
   * @param p0 Value at point 0
   * @param p1 Value at point 1
   * @param p2 Value at point 2
   * @param p3 Value at point 3
   * @param t Interpolation parameter [0, 1]
   * @return Interpolated value
   */
  double catmullRom(double p0, double p1, double p2, double p3, double t);
  /**
   * @brief Calculates the crossing point's latitude, longitude, and time using Catmull-Rom spline interpolation.
   *
   * This function iterates through the buffer of GPS points and finds the best pair of consecutive points
   * with the smallest sum of distances to the line defined by two points (pointALat, pointALng) and (pointBLat, pointBLng).
   * It then uses Catmull-Rom spline interpolation to calculate the crossing point's latitude, longitude, and time.
   *
   * @param crossingLat Reference to the variable that will store the crossing point's latitude.
   * @param crossingLng Reference to the variable that will store the crossing point's longitude.
   * @param crossingTime Reference to the variable that will store the crossing point's time.
   * @param crossingOdometer Reference to the variable that will store the crossing point's odometer.
   * @param pointALat Latitude of the first point of the line in decimal degrees.
   * @param pointALng Longitude of the first point of the line in decimal degrees.
   * @param pointBLat Latitude of the second point of the line in decimal degrees.
   * @param pointBLng Longitude of the second point of the line in decimal degrees.
   */
  void interpolateCrossingPointOnCurve(double& crossingLat, double& crossingLng, unsigned long& crossingTime, double& crossingOdometer, double pointALat, double pointALng, double pointBLat, double pointBLng);
  #endif

  unsigned long millisecondsSinceMidnight = -1;

  // Timing variables
  bool raceStarted = false;
  bool crossing = false;
  unsigned long currentLapStartTime = 0;
  unsigned long lastLapTime = 0;
  unsigned long bestLapTime = 0;
  float currentLapOdometerStart = 0.0;
  float lastLapDistance = 0.0;
  float bestLapDistance = 0.0;
  float currentSpeedkmh = 0.0;
  int bestLapNumber = 0;
  int laps = 0;

  float totalDistanceTraveled = 0;
  float posistionPrevAlt = 0;
  double posistionPrevLat = 0;
  double posistionPrevLng = 0;

  // Distance in meters to start/finish line to start buffering
  double crossingThreshold = 10;
  // Number of GPS coordinates to store in the buffer for interpolation
  // right now a little excessive for testing
  static const int crossingPointBufferSize = 300;// = 300;

  struct crossingPointBufferEntry {
    double lat; // latitude
    double lng; // longitude
    unsigned long time; // current time in milliseconds
    float odometer; // time traveled since device start and this entry
    float speedKmh; // speed in kmph
  };

  double startFinishPointALat;
  double startFinishPointALng;
  double startFinishPointBLat;
  double startFinishPointBLng;

  const double radiusEarth = 6371.0 * 1000; // Earth's radius in meters

  crossingPointBufferEntry crossingPointBuffer[crossingPointBufferSize];
  int crossingPointBufferIndex = 0;
  bool crossingPointBufferFull = false;
};

#endif