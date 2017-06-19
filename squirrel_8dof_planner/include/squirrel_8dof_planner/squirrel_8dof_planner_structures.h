#ifndef SQUIRREL_8DOF_PLANNER_STRUCTURES_H_
#define SQUIRREL_8DOF_PLANNER_STRUCTURES_H_

#include <ros/ros.h>

#include <string>
#include <utility>
#include <vector>
#include <cmath>
#include <stdint.h>

namespace SquirrelMotionPlanner
{

#define TINY_FLT 0.000001

typedef int32_t Int;
typedef uint32_t UInt;
typedef double Real;

/**
 * @brief A 2D cell structure with an x and y coordinate.
 * Used for representing positive integer of a 2D cell, e.g., indices of a 2D array.
 */
struct Cell2D
{
  UInt x;     ///< x coordinate of the 2D cell.
  UInt y;     ///< y coordinate of the 2D cell.

  /**
   * @brief Constructor, leaves x and y uninitialized.
   */
  Cell2D();

  /**
   * @brief Constructor, initializes x and y with the provided values.
   */
  Cell2D(const UInt &x, const UInt &y);

  /**
   * @brief Equal to operator, compares x and y with the x and y values of cell.
   * @param cell The cell to which the current x and y coordinates are to be compared.
   * @return Returns false if either x or y differ from the current values.
   */
  bool operator==(const Cell2D &cell) const;
};

/**
 * @brief A 2D tuple structure with an x and y value.
 * Used for representing a 2D floating point tuple, e.g., a metric 2D coordinate.
 */
struct Tuple2D
{
  Real x;     ///< x value of the 2D tuple.
  Real y;     ///< y value of the 2D tuple.

  /**
   * @brief Constructor, leaves x and y uninitialized.
   */
  Tuple2D();

  /**
   * @brief Constructor, initializes x and y with the provided values.
   */
  Tuple2D(Real x, Real y);

  /**
   * @brief Addition operator, that returns point with the added x and y values.
   * @param point Tuple2D to be added to the current x and y values.
   * @return Returns a Tuple2D with the added x and y values.
   */
  Tuple2D operator+(const Tuple2D &point) const;

  /**
   * @brief Addition operator, that adds a point to the current x and y values.
   * @param point Tuple2D to be added to the current x and y values.
   */
  void operator+=(const Tuple2D &point);

  /**
   * @brief Subtraction operator, that returns point with the subtracted x and y values.
   * @param point Tuple2D to be subtracted from the current x and y values.
   * @return Returns a Tuple2D with the subtracted x and y values.
   */
  Tuple2D operator-(const Tuple2D &point) const;

  /**
   * @brief Multiplication operator, that returns point with the multiplied x and y values.
   * @param factor A floating point value to be multiplied to the current x and y values.
   * @return Returns a Tuple2D with the multiplied x and y values.
   */
  Tuple2D operator*(const Real &factor) const;

  /**
   * @brief Multiplication operator, that multiplies the current x and y values by a given value.
   * @param factor A floating point value to be multiplied to the current x and y values.
   */
  void operator*=(const Real &factor);

  /**
   * @brief Division operator, that devides the current x and y values by a given value.
   * @param divisor A floating point value, that devides the current x and y values by a given value.
   */
  void operator/=(const Real &divisor);

  /**
   * @brief Division operator, that returns a Tuple2D with divided x and y values.
   * @param divisor A floating point value, that devides the current x and y values by a given value.
   * @return Returns a Tuple2D with the divided x and y values.
   */
  Tuple2D operator/(const Real &divisor) const;

  /**
   * @brief Finds the Euclidean distance to a given point.
   * @param point A Tuple2D to which the distance is found.
   * @return Returns a floating point value of the Euclidean distance to the given point.
   */
  Real distance(const Tuple2D &point) const;
};

/**
 * @brief A 2D line segment with a start and end point and a normalized direction.
 */
struct LineSegment2D
{
  Tuple2D start;     ///< Start point of the line segment.
  Tuple2D end;     ///< End point of the line segment.
  Tuple2D direction;     ///< Normalized direction of the line segment, pointing from start to end.

  Real length;     ///< Euclidean distance between start and end.
  Real angle;     ///< The angle to the x-axis, given in radians from -pi to pi.

  /**
   * @brief Constructor that leaves all members un-initialized.
   */
  LineSegment2D();

  /**
   * @brief Constructor that initializes all members.
   * @param start The start point of the line segment.
   * @param end The end point of the line segment.
   */
  LineSegment2D(const Tuple2D &start, const Tuple2D &end);

  /**
   * @brief Updates the start and end point of the line segment and recomputes all other members accordingly.
   * @param The start point of the line segment.
   * @param The end point of the line segment.
   */
  void setPoints(const Tuple2D &start, const Tuple2D &end);

  /**
   * @brief Updates the start and end point of the line segment and recomputes all other members accordingly.
   * @param The start point of the line segment.
   * @param The end point of the line segment.
   */
  void clipStart(const Real &distance);
  void clipEnd(const Real &distance);
  void clipBoth(const Real &distance); //clipping distance for both ends, if it exceeds the length, mid point is left as point of line segment

  Tuple2D getPointAbsolute(const Real &distanceAbsolute) const;
  Tuple2D getPointRelative(const Real &distanceRelative) const;
  Tuple2D getPointMiddle() const;
};

struct ParametricFunctionCubic2D
{
  Real xA, xB, xC, xD;
  Real yA, yB, yC, yD;
  Real length; //approximate lenght of arc for parameter t in the interval [0:1], interpolated between points on function
  Real lengthRecip; // 1 / length, used for calculating absolute points and directions

  ParametricFunctionCubic2D();
  ParametricFunctionCubic2D(const Tuple2D &pointStart, const Tuple2D &pointEnd, const Tuple2D &pointRef, const Real &smoothingFactor,
                            const UInt &lengthDiscretization);

  Tuple2D getPointParametric(const Real &t) const;
  Tuple2D getPointAbsolute(const Real &distance) const;
  Tuple2D getDirectionParametric(const Real &t) const;
  Tuple2D getDirectionAbsolute(const Real &distance) const;
  Real getAngleAbsolute(const Real &distance) const;
};

struct AStarNode
{
  Real g, f;
  Cell2D cell, cellParent;
  bool open, closed, occupied;
  UInt indexOpenList;

  std::vector<std::pair<AStarNode*, Real> > neighbors;
};

struct AStarPath2D
{
  std::vector<Cell2D> cells;
  std::vector<Tuple2D> points;
  bool valid;
};

} //namespace SquirrelMotionPlanner

#endif /* SQUIRREL_8DOF_PLANNER_STRUCTURES_H_ */
