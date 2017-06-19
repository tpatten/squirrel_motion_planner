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

struct Tuple2D
{
  Real x, y;

  Tuple2D();
  Tuple2D(Real x, Real y);

  Tuple2D operator+(const Tuple2D &point) const;
  void operator+=(const Tuple2D &point);
  Tuple2D operator-(const Tuple2D &point) const;
  Tuple2D operator*(const Real &factor) const;
  void operator*=(const Real &factor);
  void operator/=(const Real &divisor);
  Tuple2D operator/(const Real &divisor) const;

  Real distance(const Tuple2D &point) const;
};

struct LineSegment2D
{
  Tuple2D start, end;
  Tuple2D direction;

  Real length, angle;

  LineSegment2D();
  LineSegment2D(const Tuple2D &start, const Tuple2D &end);

  void setPoints(const Tuple2D &start, const Tuple2D &end);

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
