#ifndef COORDINATE_H
#define COORDINATE_H

#include <tgmath.h>

class Coordinate {
private:
  double x, y;
public:
  Coordinate() {
    this->x = 0;
    this->y = 0;
  }
  Coordinate(double x, double y) {
    this->x = x;
    this->y = y;
  }
  /**
   * @brief   Get the X component of this coordinate.
   * @return  The X component of this coordinate.
   */
  double getX(void) const { return this->x; }
  /**
   * @brief   Get the Y component of this coordinate.
   * @return  The Y component of this coordinate.
   */
  double getY(void) const { return this->y; }
  /**
   * @brief   Set the Y component of this coordinate.
   * @param y The new Y component.
   */
  void setX(double x) { this->x = x; }
  /**
   * @brief   Set the Y component of this coordinate.
   * @param y The new Y component.
   */
  void setY(double y) { this->y = y; }

  /**
   * @brief   Calculate the euclidean distance between 2 coordinates.
   * @param c The coordinate to calculate the distance to.
   * @return  The euclidean distance between this coordinate and the coordinate c.
   */
  double dist(const Coordinate &c) const {
    return sqrt(pow(this->x - c.x, 2) + pow(this->y - c.y, 2));
  }
};

#endif // COORDINATE_H
