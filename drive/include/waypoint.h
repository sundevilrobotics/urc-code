#ifndef WAYPOINT_H
#define WAYPOINT_H

class Waypoint
{
private:
  double x, y, heading;

public:
  Waypoint(double, double);
  double getX();
  double getY();
  // double getHeading();
};

#endif
