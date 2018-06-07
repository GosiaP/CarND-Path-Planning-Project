#ifndef MAP_H
#define MAP_H

#include <vector>
#include <string>
#include "utility.h"



class HighwayMap
{
public:
  HighwayMap(const std::string &file);

  int closestWaypoint(double x, double y) const;
  int nextWaypoint(double x, double y, double theta) const;
  std::vector<double> getFrenet(double x, double y, double theta) const;
  std::vector<double> getXY(double s, double d) const;

  std::size_t getNumOfData() const { return mWaypointsX.size(); }


private:
  std::vector<double> mWaypointsX; // list of waypoint x coordinates
  std::vector<double> mWaypointsY; // list of waypoint y coordinates
  std::vector<double> mWaypointsS; // list of waypoint s coordinates
  std::vector<double> mWaypointsDx; // list of waypoint deltas x
  std::vector<double> mWaypointsDy;  // list of waypoint deltas y
};

#endif // MAP_H
