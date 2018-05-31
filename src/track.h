#ifndef MAP_H
#define MAP_H

#include <vector>
#include <string>


class Track
{
public:
  Track(const std::string &file);

  int closestWaypoint(double x, double y) const;
  int nextWaypoint(double x, double y, double theta) const;
  std::vector<double> getFrenet(double x, double y, double theta) const;
  std::vector<double> getXY(double s, double d) const;

  std::size_t getNumOfData() const { return m_WaypointsX.size(); }


private:
  std::vector<double> m_WaypointsX; // list of waypoint x coordinates
  std::vector<double> m_WaypointsY; // list of waypoint y coordinates
  std::vector<double> m_WaypointsS; // list of waypoint s coordinates
  std::vector<double> m_WaypointsDx; // list of waypoint deltas x
  std::vector<double> m_WaypointsDy;  // list of waypoint deltas y
};

#endif // MAP_H
