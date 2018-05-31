
#include "track.h"
#include "utility.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <limits>



Track::Track(const std::string &mapFile)
   : m_WaypointsX()
   , m_WaypointsY()
   , m_WaypointsS()
   , m_WaypointsDx()
   , m_WaypointsDy()
{
  // The max s value before wrapping around the track back to 0
  static const double maxS = 6945.554;
  std::ifstream inStream(mapFile.c_str(), std::ifstream::in);

  std::string line;
  while (std::getline(inStream, line))
  {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float dX;
    float dY;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> dX;
    iss >> dY;
    m_WaypointsX.push_back(x);
    m_WaypointsY.push_back(y);
    m_WaypointsS.push_back(s);
    m_WaypointsDx.push_back(dX);
    m_WaypointsDy.push_back(dY);
  }
}


int Track::closestWaypoint(double x, double y) const
{
  double closestLen(std::numeric_limits<double>::max());
  int closestWaypoint(0);
  for(int i = 0; i < m_WaypointsX.size(); ++i)
  {
    double mapX = m_WaypointsX[i];
    double mapY = m_WaypointsY[i];
    double dist = Utility::distance(x, y, mapX, mapY);
    if(dist < closestLen)
    {
      closestLen = dist;
      closestWaypoint = i;
    }
  }
  return closestWaypoint;
}


int Track::nextWaypoint(double x, double y, double theta) const
{
  int closestWpIdx = closestWaypoint(x,y);
  double mapX = m_WaypointsX[closestWpIdx];
  double mapY = m_WaypointsY[closestWpIdx];
  double heading = atan2( (mapY - y),(mapX - x) );
  double angle = abs(theta - heading);
  if(angle > Utility::pi() / 4)
  {
    ++closestWpIdx;
  }
  return closestWpIdx;
}


std::vector<double> Track::getFrenet(double x, double y, double theta) const
{
  int next_wp = nextWaypoint(x,y, theta);
  int prev_wp = next_wp - 1;
  if(next_wp == 0)
  {
    prev_wp  = m_WaypointsX.size() - 1;
  }

  double n_x = m_WaypointsX[next_wp]- m_WaypointsX[prev_wp];
  double n_y = m_WaypointsY[next_wp]- m_WaypointsY[prev_wp];
  double x_x = x - m_WaypointsX[prev_wp];
  double x_y = y - m_WaypointsY[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = Utility::distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000 - m_WaypointsX[prev_wp];
  double center_y = 2000 - m_WaypointsY[prev_wp];
  double centerToPos = Utility::distance(center_x,center_y,x_x,x_y);
  double centerToRef = Utility::distance(center_x,center_y,proj_x,proj_y);

  if(centerToPos <= centerToRef)
  {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp; i++)
  {
    frenet_s += Utility::distance(m_WaypointsX[i],m_WaypointsY[i],m_WaypointsX[i+1],m_WaypointsY[i+1]);
  }

  frenet_s += Utility::distance(0, 0, proj_x,proj_y);

  return {frenet_s, frenet_d};
}


std::vector<double> Track::getXY(double s, double d) const
{
  int prev_wp = -1;

  while(s > m_WaypointsS[prev_wp + 1] && prev_wp < static_cast<int>(m_WaypointsS.size() - 1))
  {
    ++prev_wp;
  }

  int wp2 = (prev_wp + 1) % m_WaypointsX.size();

  double heading = atan2((m_WaypointsY[wp2]-m_WaypointsY[prev_wp]),(m_WaypointsX[wp2]-m_WaypointsX[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s - m_WaypointsS[prev_wp]);

  double seg_x = m_WaypointsX[prev_wp] + seg_s * cos(heading);
  double seg_y = m_WaypointsY[prev_wp] + seg_s * sin(heading);

  double perp_heading = heading - Utility::pi() / 2;

  double x = seg_x + d * cos(perp_heading);
  double y = seg_y + d * sin(perp_heading);

  return {x, y};
}

