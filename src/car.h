#ifndef Car_H
#define Car_H

#include <vector>
#include <assert.h>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

class Track;

struct TPath
{
  typedef std::vector<double> TCoordinates;

  TPath()
    : x()
    , y()
  { }

  TPath(const TCoordinates &xCoord, const TCoordinates &yCoord)
    : x(xCoord)
    , y(yCoord)
  { }

  TCoordinates x;
  TCoordinates y;
};

class Car
{
public:

  Car(double _x, double _y, double _s, double _d, double _yaw, double _speed, const TPath &_path);
  Car(const nlohmann::json &j);


  void carToMapCoordinates(double &inOutX, double &inOutY) const;

  void mapToCarCoordinates(double &inOutX, double &inOutY) const;

  double getSafetyDistance(double friction) const;

  double getSCoordInTime(double dt) const;

  int getLaneNum() const;

 
public:
  double x; // x coordinate in map coordinate system
  double y; // y coordinate in map coordinate system
  double s; // s coordinate in Frenet coordinate system 
  double d; // d coordinate in Frenet coordinate system 
  double yaw; // yaw angle of car in radians in map coordinate system
  double speed; // in m/s
  TPath path;

};



class TrafficPlanner
{
public:
  TrafficPlanner(const Track &track);

  void update(const nlohmann::json &j);
  TPath getEgoCarPath() const;

private:
  Car predictCarKeepingItsLane(const Car& car, double dt) const;

private:
  typedef std::vector<Car> TCars;

  const Track  &mTrack;
  Car           mEgoCar;    // ego (main) car
  TCars         mOtherCars; // list of all other cars on the same side of the road
  
  const static int PATH_ITEM_COUNT = 50;
  const static int EGO_CAR_LANE_NUM = 6;
  

};

#endif // Car_H
