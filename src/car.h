#ifndef CAR_H
#define CAR_H

#include "json.hpp"

struct Traffic;


struct NonEgoCar
{
  // unique identifier of a car
  int uid;

  // position in map coordinates
  double x, y;

  // velocity in map coordinates direction (m/s)
  double vx, vy;

  // position in Frenet coordinates
  double s, d;

  NonEgoCar(nlohmann::json const& j);
};


struct EgoCar {
  // ego position in map coordinates
  double x, y;

  // ego yaw angle in map coordinates (radians)
  double yaw;

  // ego speed (m/s)
  double speed;

  // position in Frenet coordinates
  double s, d;

  EgoCar();
  EgoCar(nlohmann::json const& j);
};

/**
* Enacapsulates current ego position and its prvious position
* inclusive a lane number th ecar is located.
*/
struct EgoCarLocalization
{
  EgoCar ego; // ego car
  double prev_x, prev_y; // previous x, y positions  
  int lane_id; // reference lane

  EgoCarLocalization(Traffic const& trf);
};


#endif // CAR_H
