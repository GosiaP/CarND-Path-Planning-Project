
#include "car.h"
#include "utility.h"
#include "road.h"


NonEgoCar::NonEgoCar(nlohmann::json const& j)
  : uid(j[0].get<int>())
  , x(j[1])
  , y(j[2])
  , vx(j[3])
  , vy(j[4])
  , s(j[5])
  , d(j[6])
{
}


EgoCar::EgoCar()
  : x(0.0)
  , y(0.0)
  , yaw(0.0)
  , speed(0.0)
  , s(0.0)
  , d(0.0)
{
}

EgoCar::EgoCar(nlohmann::json const& j)
  : x(j["x"])
  , y(j["y"])
  , yaw(Utility::deg2rad(j["yaw"]))
  , s(j["s"])
  , d(j["d"])
  , speed(Utility::mph2mps(j["speed"]))
{
}

EgoCarLocalization::EgoCarLocalization(Traffic const& trf)
{
  // estimate reference position based on current and past data.
  // The estimations are from the lessons and from Q / A walkthrough
  if (trf.previous_path.size() < 2u)
  {
    // first time calculation
    ego = trf.ego;
    prev_x = ego.x - std::cos(ego.yaw);
    prev_y = ego.y - std::sin(ego.yaw);
  }
  else
  {
    // standard case, path calculated before
    ego.x = *(trf.previous_path.x.end() - 1);
    ego.y = *(trf.previous_path.y.end() - 1);

    prev_x = *(trf.previous_path.x.end() - 2);
    prev_y = *(trf.previous_path.y.end() - 2);

    ego.yaw = std::atan2(ego.y - prev_y, ego.x - prev_x);
    ego.speed = Utility::distance(prev_x, prev_y, ego.x, ego.y) / 0.02;

    ego.s = trf.path_end.s;
    ego.d = trf.path_end.d;
  }

  lane_id = Utility::getLaneId(ego.d);
}


