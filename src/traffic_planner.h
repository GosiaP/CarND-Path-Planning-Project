#ifndef TRAFFIC_PLANNER_H
#define TRAFFIC_PLANNER_H

#include <vector>
#include <assert.h>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "car.h"
#include "map.h"
#include "road.h"
#include "spline.h"

  enum EState
  {
    Initial,
    KeepLane,
    PrepareChangingLane,
    ChangingLane
  };

  struct Behaviour
  {
    EState state;  // current state
    double dist_s; // distance car is in current state (s frenet coordinate)
    int lane_id;   // current lane
    double speed;
    int changing_into_lane_id;

    Behaviour()
      : state(Initial)
      , dist_s(0.0)
      , lane_id(1)
      , speed(0.)
      , changing_into_lane_id(-1)
    {
    }
  };

  class BehaviourPlanner
  {
  private:
    Behaviour mBehaviour;

    double cte(EgoCarLocalization const& egoCarLo) const;

    // discrete cost function
    int findBestLane(EgoCarLocalization const& egoCarLo, LaneInfoOnRoad const& roadLI) const;
  
  public:
    BehaviourPlanner();
    Behaviour const& getBehaviour() const
    {
      return mBehaviour;
    }

    void getNextBehaviour(HighwayMap const& map, EgoCarLocalization const& egoCarLo, LaneInfoOnRoad const &roadLI);
  };


  class TrafficPlanner
  {
  public:
    TrafficPlanner(const HighwayMap &map);
    Path getEgoCarPath(Traffic const& traffic);

  private:
    HighwayMap        mMap;
    BehaviourPlanner  mPlanner;
    Behaviour         mBehaviour;

    void adaptSpeed(LaneInfoOnRoad const& roadLaneInfo, EgoCar const& ego);
    tk::spline interpolatePath(EgoCarLocalization const& egoCarLo) const;
    Path createTrajectory(EgoCarLocalization const& egoCarLo, Traffic const& traffic) const;
  };

#endif // TRAFFIC_PLANNER_H
