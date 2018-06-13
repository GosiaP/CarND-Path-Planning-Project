#ifndef TRAFFIC_PLANNER_H
#define TRAFFIC_PLANNER_H

#include <vector>
#include <assert.h>
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

  /**
  * Is responsible for estiamation of current situation on a road.
  * Best lane for ego car is estimated and coresponding action
  * will be started. The action is represented by behaviour.
  */
  class BehaviourPlanner
  {
  private:
    Behaviour mBehaviour;

    double cte(EgoCarLocalization const& egoCarLo) const;

    /**
    * Discrete cost function to find current best lane
    * that will be target lane for ego car.
    */
    int findBestLane(EgoCarLocalization const& egoCarLo, LaneInfoOnRoad const& roadLI) const;
  
  public:
    BehaviourPlanner();
    void getNextBehaviour(HighwayMap const& map, EgoCarLocalization const& egoCarLo, LaneInfoOnRoad const &roadLI);

    Behaviour const& getBehaviour() const 
    {
      return mBehaviour;
    }
  };

  /**
  * Encasulates planner to find best trajectory for ego car.
  * Uses ego car planning state (behaviour), sensor fusion predictions to find the trajectory.
  * Ensures that ego using this trajectory with have no collision with
  * others cars and will not violoate speed limit.
  */
  class TrafficPlanner
  {
  public:
    TrafficPlanner(const HighwayMap &map);

    /**
    * Provides a new path of ego car.
    */
    Path getEgoCarPath(Traffic const& traffic);

  private:
    HighwayMap        mMap;
    BehaviourPlanner  mPlanner;
    Behaviour         mBehaviour;

    /**
    * Ensures the limit speed of ego car is not violated.
    */
    void adaptSpeed(LaneInfoOnRoad const& roadLaneInfo, EgoCar const& ego);

    /**
    * Determine a discrete sequence of trajectory base on
    * previous and current ego car position. Interpolates the postion
    * to cubic spline curve.
    */
    tk::spline interpolatePath(EgoCarLocalization const& egoCarLo) const;

    Path createTrajectory(EgoCarLocalization const& egoCarLo, Traffic const& traffic) const;
  };

#endif // TRAFFIC_PLANNER_H
