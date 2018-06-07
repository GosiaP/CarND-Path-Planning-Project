
#include "utility.h"
#include "traffic_planner.h"



BehaviourPlanner::BehaviourPlanner()
  : mBehaviour()
{
}

int BehaviourPlanner::findBestLane(EgoCarLocalization const& egoCarLo, LaneInfoOnRoad const& roadLI) const
{
  using best_lane_info_t = std::tuple<int, double, LaneInfo>;

  std::vector<best_lane_info_t> bli;
  {
    int lane_id = 0;
    std::transform(roadLI.begin(), roadLI.end(), 
      std::back_inserter(bli), [&](LaneInfo const& li)
      {
        return std::make_tuple(lane_id++, li.getSpeed(), li);
      });
  }

  // pick only clear lanes with infinite speeds 
  bli.erase(std::remove_if(bli.begin(), bli.end(),
    [&](best_lane_info_t const& li)
      {
        return
          !std::get<2>(li).isFree() && std::get<1>(li) < Utility::INF;
      }), bli.end());

  // sort lanes by distance to reference lane
  std::sort(bli.begin(), bli.end(),
    [&](auto const& l, auto const& r) {
    return abs(std::get<0>(l) - egoCarLo.lane_id) < abs(std::get<0>(l) - egoCarLo.lane_id);
  });

  return !bli.empty() ? std::get<0>(bli.front()) : egoCarLo.lane_id;
}

double BehaviourPlanner::cte(EgoCarLocalization const& egoCarLo) const
{
  return egoCarLo.ego.d - Utility::getLaneCenter_D(mBehaviour.lane_id);
}

void BehaviourPlanner::getNextBehaviour(HighwayMap const& map,
                                        EgoCarLocalization const& egoCarLo,
                                        LaneInfoOnRoad const &roadLI)
{
  std::cout << "next behaviour" << std::endl;

  // consider safety margin
  const auto max_speed = Utility::mph2mps(Utility::SPEED_LIMIT) - Utility::ACCELER_LIMIT;

  if (mBehaviour.state == Initial)
  {
    mBehaviour.changing_into_lane_id = -1;
    mBehaviour.lane_id = egoCarLo.lane_id;
    mBehaviour.speed = 0.;
    mBehaviour.state = KeepLane;
    mBehaviour.dist_s = egoCarLo.ego.s;
  }

  int best_lane_id = findBestLane(egoCarLo, roadLI);

  while (true)
  {
    double distInState = Utility::normalize_s(egoCarLo.ego.s - mBehaviour.dist_s);

    if (mBehaviour.state == KeepLane)
    {
      assert(mBehaviour.lane_id == egoCarLo.lane_id);

      mBehaviour.speed = max_speed;
      mBehaviour.changing_into_lane_id = -1;

      // evaluate lane changes if current lane is free or not
      if ( roadLI[mBehaviour.lane_id].front.gap < Utility::MIN_HORIZON 
        && roadLI[mBehaviour.lane_id].front.speed < max_speed)
      {
        // change if it is not the best one
        if (roadLI[mBehaviour.lane_id].front.speed + Utility::ACCELER_LIMIT < roadLI[best_lane_id].front.speed)
        {
          mBehaviour.changing_into_lane_id = best_lane_id;
          if (mBehaviour.state != PrepareChangingLane)
          {
            // Set the new state and the reference position
            mBehaviour.state = PrepareChangingLane;
            mBehaviour.dist_s = egoCarLo.ego.s;
          }
          continue;
        }
      }
      break;
    }
    else
      if (mBehaviour.state == PrepareChangingLane)
      {
        assert(mBehaviour.changing_into_lane_id != egoCarLo.lane_id);

        mBehaviour.lane_id = egoCarLo.lane_id + ((mBehaviour.changing_into_lane_id > egoCarLo.lane_id) ? 1 : -1);

        if (roadLI[mBehaviour.lane_id].isUsable())
        {
          if (mBehaviour.state != ChangingLane)
          {
            // set the new state and the reference position
            mBehaviour.state = ChangingLane;
            mBehaviour.dist_s = egoCarLo.ego.s;
          }
          continue;
        }
        else 
          if (mBehaviour.changing_into_lane_id != best_lane_id || distInState > 500)
          {
            // waiting too long
            mBehaviour.lane_id = egoCarLo.lane_id;
            if (mBehaviour.state != KeepLane)
            {
              // set the new state and the reference position
              mBehaviour.state = KeepLane;
              mBehaviour.dist_s = egoCarLo.ego.s;
            }
            continue;
          }
          else
          { 
            // not feasible
            if (roadLI[egoCarLo.lane_id].front.gap < 30)
            {
              // can't change lane, try slow down
              if (roadLI[mBehaviour.lane_id].back.gap < Utility::LANE_CHANGE_BACK_DIST)
              {
                mBehaviour.speed =
                  std::min(mBehaviour.speed, roadLI[mBehaviour.lane_id].back.speed - 0.5);
              }
              else
              {
                mBehaviour.speed =
                  std::min(mBehaviour.speed, roadLI[mBehaviour.lane_id].front.speed - 0.5);
              }
            }
            // don't change lane
            mBehaviour.lane_id = egoCarLo.lane_id;
          }
          break;
        }
        else
          if (mBehaviour.state == ChangingLane)
          {
            if ( egoCarLo.lane_id == mBehaviour.lane_id
              && std::fabs(cte(egoCarLo)) < 0.2
              && 100 < distInState)
            {
              // lane change done
              if (mBehaviour.changing_into_lane_id >= 0 && mBehaviour.changing_into_lane_id != egoCarLo.lane_id)
              {
                if (mBehaviour.state != PrepareChangingLane)
                {
                  // set the new state and the reference position
                  mBehaviour.state = PrepareChangingLane;
                  mBehaviour.dist_s = egoCarLo.ego.s;
                }
                continue;
              }

              mBehaviour.changing_into_lane_id = -1;
              if (mBehaviour.state != KeepLane)
              {
                // set the new state and the reference position
                mBehaviour.state = KeepLane;
                mBehaviour.dist_s = egoCarLo.ego.s;
              }
              continue;
            }

            if (std::fmin(roadLI[mBehaviour.lane_id].front.gap, roadLI[mBehaviour.lane_id].back.gap) < 5.)
            {
              // stay on the lane
              mBehaviour.lane_id = egoCarLo.lane_id;
              mBehaviour.changing_into_lane_id = -1;
            }

            std::cout << "changing to lane" << mBehaviour.lane_id << std::endl;

            mBehaviour.speed = max_speed;
            break;
          }
    break;
  }

  mBehaviour.speed = std::max(0., std::min(max_speed, mBehaviour.speed));

  std::cout
    << " changing lane " << mBehaviour.changing_into_lane_id << std::endl
    << " target lane   " << mBehaviour.lane_id << std::endl
    << " target speed  " << std::setprecision(2) << std::fixed
    << Utility::mps2mph(mBehaviour.speed) << std::endl;
}


TrafficPlanner::TrafficPlanner(const HighwayMap &map)
  : mMap(map)

{
}

Path TrafficPlanner::getEgoCarPath(Traffic const& traffic)
{
  EgoCarLocalization egoCarLI(traffic);

  LaneInfoOnRoad roadLI = LaneInfoOnRoad::create(egoCarLI.ego.s, traffic);

  mPlanner.getNextBehaviour(mMap, egoCarLI, roadLI);
  mBehaviour = mPlanner.getBehaviour();
  
  adaptSpeed(roadLI, egoCarLI.ego);

  return createTrajectory(egoCarLI, traffic);
}


tk::spline TrafficPlanner::interpolatePath(EgoCarLocalization const& egoCarLo) const
{
  auto const target_d = Utility::getSafeLaneCenter_D(mBehaviour.lane_id);

  std::cout << "trajectory" << std::endl
    << " target lane " << mBehaviour.lane_id << std::endl
    << " target speed " << std::setprecision(1) << Utility::mps2mph(mBehaviour.speed) << std::endl
    << " target d " << std::setprecision(2) << target_d << std::endl;

  // trajectory points
  Path newPath;

  // Build a path tangent to the previous end states
  newPath.append( egoCarLo.prev_x, egoCarLo.prev_y);
  newPath.append( egoCarLo.ego.x, egoCarLo.ego.y);

  // add 3 more points spaced 30 m
  for (int i = 1; i <= 3; i++)
  {
    std::vector<double> xy = mMap.getXY(egoCarLo.ego.s + 30 * i, target_d);
    newPath.append(xy[0], xy[1]);
  }

  // change the points to the reference coordinate
  for (int i = 0; i < newPath.size(); i++)
  {
    const auto dx = newPath.x[i] - egoCarLo.ego.x;
    const auto dy = newPath.y[i] - egoCarLo.ego.y;
    newPath.x[i] = dx * std::cos(-egoCarLo.ego.yaw) - dy * std::sin(-egoCarLo.ego.yaw);
    newPath.y[i] = dx * std::sin(-egoCarLo.ego.yaw) + dy * std::cos(-egoCarLo.ego.yaw);
  }

  // match splines
  tk::spline spl;
  spl.set_points(newPath.x, newPath.y);

  return spl;
}


Path TrafficPlanner::createTrajectory(EgoCarLocalization const& egoCarLo, Traffic const& traffic) const
{
  tk::spline fSpline = interpolatePath(egoCarLo);

  // ddd previous path for continuity
  Path path = traffic.previous_path;

  // set a horizon of 30 m
  auto const target_x = 30;
  auto const target_y = fSpline(target_x);
  auto const target_dist = Utility::v_len(target_x, target_y);
  auto const t = target_x / target_dist * 0.02;

  // sample the spline curve to reach the target speed
  for (auto i = 1; i <= Utility::PATH_ITEM_COUNT - path.x.size(); i++)
  {
    auto x = i * t * mBehaviour.speed;
    auto y = fSpline(x);
    // transform back to world coordinates
    auto xw = x * std::cos(egoCarLo.ego.yaw) - y * sin(egoCarLo.ego.yaw) + egoCarLo.ego.x;
    auto yw = x * std::sin(egoCarLo.ego.yaw) + y * cos(egoCarLo.ego.yaw) + egoCarLo.ego.y;
    // append the trajectory points
    path.append(xw, yw);
  }
  return path;
}

void TrafficPlanner::adaptSpeed(LaneInfoOnRoad const& roadLI, EgoCar const& ego)
{
  const LaneInfo &li = roadLI[mBehaviour.lane_id];

  if (li.front.gap < 30.)
  {
    if (li.front.gap < 15.)
      mBehaviour.speed = std::max(0., std::min(mBehaviour.speed, li.front.speed - Utility::ACCELER_LIMIT));
    else
      mBehaviour.speed = std::min(mBehaviour.speed, li.front.speed);

    std::cout
      << " reduce speed to " << Utility::mps2mph(mBehaviour.speed) << " mph " << std::endl
      << " keep safety gap in "
      << li.front.gap << " m"
      << std::endl;
  }

  if (mBehaviour.speed < ego.speed) {
    // slow down
    mBehaviour.speed = std::max(mBehaviour.speed, ego.speed - Utility::ACCELER_LIMIT);
  }
  else if (ego.speed < mBehaviour.speed) {
    // speed up
    mBehaviour.speed = std::min(mBehaviour.speed, ego.speed + Utility::ACCELER_LIMIT);
  }
}








