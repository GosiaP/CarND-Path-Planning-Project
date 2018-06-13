
#include "utility.h"
#include "road.h"


LaneInfo::LaneInfo()
  : front(false,  -1, Utility::INF, Utility::INF)
  , back(false,  -1, Utility::INF, 0.)
{
}

bool LaneInfo::isUsable() const
{
  return
    (!front.exist || Utility::LANE_CHANGE_FRONT_DIST < front.gap)
    &&
    (!back.exist || Utility::LANE_CHANGE_BACK_DIST < back.gap);
}

bool LaneInfo::isFree() const
{
  return isUsable() && !front.exist;
}

double LaneInfo::getSpeed() const
{
  return Utility::MIN_HORIZON < front.gap
    ? Utility::INF
    : back.gap < Utility::LANE_CHANGE_BACK_DIST
    ? back.speed
    : front.speed;
}





LaneInfoOnRoad::LaneInfoOnRoad(std::size_t s)
  : std::vector<LaneInfo>(s)
{
}

LaneInfoOnRoad LaneInfoOnRoad::create(double ego_s, Traffic const& traffic)
{
  LaneInfoOnRoad laneOnRoad(3);

  auto pred_distance = traffic.previous_path.size();

  for (auto & other : traffic.otherCars)
  {
    auto lane_id = Utility::getLaneId(other.d);

    // car is on our side of the road
    if (0 <= lane_id && lane_id < 3)
    {
      auto& l = laneOnRoad[lane_id];

      // estimate positions based on sensor data
      auto other_speed = Utility::v_len(other.vx, other.vy);
      auto other_s = other.s + other_speed * pred_distance * 0.02;
      auto other_gap = std::fabs(other_s - ego_s);

      // Is other car in front of me and closer than this one?
      if (ego_s <= other_s && other_gap < l.front.gap)
      {
        l.front = LaneInfo::CarOnLane(true, other.uid, other_gap, other_speed);
      }

      // Is the other car behind me and closer than minimal horizon...
      if (other_s < ego_s && other_gap < std::min(Utility::MIN_HORIZON, l.back.gap))
      {
        l.back = LaneInfo::CarOnLane(true, other.uid, other_gap, other_speed);
      }
    }
  }

  return laneOnRoad;
}

Traffic::Traffic(nlohmann::json const& j)
{
  ego = EgoCar(j);

  // Previous path data given to the Planner
  previous_path.x = j["previous_path_x"].get<std::vector<double>>();
  previous_path.y = j["previous_path_y"].get<std::vector<double>>();
  // Previous path's end s and d values
  path_end.s = j["end_path_s"];
  path_end.d = j["end_path_d"];

  auto sensor_fusion = j["sensor_fusion"];

  for (int i = 0; i < sensor_fusion.size(); ++i)
  {
    NonEgoCar otherCar(sensor_fusion[i]);
    otherCars.push_back(otherCar);
  }
}






