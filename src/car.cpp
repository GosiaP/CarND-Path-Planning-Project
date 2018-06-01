
#include "car.h"
#include "utility.h"
#include "track.h"


Car::Car(double _x, double _y, double _s, double _d, double _yaw, double _speed, const TPath &_path)
  : x(_x)
  , y(_y)
  , s (_s)
  , d(_d)
  , yaw(_yaw)
  , speed(_speed)
  , path(_path)
{
}

Car::Car(const nlohmann::json &j)
  : x(j["x"])
  , y(j["y"])
  , s(j["s"])
  , d(j["d"])
  , yaw(Utility::deg2rad(j["yaw"]))
  , speed(Utility::mph2mps(j["speed"]))
  , path(j["previous_path_x"], j["previous_path_y"])
{
}


void Car::carToMapCoordinates(double &inOutX, double &inOutY) const
{
  double xRef = inOutX;
  double yRef = inOutY;
  inOutX = xRef * cos(yaw) - yRef * sin(yaw) + x;
  inOutY = xRef * sin(yaw) + yRef * cos(yaw) + y;
}

void Car::mapToCarCoordinates(double &inOutX, double &inOutY) const
{
  double shiftX = inOutX - x;
  double shiftY = inOutY - y;
  inOutX = shiftX * cos(-yaw) - shiftY * sin(-yaw);
  inOutY = shiftX * sin(-yaw) + shiftY * cos(-yaw);
}

double Car::getSafetyDistance(double friction) const
{
  return (speed * speed) / (2.0 * friction * 9.81);
}

int Car::getLaneNum() const
{
  return std::max(0, int(d) / 4);
}


//------------------------------


TrafficPlanner::TrafficPlanner(const Track &track)
  : mTrack(track)
  , mEgoCar(0.0, 0.0, 0.0, EGO_CAR_LANE_NUM, 0.0, 0.0, TPath())
  , mOtherCars()
{
}

void TrafficPlanner::update(const nlohmann::json &j)
{
  mEgoCar = Car(j);
  mOtherCars.clear();

  auto sensor_fusion = j["sensor_fusion"];
  for (int i = 0; i < sensor_fusion.size(); ++i)
  {
    auto d = sensor_fusion[i];
    // id, x, y, vx, vy, s, d
    Car car(d[1], d[2], d[5], d[6],
      Utility::vector_angle(d[3], d[4]),
      Utility::vector_len(d[3], d[4]),
      TPath());
    mOtherCars.push_back(car);
  }
}

Car TrafficPlanner::predictCarKeepingItsLane(const Car& car, double dt) const
{
  double pred_s = car.s + dt * car.speed;

  std::vector<double> xy = mTrack.getXY(pred_s, car.d);
  Car newCar(xy[0],
              xy[1],
              pred_s,
              car.d,
              car.yaw,
              car.speed,
              TPath());
  return newCar;
}

TPath TrafficPlanner::getEgoCarPath() const
{
  TPath path;
  Car car = mEgoCar;
  for (int i = 0; i < PATH_ITEM_COUNT; ++i)
  {
    car = predictCarKeepingItsLane(car, 0.2);
    path.x.push_back(car.x);
    path.y.push_back(car.y);
  }
  return path;
}



