
#include "car.h"
#include "utility.h"



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
  : x(j[1]["x"])
  , y(j[1]["y"])
  , s(j[1]["s"])
  , d(j[1]["d"])
  , yaw(Utility::deg2rad(j[1]["yaw"]))
  , speed(Utility::mph2mps(j[1]["speed"]))
  , path(j[1]["previous_path_x"], j[1]["previous_path_y"])
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

Traffic::Traffic(const nlohmann::json &j)
  : mEgoCar(j)
  , mOtherCars()
{
  auto sensor_fusion = j[1]["sensor_fusion"];

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


