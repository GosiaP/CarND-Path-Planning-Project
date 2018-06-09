#ifndef ROAD_H
#define ROAD_H

#include <vector>
#include <assert.h>
#include "car.h"
#include "utility.h"


  struct Path
  {
    typedef std::vector<double> TCoordinates;

    TCoordinates x, y;

    Path()
      : x()
      , y()
    { }

    Path(const TCoordinates &xCoord, const TCoordinates &yCoord)
      : x(xCoord)
      , y(yCoord)
    { }

    size_t size() const
    {
      return x.size();
    }

    bool empty() const
    {
      return x.empty();
    }

    void append(const double x_, const double y_)
    {
      x.push_back(x_);
      y.push_back(y_);
    }
  };

  struct LaneInfo
  {
    struct CarOnLane
    {
      bool exist;
      int id;
      double gap;
      double speed;

      CarOnLane()
        : exist(false)
        , id(-1)
        , gap(Utility::INF)
        , speed(Utility::INF)
      {
      }

      CarOnLane(bool _exist, int _id, double _gap, double _speed)
        : exist(_exist)
        , id(_id)
        , gap(_gap)
        , speed(_speed)
      {
      }
    };

    CarOnLane front;
    CarOnLane back;

    LaneInfo();
    bool isUsable() const;
    bool isFree() const;
    double getSpeed() const;
  };


  struct LaneInfoOnRoad : public std::vector<LaneInfo>
  {
    LaneInfoOnRoad(std::size_t s);
    static LaneInfoOnRoad create(double ego_s_, Traffic const& traffic);
  };


  struct Traffic
  {
    EgoCar ego;         // ego car
    Path previous_path; // previous path data given to the traffic planner
    FrenetPnt path_end; // previous path's end s and d values
    std::vector<NonEgoCar> otherCars; // list of all other cars on the same side of the road.

    Traffic(nlohmann::json const& j);
  };

#endif // ROAD_H
