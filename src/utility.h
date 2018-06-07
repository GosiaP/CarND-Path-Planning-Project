#ifndef UTILITY_H
#define UTILITY_H

#include <math.h>
#include <limits>
#include <algorithm>

struct FrenetPnt
{
  double s;
  double d;
};



struct Utility
{
  static constexpr double pi()
  {
    return 3.14159265358979323846;
    //return M_PI;
  }

  // safe lane distance 
  static constexpr  double LANE_MARGIN = 0.05; // [m]

  // constant for infinity
  static constexpr double INF = std::numeric_limits<double>::infinity();

  // paths length -  50 points
  static constexpr std::size_t PATH_ITEM_COUNT = 50u;

  // safe distance buffer for lane changes in front of me
  static constexpr double LANE_CHANGE_FRONT_DIST = 15; // [m]

  // safe distance buffer for lane changes at the back
  static constexpr double LANE_CHANGE_BACK_DIST = 7; // [m]

  // horizon in the lane environment model
  static constexpr double MIN_HORIZON = 50; // [m]

  // speed limit in mph
  static constexpr double SPEED_LIMIT = 50.0; // [mph]

  // target acceleration
  static constexpr double ACCELER_LIMIT = 0.16; // [m/s^2]

  
  // For converting back and forth between radians and degrees.
  static double deg2rad(double x)
  {
    return x * pi() / 180;
  }
  static double rad2deg(double x)
  {
    return x * 180 / pi();
  }

  static double mph2mps(double mph)
  {
    return mph * 0.44704;
  }


  static double mps2mph(double x)
  {
    return x / 0.44704;
  }

  static double meters2miles(double x)
  {
    return x / 1609.34;
  }

  /**
  * @brief Retrieves vector length.
  * @param x vector x component
  * @param y vector y component
  * @return |v|
  */
  static double v_len(double x, double y)
  {
    return sqrt(x * x + y * y);
  }

  /**
  * Calculates euclidian distance.
  * @param x1 - point 1 x
  * @param y1 - point 1 y
  * @param x2 - point 2 x
  * @param y2 - point 2 y
  * @return distnace.
  */
  static double distance(double x1, double y1, double x2, double y2)
  {
    return v_len(x2 - x1, y2 - y1);
  }

  /**
   * Calculates a difference between two angles (in radians)
   */
  static double calc_angle_diff(double  a1, double a2)
  {
    const double diff = Utility::normalize_angle(a2) - Utility::normalize_angle(a1);
    return Utility::normalize_angle(diff);
  }

  /**
   * Normalizes angle in radians
   */
  static double normalize_angle(double a)
  {
    while (a > pi())
    {
      a -= 2.0f * pi();
    }
    while (a < -pi())
    {
      a += 2.0f * pi();
    }
    return a;
  }

  static double get_max_s()
  {
    return 6945.554;
  }

  static double normalize_s(double s_)
  {
    while (s_ < 0.) {
      s_ += get_max_s();
    }
    return s_;
  }


  /**
   * Calculates vector angle in radians.
   * @param x - vector x component
   * @param y - vector y component
   * @return vector angle.
   */
  static double vector_angle(double x, double y)
  {
    return atan2(y, x);
  }

  /**
   * Checks if number is near zero (using "epsilon" accuracy)
   * @param num - floating point number
   * @return if number is near zero
   */
  static bool is_zero(double num)
  {
    return fabs(num) < std::numeric_limits<double>::epsilon();
  }


  // Lane center from the origin (d)
  static double getLaneCenter_D(int lane_id_)
  {
    return (0.5 + lane_id_) * 4.0;
  }

  // Lane center with safety margin in border lanes 
  static double getSafeLaneCenter_D(int lane_id_)
  {
    auto c = getLaneCenter_D(lane_id_);

    if (lane_id_ == 0)
      c += LANE_MARGIN;
    else if (lane_id_ == 2)
      c -= LANE_MARGIN;

    return c;
  }

  // Lane at a given distance from the road center
  static int getLaneId(double d_)
  {
    return std::max(0, int(d_) / 4);
    //return std::floor(d_ / 4.0);
  }
};

#endif /* TOOLS_H */
