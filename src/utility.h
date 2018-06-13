#ifndef _MY_UTILITY_H
#define _MY_UTILITY_H

#include <math.h>
#include <limits>
#include <algorithm>
#include <cstddef>

  struct FrenetPnt
  {
    double s;
    double d;
  };


  
  struct Utility
  {
    static const double pi()
    {
      return 3.14159265358979323846;
      //return M_PI;
    }

    // safe lane distance 
    static const  double LANE_MARGIN;

    // constant for infinity
    static const double INF;

    // paths(trajectory) length -  50 points
    static const std::size_t PATH_ITEM_COUNT = 50u;

    // safe distance buffer for lane changes in front of ego car
    static const double LANE_CHANGE_FRONT_DIST; // [m]

    // safe distance buffer for lane changes in back of ego car
    static const double LANE_CHANGE_BACK_DIST; // [m]

    // horizon in the lane environment model
    static const double MIN_HORIZON; // [m]

    // speed limit in mph
    static const double SPEED_LIMIT; // [mph]

    // target acceleration
    static const double ACCELER_LIMIT; // [m/s^2]

  
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

    /**
    * Provides lane center from the origin in d Frenet.
    * @param lane_id - lane number.
    * @return d Frenet coordinate.
    */
    static double getLaneCenter_D(int lane_id_)
    {
      return (0.5 + lane_id_) * 4.0;
    }

    /**
    * Provides lane center with safety margin in border lanes.
    * @param lane_id - lane number.
    * @return d Frenet coordinate.
    */
    static double getSafeLaneCenter_D(int lane_id)
    {
      auto c = getLaneCenter_D(lane_id);

      if (lane_id == 0)
        c += LANE_MARGIN;
      else if (lane_id == 2)
        c -= LANE_MARGIN;

      return c;
    }

    /**
    * Provides a lane number from road center.
    * @param d - d coordinate in Frenet
    * @return lane number.
    */
    static int getLaneId(double d)
    {
      return std::max(0, int(d) / 4);
    }
  };



#endif /* _MY_UTILITY_H */
