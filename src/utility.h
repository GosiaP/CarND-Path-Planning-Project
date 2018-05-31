#ifndef UTILITY_H
#define UTILITY_H

#include <math.h>
#include <limits>

class Utility
{
public:


  static constexpr double pi()
  {
    return M_PI;
  }
  
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
    while (a > M_PI)
    {
      a -= 2.0f * M_PI;
    }
    while (a < -M_PI)
    {
      a += 2.0f * M_PI;
    }
    return a;
  }

  /**
   * @brief Retrieves vector length.
   * @param x vector x component
   * @param y vector y component
   * @return |v|
   */
  static double vector_len(double x, double y)
  {
    return sqrt(x * x + y * y);
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
   * Calculates euclidian distance.
   * @param x1 - point 1 x
   * @param y1 - point 1 y
   * @param x2 - point 2 x
   * @param y2 - point 2 y
   * @return distnace.
   */
  static double distance(double x1, double y1, double x2, double y2)
  {
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
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
};

#endif /* TOOLS_H */
