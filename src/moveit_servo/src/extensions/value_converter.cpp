#include "moveit_servo/extensions/value_converter.h"

double convertToROS(double value, InputValueType input_type)
{
  switch (input_type)
  {
    case InputValueType::DEGREE:
    {
      constexpr double deg2rad = M_PI / 180.0;
      return value * deg2rad;
    }
    case InputValueType::MILLIMETER:
    {
      return value / 1000.0;
    }
    case InputValueType::CENTIMETER:
    {
      return value / 100.0;
    }
    case InputValueType::HERZ:
    {
      return 1.0 / value;
    }
    default:
      return value;
  }
}
