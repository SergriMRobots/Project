#ifndef VALUE_CONVERTER_H
#define VALUE_CONVERTER_H
#include <cmath>
enum class InputValueType
{
  CENTIMETER,
  MILLIMETER,
  DEGREE,
  HERZ
};
double convertToROS(double value, InputValueType input_type);

#endif  // VALUE_CONVERTER_H
