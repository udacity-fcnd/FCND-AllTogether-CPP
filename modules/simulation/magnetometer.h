#pragma once

#include <math.h>
#include "modules/common/Common.h"
#include "modules/math/Random.h"
#include "modules/math/Quaternion.h"
#include "third_party/matrix/math.hpp"
#include <random>

class magnetometer
{
public:
  V3F mag;
  float fx_stddev = 0.0001f;
  float fy_stddev = 0.0001f;
  float fz_stddev = 0.0001f;
  void magnetometer_sensor(float declination, SLR::Quaternion<float> attitude, V3F &mag_measurement);
};
