#pragma once

#include "modules/common/Common.h"
#include "modules/math/Random.h"
#include "modules/math/Quaternion.h"
#include "third_party/matrix/math.hpp"
#include <random>

class rangefinder
{
public:
  float fd_stddev = 0.0001f;
  void range_sensor(V3F position, SLR::Quaternion<float> attitude, float &measurement);
};

