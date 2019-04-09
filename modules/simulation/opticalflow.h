#pragma once

#include "modules/common/Common.h"
#include "modules/math/Random.h"
#include "modules/math/Quaternion.h"
#include "third_party/matrix/math.hpp"
#include <random>

class opticalflow
{
public:
  float omega_factor = 1.25f; // Taken directly from Michael Hamer's code on estimation on crazyflie
  float N_pixel = 30.0f;
  float aperture = 4.2f * M_PI / 180.f;
  float fx_stddev = 0.0001f;
  float fy_stddev = 0.0001f;
  void opticalflow_sensor(float dt, V3F position, V3F velocity, SLR::Quaternion<float> attitude, V3F omega, float &x_measurement, float &y_measurement);

};
