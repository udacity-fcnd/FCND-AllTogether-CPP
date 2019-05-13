#pragma once

#include "modules/common/DataSource.h"
#include "modules/planning/Trajectory.h"

class BaseQuadPlanner : public DataSource {
public:
  BaseQuadPlanner() {
  }
  virtual ~BaseQuadPlanner() {
  }

  virtual void Init() {
  }
  virtual Trajectory RunPlanner() {
  }

  Trajectory trajectory;

private:
  string _config;
};