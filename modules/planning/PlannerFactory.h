#pragma once

#include <string>
#include <memory>
#include "modules/planning/AstarPlanner.h"

using std::shared_ptr;
using std::string;
using namespace planning;

inline shared_ptr<AstarPlanner> CreatePlanner(string config,
                                              const float altitude,
                                              string planner_type = "Astar") {
  /**
   * create Astar planner
   */
  auto planner = std::make_shared<AstarPlanner>();
  planner->run_astar_planner(config, altitude);

  return planner;
}