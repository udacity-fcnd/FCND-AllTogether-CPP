#include <string>
#include "modules/planning/AstarPlanner.h"
#include "modules/planning/PlannerFactory.h"

using namespace std;
using namespace planning;

int main() {
  // test Astar planner
  string map = "../misc/data/colliders.csv";
  shared_ptr<AstarPlanner> planner = CreatePlanner(map, 10.0);

  return 0;
}