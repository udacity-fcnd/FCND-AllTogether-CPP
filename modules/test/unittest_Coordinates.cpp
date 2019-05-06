#include <iostream>
#include <iomanip>
#include <string>
#include <cassert>
#include "modules/math/V3F.h"
#include "modules/common/Coordinates.h"
#include <GeographicLib/GeoCoords.hpp>

using namespace std;
using namespace GeographicLib;

int main() {
  // Sample forward calculation
  V3F global_home(37.792480, -122.397450, 0.0);

  V3F global_position(37.793480, -122.398450, 85.5);

  V3F local_position = global_to_local(global_position, global_home);

  V3F global_pos_test = local_to_global(local_position, global_home);

  cout << global_pos_test.x << " " << global_pos_test.y << " "
       << global_pos_test.z << endl;

  assert(global_position == global_pos_test);

  cout << local_position.x << " " << local_position.y << " " << local_position.z
       << endl;

  return 0;
}