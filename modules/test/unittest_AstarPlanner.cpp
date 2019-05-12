#include <algorithm>
#include <cassert>
#include <fstream>
#include <iostream>
#include <sstream>
#include "modules/planning/AstarPlanner.h"

#define eps 1e-6

using namespace std;
using namespace planning;

// read obstacal map
void readColliderMap(vector<Collider>& colliders, Point& home) {
  /**
   *  File pointer
   */
  fstream fin;

  /**
   *  open colliders file
   */
  fin.open("../misc/data/colliders.csv", std::ios::in);
  if (!fin) {
    throw std::runtime_error("AstartPlanner:Fail to read colliders.");
  }

  string line, val, name;

  /**
   * read and set home position
   */
  getline(fin, line);
  float lat0 = 0.0;
  float lon0 = 0.0;
  std::replace(line.begin(), line.end(), ',', ' ');
  stringstream ss(line);
  while (!ss.eof()) {
    ss >> name >> val;

    if (name == "lat0") {
      lat0 = stod(val);
    } else if (name == "lon0") {
      lon0 = stod(val);
    } else {
      throw std::runtime_error("AstartPlanner:Unrecognized variable name.");
    }
  }
  home.x = lat0;
  home.y = lon0;

  /**
   *  skip one line of names
   */
  getline(fin, line);

  /**
   * read and set collider position
   */
  float posX, posY, posZ, halfSizeX, halfSizeY, halfSizeZ;
  while (getline(fin, line)) {
    stringstream ss(line);

    getline(ss, val, ',');
    posX = stod(val);

    getline(ss, val, ',');
    posY = stod(val);

    getline(ss, val, ',');
    posZ = stod(val);

    getline(ss, val, ',');
    halfSizeX = stod(val);

    getline(ss, val, ',');
    halfSizeY = stod(val);

    getline(ss, val, ',');
    halfSizeZ = stod(val);

    if (!ss.eof()) {
      throw std::runtime_error("AstarPlanner:Incorrect collider data format!");
    }

    colliders.emplace_back(posX, posY, posZ, halfSizeX, halfSizeY, halfSizeZ);
  }

  return;
}

void test_read_collider(float sol, float truth) {
  assert(sol == truth);
}

int main() {
  vector<Collider> colliders;
  Point home;

  try {
    readColliderMap(colliders, home);
  } catch (std::runtime_error& e) {
    cout << e.what() << '\n';
  }

  // testing data reading
  test_read_collider(home.x, 37.792480);
  test_read_collider(home.y, -122.397450);

  int subset_size = colliders.size();

  vector<Collider> colliders_subset(colliders.begin(),
                                    colliders.begin() + subset_size);

  // test Astar planner
  AstarPlanner test;
  test.set_home(home);

  // Grid grid = test.create_grid_from_map(colliders_subset, 10.0);
  // cout << "Grid size: x=" << grid.size_x << " y=" << grid.size_y << endl;

  test.run_astar_planner(colliders_subset, 10.0);

  return 0;
}