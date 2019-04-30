#pragma once

#include <vector>
#include <unordered_map>
#include <cmath>
#include "modules/math/V3F.h"

#define EPS 1e-6

using std::unordered_map;
using std::vector;
namespace planning {

typedef V3F Point;
/**
 *
 */
struct Collider {
  Point pos;
  Point halfSize;

  Collider() = default;
  Collider(float x, float y, float z, float hx, float hy, float hz) {
    pos = Point(x, y, z);
    halfSize = Point(hx, hy, hz);
  }
};

/** struct Grid
 *
 */
struct Grid {
  size_t size_x;                     // grid size in north
  size_t size_y;                     // grid size in east
  int x0;                            // grid starting(min) point in north
  int y0;                            // grid starting(min) point in east
  vector<vector<bool>> is_obstacle;  // is_obstacle[i][j] is true
                                     // if x0 + i and y0 + j is obstacle

  Grid(size_t sx, size_t sy, int x_start = 0, int y_start = 0)
      : size_x(sx), size_y(sy), x0(x_start), y0(y_start) {
    is_obstacle = vector<vector<bool>>(sx, vector<bool>(sy, false));
  }
};

/** struct Edge
 * @brief
 */
struct Edge {
  Point start;
  Point next;
  float weight;
  Edge(Point& u, Point& v, float cost) : start(u), next(v), weight(cost) {}
};

// When using std::unordered_map<T>, we need to have std::hash<T> or
// provide a custom hash function in the constructor to unordered_map.
class PointHash {
public:
  std::size_t operator()(const Point& u) const {
    auto hash1 = std::hash<float>{}(u.x);
    auto hash2 = std::hash<float>{}(u.y);

    return hash1 ^ hash2;
  }
};

class Graph {
public:
  Graph() = default;
  Graph(int V);  // Constructor

  void setVertex() {
    V = edges.size();
  }

  int getVertex() const {
    return V;
  }

  void addEdge(const Point& u, const Point& v, const float cost);

  void addEdge(const Edge& e) {
    edges[e.start].emplace_back(e.next, e.weight);
  }

  vector<std::pair<Point, float>> neighbors(const Point& u) {
    return edges[u];
  }

private:
  int V;  // NO. of vertices

  unordered_map<Point, vector<std::pair<Point, float>>, PointHash> edges;
};

}  // namespace planning