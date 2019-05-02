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

class PointwCost : public Point {
public:
  float cost;
  PointwCost() : cost(0.0) {
    x = 0.0;
    y = 0.0;
    z = 0.0;
  }

  PointwCost(float x_, float y_, float z_, float cost_) : cost(cost_) {
    x = x_;
    y = y_;
    z = z_;
  }

  PointwCost(const Point& p, float cost_) : cost(cost_) {
    x = p.x;
    y = p.y;
    z = p.z;
  }

  Point get_pos() const {
    return Point(x, y, z);
  }

  bool operator<(const PointwCost& other) const {
    return cost < other.cost;
  }

  bool operator>(const PointwCost& other) const {
    return cost > other.cost;
  }
};

/** struct Collider
 * @brief obstacle point with geometry
 */
class Collider : public Point {
public:
  float half_size_x;
  float half_size_y;
  float half_size_z;

  Collider(float hx, float hy, float hz)
      : half_size_x(hx), half_size_y(hy), half_size_z(hz) {
    x = 0.0;
    y = 0.0;
    z = 0.0;
  }

  Collider(float x_, float y_, float z_, float hx, float hy, float hz)
      : half_size_x(hx), half_size_y(hy), half_size_z(hz) {
    x = x_;
    y = y_;
    z = z_;
  }

  Collider(const Point& p, float hx, float hy, float hz)
      : half_size_x(hx), half_size_y(hy), half_size_z(hz) {
    x = p.x;
    y = p.y;
    z = p.z;
  }
};

// /** struct Grid
//  * @brief 2D representation of obstacle map
//  */
// struct Grid {
//   int size_x;                        // grid size in north
//   int size_y;                        // grid size in east
//   int x0;                            // grid starting(min) point in north
//   int y0;                            // grid starting(min) point in east
//   vector<vector<bool>> is_obstacle;  // is_obstacle[i][j] is true
//                                      // if x0 + i and y0 + j is obstacle

//   Grid(int sx, int sy, int x_start = 0, int y_start = 0)
//       : size_x(sx), size_y(sy), x0(x_start), y0(y_start) {
//     is_obstacle = vector<vector<bool>>(sx, vector<bool>(sy, false));
//   }
// };

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
    auto hash3 = std::hash<float>{}(u.z);

    return hash1 ^ hash2 ^ hash3;
  }
};

/** class Graph
 * @brief
 */
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

  void addEdge(const Point& u, const Point& v, const float cost) {
    edges[u].emplace_back(v, u.dist(v));
  }

  void addEdge(const Edge& e) {
    edges[e.start].emplace_back(e.next, e.weight);
  }

  vector<PointwCost> neighbors(const Point& u) const {
    return edges.at(u);
  }

private:
  int V;  // NO. of vertices

  unordered_map<Point, vector<PointwCost>, PointHash> edges;
};

}  // namespace planning