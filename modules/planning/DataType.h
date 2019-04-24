#pragma once

#include <vector>
#include <unordered_map>
#include <cmath>

#define EPS 1e-6

using std::unordered_map;
using std::vector;
namespace planning {

/**
 *
 */
struct Collider {
  double posX = 0.0;       // center north
  double posY = 0.0;       // center east
  double posZ = 0.0;       // center altitude
  double halfSizeX = 0.0;  // half size north
  double halfSizeY = 0.0;  // half size east
  double halfSizeZ = 0.0;  // half size altitude

  Collider() = default;
  Collider(double x, double y, double z, double hx, double hy, double hz)
      : posX(x),
        posY(y),
        posZ(z),
        halfSizeX(hx),
        halfSizeY(hy),
        halfSizeZ(hz) {}
};

/** struct Point
 *
 */
struct Point {
  double lat = 0.0;
  double lon = 0.0;
  double alt = 0.0;

  Point() = default;

  Point(double x, double y, double z = 0.0) : lat(x), lon(y), alt(z) {}

  double dist(Point& other) {
    return sqrt(pow(lat - other.lat, 2) + pow(lon - other.lon, 2) +
                pow(alt - other.lat, 2));
  }

  bool operator==(const Point& other) const {
    return abs(lat - other.lat) < EPS && abs(lon - other.lon) < EPS &&
           abs(alt - other.alt) < EPS;
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
  double weight;
  Edge(Point& u, Point& v, double cost) : start(u), next(v), weight(cost) {}
};

// When using std::unordered_map<T>, we need to have std::hash<T> or
// provide a custom hash function in the constructor to unordered_map.
class PointHash {
public:
  std::size_t operator()(const Point& u) const {
    auto hash1 = std::hash<double>{}(u.lat);
    auto hash2 = std::hash<double>{}(u.lon);

    return hash1 ^ hash2;
  }
};

class Graph {
public:
  Graph() = default;
  Graph(int V);  // Constructor

  void countVertex() {
    V = edges.size();
  }

  void addEdge(Point& u, Point& v, double cost);

  void addEdge(Edge& e);

  vector<std::pair<Point, double>> neighbors(const Point& u) {
    return edges[u];
  }

private:
  int V;  // NO. of vertices

  unordered_map<Point, vector<std::pair<Point, double>>, PointHash> edges;
};

}  // namespace planning