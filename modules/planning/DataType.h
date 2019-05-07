#pragma once

#include <iomanip>
#include <iostream>
#include <cstring>
#include <vector>
#include <unordered_map>
#include <cmath>
#include "modules/math/V3F.h"

using std::unordered_map;
using std::vector;

#ifdef __cplusplus
extern "C" {
#endif
// wrap it in a library to avoid too many warnings
extern int wrap_stbi_write_png(char const* filename, int w, int h, int comp,
                               const void* data, int stride_in_bytes);
#ifdef __cplusplus
}
#endif

namespace planning {

#define EPSF 1e-3
#define ROW_MAX 99999

class Point : public V3F {
public:
  Point() : V3F() {
  }
  Point(const float x_, const float y_, const float z_) : V3F(x_, y_, z_) {
  }
  inline bool operator==(const V3F& b) const {
    return dist(b) < EPSF;
  }
  inline bool operator!=(const V3F& b) const {
    return dist(b) >= EPSF;
  }
};

class PointwCost : public V3F {
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
class Collider : public V3F {
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

class Image {
public:
  unsigned char* image;
  int height, width;
  int stride;
  Image(int h, int w) : height(h), width(w), stride(width * 3) {
    if (height * width > 0) {
      size_t imagesize = (size_t)(width * height * 3);
      image = new unsigned char[imagesize];
      std::memset(image, 0, imagesize);
    } else {
      image = nullptr;
    }
  }
  ~Image() {
    delete[] image;
  }

  void flip_image() {
    // flip image
    uint8_t* row = new uint8_t[stride];
    for (int y = 0; y < height / 2; ++y) {
      std::memcpy(row, &image[y * stride], (size_t)stride);
      std::memcpy(&image[y * stride], &image[(height - 1 - y) * stride],
                  (size_t)stride);
      std::memcpy(&image[(height - 1 - y) * stride], row, (size_t)stride);
    }
    delete[] row;
  }

  void write_image(const char* outputfile) {
    char path[512];
    sprintf(path, "%s", outputfile);
    wrap_stbi_write_png(path, width, height, 3, image, stride);
    printf("wrote %s\n", path);
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
  Edge(Point& u, Point& v, float cost) : start(u), next(v), weight(cost) {
  }
};

// When using std::unordered_map<T>, we need to have std::hash<T> or
// provide a custom hash function in the constructor to unordered_map.
class PointHash {
public:
  long long operator()(const Point& u) const {
    auto hash1 = std::hash<int>{}(ceil(u.x * 10));
    auto hash2 = std::hash<int>{}(ceil(u.y * 10));

    return hash1 * ROW_MAX + hash2;
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
    for (auto iter = edges.begin(); iter != edges.end(); ++iter)
      vertices.push_back((*iter).first);
  }

  vector<Point> getVertex() const {
    return vertices;
  }

  void addEdge(const Point& u, const Point& v, const float cost) {
    edges[u].emplace_back(v, u.dist(v));
  }

  void addEdge(const Edge& e) {
    edges[e.start].emplace_back(e.next, e.weight);
  }

  vector<PointwCost> neighbors(const Point& u) const {
    if (edges.find(u) == edges.end())
      return {};
    else
      return edges.at(u);
  }

private:
  int V;  // NO. of vertices

  vector<Point> vertices;

  unordered_map<Point, vector<PointwCost>, PointHash> edges;
};

}  // namespace planning