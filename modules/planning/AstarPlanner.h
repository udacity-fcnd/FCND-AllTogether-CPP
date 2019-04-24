#pragma once

#include <vector>
#include "modules/planning/BaseQuadPlanner.h"
#include "modules/planning/DataType.h"
#include "modules/planning/jc_voronoi_wrapper.h"

namespace planning {

class AstarPlanner : public BaseQuadPlanner {
public:
  AstarPlanner() {}
  ~AstarPlanner() {}

  /** class AstarPlanner
   * @brief set and get home position
   */
  void set_home(const Point& home) {
    this->home = home;
  }

  void set_home(const double lat, const double lon, const double alt = 0.0) {
    home.lat = lat;
    home.lon = lon;
    home.alt = alt;
  }

  const Point& get_home() const {
    return home;
  }

  /** class AstarPlanner
   *  @brief create grid representation of a 2D configuration space based on
   *  given obstacle data, drone altitude and saety distance
   *
   */
  Grid create_grid_from_map(const vector<Collider>& data,
                            const double altitude);
  /** class AstarPlanner
   *  @brief create (Voronoi)graph representation of a 2D configuration space
   * based on given obstacle data, drone altitude and saety distance
   *
   */
  Graph create_graph_from_map(const vector<Collider>& data,
                              const double altitude);

  /** class AstarPlanner
   * @brief create Voronoi graph edges given obstacle data
   */
  jcv_point convert_vpoint_to_jcvpoint(const Point& p) {
    jcv_point jcv_p;
    jcv_p.x = (float)p.lat;
    jcv_p.y = (float)p.lon;
    return jcv_p;
  }
  Point convert_jcvpoint_to_vpoint(const jcv_point& p) {
    Point v_p;
    v_p.lat = (double)p.x;
    v_p.lon = (double)p.y;
    return v_p;
  }
  Edge convert_jcvedge_to_vedge(const jcv_edge* edges) {
    jcv_point p_start = edges->pos[0];
    jcv_point p_end = edges->pos[1];
    Point a = convert_jcvpoint_to_vpoint(p_start);
    Point b = convert_jcvpoint_to_vpoint(p_end);
    double dist = a.dist(b);
    return Edge(a, b, dist);
  }
  vector<Edge> Voronoi(const vector<Point>& points, const int width,
                       const int height);

  // Astar search
  void Astar_Graph();

private:
  Point home;
  double safety_distance{5.0};
};

}  // namespace planning