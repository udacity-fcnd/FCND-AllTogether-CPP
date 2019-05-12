#pragma once

#include <vector>
#include "modules/planning/BaseQuadPlanner.h"
#include "modules/planning/DataType.h"
#include "modules/planning/jc_voronoi_wrapper.h"

namespace planning {

class AstarPlanner : public BaseQuadPlanner {
public:
  AstarPlanner() {
  }
  ~AstarPlanner() {
  }

  void Init() {
  }

  /** class AstarPlanner
   * @brief set and get home position
   */
  void set_home(const Point& home) {
    this->home = home;
  }

  void set_home(const float lat, const float lon, const float alt = 0.0) {
    home.x = lat;
    home.y = lon;
    home.z = alt;
  }

  const Point& get_home() const {
    return home;
  }

  /**
   *@brief run astar planning
   */
  void run_astar_planner(const vector<Collider>& data, const float altitude);

  // /** class AstarPlanner
  //  *  @brief create grid representation of a 2D configuration space based on
  //  *  given obstacle data, drone altitude and saety distance
  //  *
  //  */
  // Grid create_grid_from_map(const vector<Collider>& data, const float
  // altitude);

  /** class AstarPlanner
   *  @brief create (Voronoi)graph representation of a 2D configuration space
   * based on given obstacle data, drone altitude and saety distance
   *
   */
  vector<Point> read_vpoints_from_map(const vector<Collider>& data,
                                      const float altitude, int& width,
                                      int& height);

  /** class AstarPlanner
   * @brief create Voronoi graph edges given obstacle data
   */
  vector<Edge> Voronoi(const vector<Point>& vpoints, const int width,
                       const int height, Image& image);

  // Astar search over graph
  void astar_graph_search(const Graph& graph, const Point& start,
                          const Point& goal,
                          unordered_map<Point, Point, PointHash>& came_from,
                          unordered_map<Point, float, PointHash>& cost_so_far);

  vector<Point> reconstruct_path(
      const Point& start, const Point& goal,
      unordered_map<Point, Point, PointHash>& came_from);

  bool collinear(const Point& p1, const Point& p2, const Point& p3,
                 float epsilon = 1e-6);

  vector<Point> prune_path(const vector<Point>& path);

  /**
   * @brief heuristic function for cost-to-go estimation
   */
  float heuristic(const Point& pos, const Point& goal) {
    return pos.dist(goal);
  }

private:
  Point home;  // home position (lat, lon, alt) in global coordinate

  float safety_distance{5.0};  // safety distance away from obstacle
};

}  // namespace planning