#include <climits>
#include <iostream>
#include <queue>
#include <vector>
#include <list>

#include "modules/planning/AstarPlanner.h"
#include "modules/planning/DataType.h"
#include "modules/planning/jc_voronoi_wrapper.h"

using std::cout;
using std::list;
using std::vector;

namespace planning {
void AstarPlanner::run_astar_planner(const vector<Collider>& data,
                                     const float altitude) {
  cout << "Read obstacle points from map...\n";
  int width = 0, height = 0;
  vector<Point> vpoints = read_vpoints_from_map(data, altitude, width, height);

  // allocate space for image
  const char* outputfile = "Voronoi_Diagram.png";
  Image image(height, width);

  vector<Edge> edges = Voronoi(vpoints, width, height, image);
  if (edges.size() <= 0) {
    std::cout << "AstartPlanner::Invalid Voronoi Edge Generated!\n";
    throw "planning::AstartPlanner: Incorrect Size of Edges in Voronoi Graph";
  }

  Graph graph;
  for (const auto& e : edges) {
    graph.addEdge(e);
  }
  graph.setVertex();

  vector<Point> vertices = graph.getVertex();
  cout << "Graph Vertex NO: " << vertices.size() << endl;

  Point start = vertices[5];
  cout << "Start point: " << start.x << ' ' << start.y << endl;
  for (auto& v : vertices)
    if (v == start) {
      cout << "Found starting point in graph!\n";
      break;
    }
  assert(!graph.neighbors(start).empty());

  Point goal = vertices[4500];
  cout << "Goal point: " << goal.x << ' ' << goal.y << endl;
  for (auto& v : vertices)
    if (v == goal) {
      cout << "Found goal point in graph!\n";
      break;
    }
  assert(!graph.neighbors(goal).empty());

  cout << "Start Astar search...\n";
  unordered_map<Point, Point, PointHash> came_from;
  unordered_map<Point, float, PointHash> cost_so_far;
  astar_graph_search(graph, start, goal, came_from, cost_so_far);

  cout << "Reconstruct path...\n";
  vector<Point> path = reconstruct_path(start, goal, came_from);
  cout << "Path size: " << path.size() << endl;

  cout << "Prune path ...\n";
  vector<Point> pruned_path = prune_path(path);
  cout << "Pruned path size: " << pruned_path.size() << endl;

  unsigned char color_line[] = {255, 0, 0};
  for (int i = 0; i < pruned_path.size() - 1; ++i)
    draw_line((int)pruned_path[i].x, (int)pruned_path[i].y,
              (int)pruned_path[i + 1].x, (int)pruned_path[i + 1].y, image.image,
              width, height, 3, color_line);

  if (outputfile != nullptr) {
    image.flip_image();
    image.write_image(outputfile);
  }
}

// Grid AstarPlanner::create_grid_from_map(const vector<Collider>& data,
//                                         const float altitude) {
//   // find min and max coordinates in north and east directions
//   int grid_x_start = INT_MAX, grid_x_end = INT_MIN;
//   int grid_y_start = INT_MAX, grid_y_end = INT_MIN;  // grid corner positions
//   int pos_x_start, pos_x_end;
//   int pos_y_start, pos_y_end;  // obstacle start and end positions

//   for (const auto& p : data) {
//     pos_x_start = floor(p.x - p.half_size_x);
//     grid_x_start = grid_x_start < pos_x_start ? grid_x_start : pos_x_start;

//     pos_x_end = ceil(p.x + p.half_size_x);
//     grid_x_end = grid_x_end > pos_x_end ? grid_x_end : pos_x_end;

//     pos_y_start = floor(p.y - p.half_size_y);
//     grid_y_start = grid_y_start < pos_y_start ? grid_y_start : pos_y_start;

//     pos_y_end = ceil(p.y + p.half_size_y);
//     grid_y_end = grid_y_end > pos_y_end ? grid_y_end : pos_y_end;
//   }

//   // grid size in north and east directions
//   int size_x = grid_x_end - grid_x_start;
//   int size_y = grid_y_end - grid_y_start;

//   // initialize empty grid
//   Grid grid(size_x, size_y, grid_x_start, grid_y_start);

//   // populate grid with obstacles
//   for (const auto& p : data) {
//     if (p.z + p.half_size_z + safety_distance > altitude) {
//       pos_x_start =
//           floor(p.x - p.half_size_x - safety_distance) - grid_x_start;

//       pos_x_end = ceil(p.x + p.half_size_x + safety_distance) - grid_x_start;

//       pos_y_start =
//           floor(p.y - p.half_size_y - safety_distance) - grid_y_start;

//       pos_y_end = ceil(p.y + p.half_size_y + safety_distance) - grid_y_start;

//       for (int i = max(0, pos_x_start); i <= pos_x_end && i < grid.size_x;
//       ++i)
//         for (int j = max(0, pos_y_start); j <= pos_y_end && j < grid.size_y;
//              ++j)
//           grid.is_obstacle[i][j] = true;
//     }
//   }

//   return grid;
// }

vector<Point> AstarPlanner::read_vpoints_from_map(const vector<Collider>& data,
                                                  const float altitude,
                                                  int& width, int& height) {
  // find min and max coordinates in north and east directions
  int grid_x_start = INT_MAX, grid_x_end = INT_MIN;
  int grid_y_start = INT_MAX, grid_y_end = INT_MIN;  // grid corner positions
  int pos_x_start, pos_x_end;
  int pos_y_start, pos_y_end;  // obstacle start and end positions

  for (const auto& p : data) {
    pos_x_start = floor(p.x - p.half_size_x);
    grid_x_start = grid_x_start < pos_x_start ? grid_x_start : pos_x_start;

    pos_x_end = ceil(p.x + p.half_size_x);
    grid_x_end = grid_x_end > pos_x_end ? grid_x_end : pos_x_end;

    pos_y_start = floor(p.y - p.half_size_y);
    grid_y_start = grid_y_start < pos_y_start ? grid_y_start : pos_y_start;

    pos_y_end = ceil(p.y + p.half_size_y);
    grid_y_end = grid_y_end > pos_y_end ? grid_y_end : pos_y_end;
  }

  // grid size in north and east directions
  width = grid_x_end - grid_x_start;
  height = grid_y_end - grid_y_start;

  // initialize empty grid
  vector<Point> points(data.size());  // obstacle center positions

  // populate obstacle center points (sites in Voronoi diagram)
  for (size_t i = 0; i < data.size(); ++i) {
    points[i].x = data[i].x - grid_x_start;
    points[i].y = data[i].y - grid_y_start;
  }

  return points;
}

vector<Edge> AstarPlanner::Voronoi(const vector<Point>& vpoints,
                                   const int width, const int height,
                                   Image& image) {
  size_t numpoints = vpoints.size();
  jcv_point* jcv_points = new jcv_point[numpoints];

  // convert_vpoint_to_jcvpoint()
  for (size_t i = 0; i < numpoints; ++i) {
    jcv_points[i] = convert_vpoint_to_jcvpoint(vpoints[i]);
    assert(jcv_points[i].x >= 0 && jcv_points[i].x <= width);
    assert(jcv_points[i].y >= 0 && jcv_points[i].y <= height);
  }

  std::cout << "Calling Voronoi and Building Graph...\n";
  vector<Edge> edges =
      jcv_edge_generator((int)numpoints, jcv_points, width, height, image);

  free(jcv_points);

  return edges;
}

void AstarPlanner::astar_graph_search(
    const Graph& graph, const Point& start, const Point& goal,
    unordered_map<Point, Point, PointHash>& came_from,
    unordered_map<Point, float, PointHash>& cost_so_far) {
  priority_queue<PointwCost, vector<PointwCost>, std::greater<PointwCost>>
      frontier;
  frontier.emplace(start, 0.0);

  came_from[start] = start;
  cost_so_far[start] = 0.0;

  while (!frontier.empty()) {
    auto current = frontier.top().get_pos();
    frontier.pop();

    if (current == goal) {
      std::cout << "Arrive at final goal, exit astar search...\n";
      break;
    }

    if (graph.neighbors(current).empty()) {
      std::cout << "No pathexists for vertex: " << current.x << ' ' << current.y
                << '\n';
      throw "No path exists for vertex ...";
    }

    for (auto& next : graph.neighbors(current)) {
      Point next_pos = next.get_pos();

      float new_cost = cost_so_far[current] + next.cost;

      if (!cost_so_far.count(next_pos) || new_cost < cost_so_far[next_pos]) {
        cost_so_far[next_pos] = new_cost;

        float priority = new_cost + heuristic(next_pos, goal);

        frontier.emplace(next_pos, priority);

        came_from[next_pos] = current;
      }
    }
  }
}

vector<Point> AstarPlanner::reconstruct_path(
    const Point& start, const Point& goal,
    unordered_map<Point, Point, PointHash>& came_from) {
  vector<Point> path;

  Point current = goal;  // from goal backwards

  while (current != start) {
    path.push_back(current);
    current = came_from[current];
  }
  path.push_back(start);  // add start at end

  std::reverse(path.begin(), path.end());
  return path;
}

/** check collinearity among 3 points in 2D space
 * @brief Three points lie on the straight line if the area formed by the
 * triangle of these three points is zero. So we will check if the area formed
 * by the triangle is zero or not.
 * Formula for area of triangle is :
 * 0.5 * [x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)]
 */
bool AstarPlanner::collinear(const Point& p1, const Point& p2, const Point& p3,
                             float eps) {
  float area =
      p1.x * (p2.y - p3.y) + p2.x * (p3.y - p1.y) + p3.x * (p1.y - p2.y);
  area *= 0.5;

  if (abs(area) < eps)
    return true;
  else
    return false;
}

vector<Point> AstarPlanner::prune_path(const vector<Point>& path) {
  if (path.size() < 3) return path;

  list<Point> plist(path.begin(), path.end());

  auto iter = next(plist.begin());
  while (iter != prev(plist.end())) {
    Point& p1 = *prev(iter);
    Point& p2 = *iter;
    Point& p3 = *next(iter);

    if (collinear(p1, p2, p3, EPSF)) {
      plist.erase(iter++);
    } else {
      iter++;
    }
  }

  vector<Point> pruned_path(plist.begin(), plist.end());
  return pruned_path;
}

}  // namespace planning