#include <climits>
#include <queue>
#include <string>
#include <iostream>

#include "modules/planning/DataType.h"
#include "modules/planning/jc_voronoi_wrapper.h"
#include "modules/planning/AstarPlanner.h"

using std::vector;

namespace planning {

Grid AstarPlanner::create_grid_from_map(const vector<Collider>& data,
                                        const float altitude) {
  // find min and max coordinates in north and east directions
  int grid_x_start = INT_MAX, grid_x_end = INT_MIN;
  int grid_y_start = INT_MAX, grid_y_end = INT_MIN;  // grid corner positions
  int pos_x_start, pos_x_end;
  int pos_y_start, pos_y_end;  // obstacle start and end positions

  for (const auto& p : data) {
    pos_x_start = floor(p.pos.x - p.halfSize.x);
    grid_x_start = grid_x_start < pos_x_start ? grid_x_start : pos_x_start;

    pos_x_end = ceil(p.pos.x + p.halfSize.x);
    grid_x_end = grid_x_end > pos_x_end ? grid_x_end : pos_x_end;

    pos_y_start = floor(p.pos.y - p.halfSize.y);
    grid_y_start = grid_y_start < pos_y_start ? grid_y_start : pos_y_start;

    pos_y_end = ceil(p.pos.y + p.halfSize.y);
    grid_y_end = grid_y_end > pos_y_end ? grid_y_end : pos_y_end;
  }

  // grid size in north and east directions
  size_t size_x = grid_x_end - grid_x_start;
  size_t size_y = grid_y_end - grid_y_start;

  // initialize empty grid
  Grid grid(size_x, size_y, grid_x_start, grid_y_start);

  // populate grid with obstacles
  for (const auto& p : data) {
    if (p.pos.z + p.halfSize.z + safety_distance > altitude) {
      pos_x_start =
          floor(p.pos.x - p.halfSize.x - safety_distance) - grid_x_start;

      pos_x_end = ceil(p.pos.x + p.halfSize.x + safety_distance) - grid_x_start;

      pos_y_start =
          floor(p.pos.y - p.halfSize.y - safety_distance) - grid_y_start;

      pos_y_end = ceil(p.pos.y + p.halfSize.y + safety_distance) - grid_y_start;

      for (unsigned int i = max(0, pos_x_start);
           i <= (unsigned int)pos_x_end && i < grid.size_x; ++i)
        for (unsigned int j = max(0, pos_y_start);
             j <= (unsigned int)pos_y_end && j < grid.size_y; ++j)
          grid.is_obstacle[i][j] = true;
    }
  }

  return grid;
}

Graph AstarPlanner::create_graph_from_map(const vector<Collider>& data,
                                          const float altitude) {
  // find min and max coordinates in north and east directions
  int grid_x_start = INT_MAX, grid_x_end = INT_MIN;
  int grid_y_start = INT_MAX, grid_y_end = INT_MIN;  // grid corner positions
  int pos_x_start, pos_x_end;
  int pos_y_start, pos_y_end;  // obstacle start and end positions

  for (const auto& p : data) {
    pos_x_start = floor(p.pos.x - p.halfSize.x);
    grid_x_start = grid_x_start < pos_x_start ? grid_x_start : pos_x_start;

    pos_x_end = ceil(p.pos.x + p.halfSize.x);
    grid_x_end = grid_x_end > pos_x_end ? grid_x_end : pos_x_end;

    pos_y_start = floor(p.pos.y - p.halfSize.y);
    grid_y_start = grid_y_start < pos_y_start ? grid_y_start : pos_y_start;

    pos_y_end = ceil(p.pos.y + p.halfSize.y);
    grid_y_end = grid_y_end > pos_y_end ? grid_y_end : pos_y_end;
  }

  // grid size in north and east directions
  size_t size_x = grid_x_end - grid_x_start;
  size_t size_y = grid_y_end - grid_y_start;

  // initialize empty grid
  vector<Point> points(data.size());  // obstacle center positions

  // populate grid with obstacles
  for (unsigned int i = 0; i < data.size(); ++i) {
    points[i].x = data[i].pos.x - grid_x_start;
    points[i].y = data[i].pos.y - grid_y_start;
  }

  vector<Edge> edges = Voronoi(points, size_x, size_y);
  if (edges.size() <= 0) {
    std::cout << "AstartPlanner::Invalid Voronoi Edge Generated!\n";
    throw "planning::AstartPlanner: Incorrect Size of Edges in Voronoi Graph";
  }

  Graph graph;
  for (const auto& e : edges) {
    graph.addEdge(e);
  }
  graph.setVertex();

  return graph;
}

vector<Edge> AstarPlanner::Voronoi(const vector<Point>& vpoints,
                                   const int width, const int height) {
  int numpoints = vpoints.size();
  jcv_point* jcv_points = 0;
  jcv_points = (jcv_point*)malloc(sizeof(jcv_point) * (size_t)numpoints);
  const char* outputfile = "Voronoi_Diagram.png";

  // convert_vpoint_to_jcvpoint()
  for (unsigned int i = 0; i < numpoints; ++i) {
    jcv_points[i] = convert_vpoint_to_jcvpoint(vpoints[i]);
    assert(jcv_points[i].x > 0 && jcv_points[i].x < width);
    assert(jcv_points[i].y > 0 && jcv_points[i].y < height);
  }

  std::cout << "Calling Voronoi and Building Graph...\n";
  vector<Edge> edges =
      jcv_edge_generator(numpoints, jcv_points, width, height, outputfile);

  free(jcv_points);

  return edges;
}

}  // namespace planning