#include <vector>
#include "modules/planning/Voronoi.h"
#include "modules/planning/jc_voronoi_wrapper.h"

int main(int argc, const char** argv) {
  // Number of sites to generate
  int count = 200;
  // Image dimension
  int width = 512;
  int height = 512;
  int numrelaxations = 0;

  const char* outputfile = "example.png";

  jcv_point* points = 0;
  jcv_rect* rect = 0;
  points = (jcv_point*)malloc(sizeof(jcv_point) * (size_t)count);
  if (!points) return 0;

  int pointoffset = 10;  // move the points inwards, for aestetic reasons

  srand(0);

  for (int i = 0; i < count; ++i) {
    points[i].x = (float)(pointoffset + rand() % (width - 2 * pointoffset));
    points[i].y = (float)(pointoffset + rand() % (height - 2 * pointoffset));
  }
  // std::vector<VPoint> vpoints(count);

  // int pointoffset = 10;  // move the points inwards, for aestetic reasons

  // for (int i = 0; i < count; ++i) {
  //   vpoints[i].x = (double)(pointoffset + rand() % (width - 2 *
  //   pointoffset)); vpoints[i].y = (double)(pointoffset + rand() % (height - 2
  //   * pointoffset));
  // }

  // convert_vpoint_to_jcvpoint();

  jcv_edge_generator(count, width, height, numrelaxations, points, rect,
                     outputfile);

  return 0;
}
