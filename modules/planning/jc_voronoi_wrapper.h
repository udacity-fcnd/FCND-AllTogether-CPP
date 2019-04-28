#ifndef JC_VORONOI_WRAPPER_
#define JC_VORONOI_WRAPPER_

#include <stdint.h>
#include "third_party/voronoi/jc_voronoi/src/jc_voronoi.h"
#include "modules/planning/DataType.h"
#include <vector>

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
/**
 * @brief plotting functions
 */
void plot(int x, int y, unsigned char* image, int width, int height,
          int nchannels, unsigned char* color);

void draw_line(int x0, int y0, int x1, int y1, unsigned char* image, int width,
               int height, int nchannels, unsigned char* color);

void draw_triangle(const jcv_point* v0, const jcv_point* v1,
                   const jcv_point* v2, unsigned char* image, int width,
                   int height, int nchannels, unsigned char* color);

int orient2d(const jcv_point* a, const jcv_point* b, const jcv_point* c);
int min2(int a, int b);
int max2(int a, int b);
int min3(int a, int b, int c);
int max3(int a, int b, int c);

void relax_points(const jcv_diagram* diagram, jcv_point* points);

jcv_point remap(const jcv_point* pt, const jcv_point* min, const jcv_point* max,
                const jcv_point* scale);

jcv_point convert_vpoint_to_jcvpoint(const Point& p);
Point convert_jcvpoint_to_vpoint(const jcv_point& p);

vector<Edge> jcv_edge_generator(const int numpoints, jcv_point* points,
                                const int width, const int height,
                                const char* outputfile);

void jcv_image_generator(const int count, const jcv_point* points,
                         const int width, const int height,
                         const jcv_diagram* diagram, const char* outputfile);

}  // namespace planning
#endif  // JC_VORONOI_WRAPPER_