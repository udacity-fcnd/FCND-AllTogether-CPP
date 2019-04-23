#ifndef JC_VORONOI_WRAPPER_
#define JC_VORONOI_WRAPPER_

#include <stdint.h>
#include "third_party/voronoi/jc_voronoi/src/jc_voronoi.h"

#ifdef __cplusplus
extern "C" {
#endif

// I wrapped it in a library because it spams too many warnings
extern int wrap_stbi_write_png(char const* filename, int w, int h, int comp,
                               const void* data, int stride_in_bytes);

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

const jcv_edge* jcv_edge_generator(const int count, const int width,
                                   const int height, int numrelaxations,
                                   jcv_point* points, jcv_rect* rect,
                                   const char* outputfile);

void jcv_image_generator(const int count, const int width, const int height,
                         jcv_point* points, const jcv_diagram* diagram,
                         const jcv_site* sites, const jcv_edge* edge,
                         const char* outputfile);

#ifdef __cplusplus
}
#endif
#endif  // JC_VORONOI_WRAPPER_