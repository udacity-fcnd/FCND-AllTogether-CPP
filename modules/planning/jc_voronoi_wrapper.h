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

// Search for any of the common characters: \n,;
int is_csv(const char* chars, uint32_t len);

int debug_skip_point(const jcv_point* pt);

int read_input(const char* path, jcv_point** points, uint32_t* length,
               jcv_rect** rect);

void relax_points(const jcv_diagram* diagram, jcv_point* points);

jcv_point remap(const jcv_point* pt, const jcv_point* min, const jcv_point* max,
                const jcv_point* scale);

//
// void convert_vpoint_to_jcvpoint();

void jcv_edge_generator(const int count, const int width, const int height,
                        const int numrelaxations, jcv_point* points,
                        jcv_rect* rect, const char* outputfile);

// void convert_jcvedge_to_vedge();

#ifdef __cplusplus
}
#endif
#endif  // JC_VORONOI_WRAPPER_