#include <cstdlib>
#include <cstdint>
#include <cstdio>

#define JC_VORONOI_IMPLEMENTATION
// If you wish to use doubles
// #define JCV_REAL_TYPE double
// #define JCV_FABS fabs
// #define JCV_ATAN2 atan2

#include "modules/planning/jc_voronoi_wrapper.h"

namespace planning {

/**
 * @brief helper functions from jc_voronoi/main.c
 */
void plot(int x, int y, unsigned char* image, int width, int height,
          int nchannels, unsigned char* color) {
  if (x < 0 || y < 0 || x > (width - 1) || y > (height - 1)) return;
  int index = y * width * nchannels + x * nchannels;
  for (int i = 0; i < nchannels; ++i) {
    image[index + i] = color[i];
  }
}

// http://members.chello.at/~easyfilter/bresenham.html
void draw_line(int x0, int y0, int x1, int y1, unsigned char* image, int width,
               int height, int nchannels, unsigned char* color) {
  int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
  int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
  int err = dx + dy, e2;  // error value e_xy

  for (;;) {  // loop
    plot(x0, y0, image, width, height, nchannels, color);
    if (x0 == x1 && y0 == y1) break;
    e2 = 2 * err;
    if (e2 >= dy) {
      err += dy;
      x0 += sx;
    }  // e_xy+e_x > 0
    if (e2 <= dx) {
      err += dx;
      y0 += sy;
    }  // e_xy+e_y < 0
  }
}

// http://fgiesen.wordpress.com/2013/02/08/triangle-rasterization-in-practice/
inline int orient2d(const jcv_point* a, const jcv_point* b,
                    const jcv_point* c) {
  return ((int)b->x - (int)a->x) * ((int)c->y - (int)a->y) -
         ((int)b->y - (int)a->y) * ((int)c->x - (int)a->x);
}

inline int min2(int a, int b) {
  return (a < b) ? a : b;
}

inline int max2(int a, int b) {
  return (a > b) ? a : b;
}

inline int min3(int a, int b, int c) {
  return min2(a, min2(b, c));
}
inline int max3(int a, int b, int c) {
  return max2(a, max2(b, c));
}

void draw_triangle(const jcv_point* v0, const jcv_point* v1,
                   const jcv_point* v2, unsigned char* image, int width,
                   int height, int nchannels, unsigned char* color) {
  int area = orient2d(v0, v1, v2);
  if (area == 0) return;

  // Compute triangle bounding box
  int minX = min3((int)v0->x, (int)v1->x, (int)v2->x);
  int minY = min3((int)v0->y, (int)v1->y, (int)v2->y);
  int maxX = max3((int)v0->x, (int)v1->x, (int)v2->x);
  int maxY = max3((int)v0->y, (int)v1->y, (int)v2->y);

  // Clip against screen bounds
  minX = max2(minX, 0);
  minY = max2(minY, 0);
  maxX = min2(maxX, width - 1);
  maxY = min2(maxY, height - 1);

  // Rasterize
  jcv_point p;
  for (p.y = (jcv_real)minY; p.y <= maxY; p.y++) {
    for (p.x = (jcv_real)minX; p.x <= maxX; p.x++) {
      // Determine barycentric coordinates
      int w0 = orient2d(v1, v2, &p);
      int w1 = orient2d(v2, v0, &p);
      int w2 = orient2d(v0, v1, &p);

      // If p is on or inside all edges, render pixel.
      if (w0 >= 0 && w1 >= 0 && w2 >= 0) {
        plot((int)p.x, (int)p.y, image, width, height, nchannels, color);
      }
    }
  }
}

void relax_points(const jcv_diagram* diagram, jcv_point* points) {
  const jcv_site* sites = jcv_diagram_get_sites(diagram);
  for (int i = 0; i < diagram->numsites; ++i) {
    const jcv_site* site = &sites[i];
    jcv_point sum = site->p;
    int count = 1;

    const jcv_graphedge* edge = site->edges;

    while (edge) {
      sum.x += edge->pos[0].x;
      sum.y += edge->pos[0].y;
      ++count;
      edge = edge->next;
    }

    points[site->index].x = sum.x / count;
    points[site->index].y = sum.y / count;
  }
}

// Remaps the point from the input space to image space
inline jcv_point remap(const jcv_point* pt, const jcv_point* min,
                       const jcv_point* max, const jcv_point* scale) {
  jcv_point p;
  p.x = (pt->x - min->x) / (max->x - min->x) * scale->x;
  p.y = (pt->y - min->y) / (max->y - min->y) * scale->y;
  return p;
}

jcv_point convert_vpoint_to_jcvpoint(const Point& p) {
  jcv_point jcv_p;
  jcv_p.x = p.x;
  jcv_p.y = p.y;
  return jcv_p;
}

Point convert_jcvpoint_to_vpoint(const jcv_point& p) {
  Point v_p;
  v_p.x = p.x;
  v_p.y = p.y;
  return v_p;
}

/**
 * @brief main functions to generate vonori diagram, sites and edges
 */
vector<Edge> jcv_edge_generator(const int numpoints, jcv_point* points,
                                const int width, const int height,
                                const char* outputfile) {
  jcv_diagram diagram;
  memset(&diagram, 0, sizeof(jcv_diagram));

  printf("Generating Voronoi Diagram...\n");
  jcv_diagram_generate(numpoints, (const jcv_point*)points, 0, &diagram);

  // generate edges
  printf("Generating Voronoi Edges...\n");
  vector<Edge> edges;

  jcv_point dimensions;
  dimensions.x = (jcv_real)width;
  dimensions.y = (jcv_real)height;
  const jcv_edge* edge = jcv_diagram_get_edges(&diagram);
  while (edge) {
    // Remaps the point from the input space to image space
    jcv_point p0 =
        remap(&edge->pos[0], &diagram.min, &diagram.max, &dimensions);
    jcv_point p1 =
        remap(&edge->pos[1], &diagram.min, &diagram.max, &dimensions);

    // convert back to Vpoint and Vedge
    Point a = convert_jcvpoint_to_vpoint(p0);
    Point b = convert_jcvpoint_to_vpoint(p1);
    edges.emplace_back(a, b, a.distXY(b));

    edge = jcv_diagram_get_next_edge(edge);
  }

  // generate image
  if (outputfile) {
    printf("Generating Images ...\n");
    jcv_image_generator(numpoints, points, width, height, &diagram, outputfile);
  }

  jcv_diagram_free(&diagram);

  return edges;
}

/**
 * @brief generate image
 */
void jcv_image_generator(const int count, const jcv_point* points,
                         const int width, const int height,
                         const jcv_diagram* diagram, const char* outputfile) {
  size_t imagesize = (size_t)(width * height * 3);
  unsigned char* image = (unsigned char*)malloc(imagesize);
  memset(image, 0, imagesize);

  unsigned char color_pt[] = {255, 255, 255};
  unsigned char color_line[] = {220, 220, 220};

  jcv_point dimensions;
  dimensions.x = (jcv_real)width;
  dimensions.y = (jcv_real)height;

  // If you want to draw triangles, or relax the diagram,
  // you can iterate over the sites and get all edges easily
  // generate sites
  const jcv_site* sites = jcv_diagram_get_sites(diagram);
  for (int i = 0; i < diagram->numsites; ++i) {
    const jcv_site* site = &sites[i];

    srand((unsigned int)site->index);  // generating colors for the triangles

    unsigned char color_tri[3];
    unsigned char basecolor = 120;
    color_tri[0] = basecolor + (unsigned char)(rand() % (235 - basecolor));
    color_tri[1] = basecolor + (unsigned char)(rand() % (235 - basecolor));
    color_tri[2] = basecolor + (unsigned char)(rand() % (235 - basecolor));

    jcv_point s = remap(&site->p, &diagram->min, &diagram->max, &dimensions);

    const jcv_graphedge* e = site->edges;
    while (e) {
      jcv_point p0 =
          remap(&e->pos[0], &diagram->min, &diagram->max, &dimensions);
      jcv_point p1 =
          remap(&e->pos[1], &diagram->min, &diagram->max, &dimensions);

      draw_triangle(&s, &p0, &p1, image, width, height, 3, color_tri);
      e = e->next;
    }
  }

  // Plot the sites
  for (int i = 0; i < count; ++i) {
    jcv_point p = remap(&points[i], &diagram->min, &diagram->max, &dimensions);
    plot((int)p.x, (int)p.y, image, width, height, 3, color_pt);
  }

  // flip image
  int stride = width * 3;
  uint8_t* row = (uint8_t*)malloc((size_t)stride);
  for (int y = 0; y < height / 2; ++y) {
    memcpy(row, &image[y * stride], (size_t)stride);
    memcpy(&image[y * stride], &image[(height - 1 - y) * stride],
           (size_t)stride);
    memcpy(&image[(height - 1 - y) * stride], row, (size_t)stride);
  }

  char path[512];
  sprintf(path, "%s", outputfile);
  wrap_stbi_write_png(path, width, height, 3, image, stride);
  printf("wrote %s\n", path);

  free(image);
  free(row);
}
}  // namespace planning