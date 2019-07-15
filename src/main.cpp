#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <cassert>
#include <SDL2/SDL.h>
#include "model.h"
#include "math.h"
#include "sdl_window.h"
#include "rasterizer.h"
#include "texture.h"

#define FPS 60
#define FRAME_DELAY 1000 / FPS

#define PI 3.141592

enum {
  WINDOW_WIDTH = 600,
  WINDOW_HEIGHT = 600
};

RGBColor RED = RGBColor(255, 0, 0);
RGBColor GREEN = RGBColor(0, 255, 0);
RGBColor BLUE = RGBColor(0, 0, 255);
RGBColor YELLOW = RGBColor(255, 255, 0);
RGBColor PURPLE = RGBColor(255, 0, 255);
RGBColor CYAN = RGBColor(100, 255, 0);

Model create_sphere(int recursion_level) {

  float t = (1 + sqrt(5)) / 2;

  // this fixes the position to be on unit sphere
  auto create_vertex = [](float x, float y, float z) {
    double length = sqrt(x * x + y * y + z * z);
    return Vertex(x/length, y/length, z/length);
  };

  std::vector<Vertex> vertices = std::vector<Vertex>();

  // Creates 12 vertices of a icosahedron
  vertices.push_back(create_vertex(-1,  t,  0)); // point 0
  vertices.push_back(create_vertex( 1,  t,  0));
  vertices.push_back(create_vertex(-1, -t,  0));
  vertices.push_back(create_vertex( 1, -t,  0));

  vertices.push_back(create_vertex( 0, -1,  t));
  vertices.push_back(create_vertex( 0,  1,  t));
  vertices.push_back(create_vertex( 0, -1, -t));
  vertices.push_back(create_vertex( 0,  1, -t));

  vertices.push_back(create_vertex( t,  0, -1));
  vertices.push_back(create_vertex( t,  0,  1));
  vertices.push_back(create_vertex(-t,  0, -1));
  vertices.push_back(create_vertex(-t,  0,  1)); // point 11


  //auto create_triangle = [&](int i1, int i2, int i3, int n1, int n2, int n3) {
  auto create_triangle = [&](int i1, int i2, int i3) {
    RGBColor color = RED;
    return Triangle(std::array<int, 3> {i1, i2, i3},
                    std::array<int, 3> {i1, i1, i1},
                    color, nullptr,
                    std::array<int, 6> {0, 0, 0, 0, 0, 0});
  };

  std::vector<Triangle> triangles = std::vector<Triangle>();

  // create the 20 triangles of icosahedron
  // 5 triangles around the point 0
  triangles.push_back(create_triangle(0, 11, 5));
  triangles.push_back(create_triangle(0, 5, 1));
  triangles.push_back(create_triangle(0, 1, 7));
  triangles.push_back(create_triangle(0, 7, 10));
  triangles.push_back(create_triangle(0, 10, 11));

  // 5 adjacent faces
  triangles.push_back(create_triangle(1, 5, 9));
  triangles.push_back(create_triangle(5, 11, 4));
  triangles.push_back(create_triangle(11, 10, 2));
  triangles.push_back(create_triangle(10, 7, 6));
  triangles.push_back(create_triangle(7, 1, 8));

  // 5 faces around point 3
  triangles.push_back(create_triangle(3, 9, 4));
  triangles.push_back(create_triangle(3, 4, 2));
  triangles.push_back(create_triangle(3, 2, 6));
  triangles.push_back(create_triangle(3, 6, 8));
  triangles.push_back(create_triangle(3, 8, 9));

  // 5 adjacent faces
  triangles.push_back(create_triangle(4, 9, 5));
  triangles.push_back(create_triangle(2, 4, 11));
  triangles.push_back(create_triangle(6, 2, 10));
  triangles.push_back(create_triangle(8, 6, 7));
  triangles.push_back(create_triangle(9, 8, 1));

  // creates a new vertex at the middle of the two vertices
  auto get_middle_point = [&](Vertex v0, Vertex v1) {

    Vertex middle = create_vertex((v0.x + v1.x) / 2.0, (v0.y + v1.y) / 2.0,
                                  (v0.z + v1.z) / 2.0);

    // check if the vertex is already in vertices
    ptrdiff_t pos = std::distance(vertices.begin(), std::find(vertices.begin(), vertices.end(), middle));
    if (pos >= vertices.size()) {
      vertices.push_back(middle);
      pos = vertices.size()-1;
    }

    return pos;
  };


  // refine triangles
  for (int i = 0; i < recursion_level; i++) {
    std::vector<Triangle> triangles2 = std::vector<Triangle>();

    for (auto &t: triangles) {
      // replace the triangle by 4 triangles
      int a = get_middle_point(vertices[t.indexes[0]], vertices[t.indexes[1]]);
      int b = get_middle_point(vertices[t.indexes[1]], vertices[t.indexes[2]]);
      int c = get_middle_point(vertices[t.indexes[2]], vertices[t.indexes[0]]);
      
      int v0_idx = std::distance(vertices.begin(), std::find(vertices.begin(), vertices.end(),
                                 vertices[t.indexes[0]]));
      int v1_idx = std::distance(vertices.begin(), std::find(vertices.begin(), vertices.end(),
                                 vertices[t.indexes[1]]));
      int v2_idx = std::distance(vertices.begin(), std::find(vertices.begin(), vertices.end(),
                                 vertices[t.indexes[2]]));

      triangles2.push_back(create_triangle(v0_idx, a, c));
      triangles2.push_back(create_triangle(v1_idx, b, a));
      triangles2.push_back(create_triangle(v2_idx, c, b));
      triangles2.push_back(create_triangle(a, b, c));
    }
    triangles = triangles2;
  }

  std::vector<Vertex> normals = vertices;
  Model sphere_model = Model(triangles, vertices, normals);

  return sphere_model;
}

std::vector<Instance> create_squares(int r, Texture* texture) {

  Vertex v0 = Vertex( 1,  1,  1);
  Vertex v1 = Vertex(-1,  1,  1);
  Vertex v2 = Vertex(-1, -1,  1);
  Vertex v3 = Vertex( 1, -1,  1);
  Vertex v4 = Vertex( 1,  1, -1);
  Vertex v5 = Vertex(-1,  1, -1);
  Vertex v6 = Vertex(-1, -1, -1);
  Vertex v7 = Vertex( 1, -1, -1);

  std::vector<Vertex> vertices = {v0, v1, v2, v3, v4, v5, v6, v7};

  Vertex n0 = Vertex(0, 0, 1);
  Vertex n1 = Vertex(1, 0, 0);
  Vertex n2 = Vertex(0, 0, -1);
  Vertex n3 = Vertex(-1, 0, 0);
  Vertex n4 = Vertex(0, 1, 0);
  Vertex n5 = Vertex(0, -1, 0);

  std::vector<Vertex> normals = {n0, n1, n2, n3, n4, n5};

  Texture* t2 = texture;

  Triangle t[] =  {
    Triangle(std::array<int, 3> {0, 1, 2}, std::array<int, 3> {0, 0, 0}, RED, texture,
             std::array<int, 6>    {0, 1, 1, 1, 1, 0}),
    Triangle(std::array<int, 3> {0, 2, 3}, std::array<int, 3> {0, 0, 0}, RED, texture,
             std::array<int, 6>    {1, 0, 0, 1, 1, 1}),
    Triangle(std::array<int, 3> {4, 0, 3}, std::array<int, 3> {1, 1, 1}, GREEN, texture,
             std::array<int, 6>  {1, 1, 0, 1, 0, 0}),
    Triangle(std::array<int, 3> {4, 3, 7}, std::array<int, 3> {1, 1, 1}, GREEN, texture,
             std::array<int, 6>  {0, 1, 1, 0, 0, 0}),
    Triangle(std::array<int, 3> {5, 4, 7}, std::array<int, 3> {2, 2, 2}, BLUE, texture,
             std::array<int, 6>   {0, 0, 1, 0, 1, 1}),
    Triangle(std::array<int, 3> {5, 7, 6}, std::array<int, 3> {2, 2, 2}, BLUE, texture,
             std::array<int, 6>   {0, 0, 1, 1, 0, 1}),
    Triangle(std::array<int, 3> {1, 5, 6}, std::array<int, 3> {3, 3, 3}, YELLOW, texture,
             std::array<int, 6> {0, 0, 1, 0, 1, 1}),
    Triangle(std::array<int, 3> {1, 6, 2}, std::array<int, 3> {3, 3, 3}, YELLOW, texture,
             std::array<int, 6> {0, 1, 1, 0, 0, 0}),
    Triangle(std::array<int, 3> {1, 0, 5}, std::array<int, 3> {4, 4, 4}, PURPLE, texture,
             std::array<int, 6> {0, 0, 1, 0, 1, 1}),
    Triangle(std::array<int, 3> {5, 0, 4}, std::array<int, 3> {4, 4, 4}, PURPLE, texture,
             std::array<int, 6> {0, 0, 1, 0, 1, 1}),
    Triangle(std::array<int, 3> {2, 6, 7}, std::array<int, 3> {5, 5, 5}, CYAN, texture, 
             std::array<int, 6>   {0, 0, 1, 0, 1, 1}),
    Triangle(std::array<int, 3> {2, 7, 3}, std::array<int, 3> {5, 5, 5}, CYAN, texture, 
             std::array<int, 6>   {0, 0, 1, 0, 1, 1})
    };

  std::vector<Triangle> triangles(std::begin(t), std::end(t));

  Model cube_model = Model(triangles, vertices, normals);

  // Camera is inside of the cube. Move the cube further by increasing the Z coordinate
  Vertex pos = Vertex(0, 0.8, 6);
  Vertex pos2 = Vertex(0, 0, 6.5);

  float scale = 1;

  Mat4x4 identity = Mat4x4({{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0},
      {0, 0, 0, 1}});
  Mat4x4 rotation = make_OY_rotation_matrix(r);

  //Instance cube = Instance(cube_model, pos, scale, identity);

  Instance cube2 = Instance(cube_model, pos2, scale, rotation);


  std::vector<Instance> instances = std::vector<Instance>();

  //instances.push_back(cube);
  instances.push_back(cube2);

  return instances;
}


int main() {

  SDLWindow window = SDLWindow(WINDOW_WIDTH, WINDOW_HEIGHT);
  window.show();

  printf("lets start the loop\n");

  Rasterizer rasterizer = Rasterizer(window);

  // clipping planes
  std::vector<Plane> clipping_planes = std::vector<Plane>();
  //float s2 = sqrt(2);
  //s2 = 3.5;
  //clip4ing_planes.push_back(Plane(Vertex(0, 0, 1), -1)); // Near
  //clipping_planes.push_back(Plane(Vertex(s2, 0, s2), 0)); // Left
  //clipping_planes.push_back(Plane(Vertex(-s2, 0, s2), 0)); // Right
  //clipping_planes.push_back(Plane(Vertex(0, -s2, s2), 0)); // Top
  //clipping_planes.push_back(Plane(Vertex(0, s2, s2), 0)); // Bottom


  std::vector<Light> lights = {Light(Light::type::AMBIENT, 0.4, Vertex(0, 0, 0)),
                               Light(Light::type::DIRECTIONAL, 0.2, Vertex(2, 0, 6)),
                               Light(Light::type::POINT, 0.0, Vertex(-4, 0, 1))};

  Model sphere_model = create_sphere(3);

  float l = 0;
  Camera camera = Camera(Vertex(0, 0, 0), make_OY_rotation_matrix(l), clipping_planes);
  Texture texture = Texture("crate-texture.png");

  Vertex pos = Vertex(1.2, 0, 5.5);
  float scale = 0.8;
  int r = 151;

  Mat4x4 rotation = make_OY_rotation_matrix(r);
  Instance sphere = Instance(sphere_model, pos, scale, rotation);

  while(1) {

    std::vector<Instance> instances = std::vector<Instance>();
    //instances.push_back(sphere);

    std::vector<Instance> squares = create_squares(r, &texture);
    instances.push_back(squares.at(0));

    window.clear();

    // milliseconds since initializing sdl
    Uint32 frame_start = SDL_GetTicks();

    rasterizer.render_scene(camera, instances, lights);

    r+=1;

    window.update();

    int elapsed = SDL_GetTicks() - frame_start;
    if ( FRAME_DELAY > elapsed ) {
      SDL_Delay(FRAME_DELAY - elapsed);
    }

    //SDL_Delay(8000);
    //return 0;

    SDL_Event event;
    if (SDL_PollEvent(&event) && event.type == SDL_QUIT) {
      printf("SDL_QUIT\n");
      break;
    }
    //  break;
  }

  return 0;
}
