#ifndef RASTERIZER_H
#define RASTERIZER_H

class Rasterizer {

  SDLWindow& window;
  std::vector<float> depth_buffer;

  std::vector<float> interpolate(const float i0, const float d0, const float i1, const float d1);
  void render_triangle(const Triangle& triangle, std::vector<Vertex>& vertices,
                       std::vector<Vertex>& normals, const Camera& camera,
                       const Instance& instance, std::vector<Light>& lights);
  Point viewport_to_canvas(const float y, const float x);
  Point project_vertex(const Vertex4& v);
  float compute_illumination(const Vertex& vertex, const Vertex& normal, const Camera& camera,
                             std::vector<Light>& lights);
  std::vector<Triangle> clip(const Camera& camera, const Model& model, const Mat4x4& transform,
                             const std::vector<Vertex>& transformed_vertices);
  std::tuple<std::array<int, 3>, std::array<int, 6>, std::array<int, 3>>
  sort(const Triangle& triangle, const std::vector<Vertex>& vertices, const std::vector<Vertex>& normals);

  public:
    Rasterizer(SDLWindow& window);
    void render_scene(const Camera& camera, const std::vector<Instance>& instances,
                      std::vector<Light>& lights);
};

#endif
