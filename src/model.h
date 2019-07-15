#ifndef MODEL_H
#define MODEL_H

#include <vector>
#include "texture.h"

struct RGBColor {

    RGBColor() {}
    RGBColor(int red, int green, int blue):
        red(red), green(green), blue(blue) {}

    int red;
    int green;
    int blue;
};

struct Point {
    Point(int y, int x): y(y), x(x) {}
    Point(const Point& point): y(point.y), x(point.x) {}

    int x;
    int y;
};

struct Vertex {

    Vertex() {}
    Vertex(float x, float y, float z): x(x), y(y), z(z) {}

    float x;
    float y;
    float z;

    Vertex& operator+=(const Vertex& rhs);
    Vertex& operator*=(const float k);
    bool operator==(const Vertex& rhs) const;
};

Vertex operator+(Vertex lhs, const Vertex& rhs);
Vertex operator*(Vertex lhs, const float k);

struct Vertex4 {
    /* a 3D vertex in homogeneous coordinates */

    Vertex4() {}
    Vertex4(float x, float y, float z, float w):
            x(x), y(y), z(z), w(w) {}

    Vertex4(const Vertex& v, float w):
            x(v.x), y(v.y), z(v.z), w(w) {}

    float x;
    float y;
    float z;
    float w;
};

struct Triangle {

    Triangle(const std::array<int, 3>& indexes, const std::array<int, 3>& normals,
             const RGBColor& color, Texture* texture, const std::array<int, 6>& uvs)
        :    indexes(indexes), normal_indexes(normal_indexes), color(color),
             texture(texture), uvs(uvs) {}

    std::array<int, 3> indexes;
    std::array<int, 3> normal_indexes;
    RGBColor color;
    Texture* texture;
    std::array<int, 6> uvs;

};


// a 4x4 matrix
struct Mat4x4 {
    Mat4x4() {};
    Mat4x4(const float (&rows)[4][4]);
    float data[4][4] = {};
};

struct Plane {
  Plane(const Vertex& normal, float distance): normal(normal), distance(distance) {}

  Vertex normal;
  float distance;
};

struct Camera {
  Camera(const Vertex& position, const Mat4x4& orientation,
         const std::vector<Plane>& clipping_planes): 
         position(position), orientation(orientation), 
         clipping_planes(clipping_planes) {}

  Vertex position;
  Mat4x4 orientation;
  std::vector<Plane> clipping_planes;
};

class Model {

  public:
    Model(const std::vector<Triangle>& triangles, const std::vector<Vertex>& vertices,
          const std::vector<Vertex>& normals);

    // TODO add getters
    std::vector<Triangle> triangles;
    std::vector<Vertex> vertices;
    std::vector<Vertex> normals;
    Vertex bounds_center;
    float bounds_radius;
};

class Instance {

  public:
    Instance(const Model& model, const Vertex& position, float scale,
             const Mat4x4& orientation);

    // TODO add getters
    Model model;
    Vertex position;
    float scale;
    Mat4x4 orientation;
    Mat4x4 transform;
};

struct Light {

  enum class type {
    AMBIENT,
    DIRECTIONAL,
    POINT
  };

  Light(Light::type type, float intensity, const Vertex& vector)
      : type(type), intensity(intensity), vector(vector) {}

  Light::type type;
  float intensity;
  Vertex vector;
};


#endif
