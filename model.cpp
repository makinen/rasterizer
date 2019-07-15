#include <iostream>
#include <cmath>

#include "model.h"
#include "math.h"

bool Vertex::operator==(const Vertex& rhs) const {
    return (this->x == rhs.x && this->y == rhs.y && this->z == rhs.z);
}

Vertex& Vertex::operator+=(const Vertex& rhs) {
    // actual addition of rhs to *this
    this->x += rhs.x;
    this->y += rhs.y;
    this->z += rhs.z;
    return *this;
}

Vertex& Vertex::operator*=(const float k) {
    // actual addition of rhs to *this
    this->x *= k;
    this->y *= k;
    this->z *= k;
    return *this;
}

Vertex operator+(Vertex lhs, const Vertex& rhs) {
    lhs.x += rhs.x;
    lhs.y += rhs.y;
    lhs.z += rhs.z;
    return lhs;
};

Vertex operator*(Vertex lhs, const float k) {
    lhs.x *= k;
    lhs.y *= k;
    lhs.z *= k;
    return lhs;
};

Instance::Instance(const Model& model, const Vertex& position,
                   float scale, const Mat4x4& orientation)
    :   model(model), position(position), scale(scale),
        orientation(orientation) {

    // calculates the transform matrix
    Mat4x4 translation = make_translation_matrix(this->position.x,
                                                 this->position.y,
                                                 this->position.z);

    Mat4x4 scaling = make_scaling_matrix(this->scale);

    this->transform = multiply_MM4(translation, multiply_MM4(
                                   this->orientation, scaling));
};

Model::Model(const std::vector<Triangle>& triangles, const  std::vector<Vertex>& vertices,
             const  std::vector<Vertex>& normals)
    :   triangles(triangles), vertices(vertices), normals(normals) {

    // Fits a sphere to the vertices. It's used to decide whether the model
    // is visible to the camera

    // Sphere center is the average of all vertices
    float sum[3] = {0, 0, 0};
    for (auto &vertex: vertices) {
      sum[0] += vertex.x;
      sum[1] += vertex.y;
      sum[2] += vertex.z;
    }
    this->bounds_center = Vertex(sum[0] / vertices.size(), sum[1] / vertices.size(),
                                 sum[2] / vertices.size());

    // Sphere radius is the maximum distance from the vertices to the center 

    float radius_sqr = 0;

    for (auto &vertex: vertices) {
      Vertex dist = vertex;
      dist.x -= this->bounds_center.x;
      dist.y -= this->bounds_center.y;
      dist.z -= this->bounds_center.z;

      float temp = dist.x * dist.x + dist.y * dist.y + dist.z * dist.z;
      if (temp > radius_sqr) {
        radius_sqr = temp;
      }
    }

    this->bounds_radius = sqrt(radius_sqr);
};
