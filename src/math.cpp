#include <iostream>
#include <cmath>
#include "model.h"
#include "math.h"

Mat4x4::Mat4x4(const float (&rows)[4][4]) {
    for (size_t i = 0; i < 4; ++i) {
        for (size_t j = 0; j < 4; ++j) {
            this->data[i][j] = rows[i][j];
        }
    }
}

// this makes a rotation matrix around the OY axis
Mat4x4 make_OY_rotation_matrix(float degrees) {

    float cosv = cos(degrees*M_PI/180.0);
    float sinv = sin(degrees*M_PI/180.0);

    return Mat4x4({{cosv, 0, -sinv, 0},
                   {0,    1, 0,     0},
                   {sinv, 0, cosv,  0},
                   {0,    0,    0,  1}});
}

Mat4x4 make_translation_matrix(float x, float y, float z) {
    return Mat4x4({{1, 0, 0, x},
                   {0, 1, 0, y},
                   {0, 0, 1, z},
                   {0, 0, 0, 1}});
}

Mat4x4 make_scaling_matrix(float scale) {
    return Mat4x4({{scale,     0,     0, 0},
                   {    0, scale,     0, 0},
                   {    0,     0, scale, 0},
                   {    0,     0,     0, 1}});
}

Vertex4 multiply_MV(const Mat4x4& m, const Vertex4& v) {

    float result[] = {0, 0, 0, 0};
    float vec[] = {v.x, v.y, v.z, v.w};

    for (size_t i = 0; i < 4; i++) {
        for (size_t j = 0; j < 4; j++) {
            result[i] += m.data[i][j] * vec[j];
        }
    }

  return Vertex4(result[0], result[1], result[2], result[3]);
}

Mat4x4 multiply_MM4(const Mat4x4& m1, const Mat4x4& m2) {

    Mat4x4 result = Mat4x4({{0, 0, 0, 0}, {0, 0, 0, 0},
                                        {0, 0, 0, 0}, {0, 0, 0, 0}});

    for (size_t i = 0; i < 4; i++) {
        for (size_t j = 0; j < 4; j++) {
            for (size_t k = 0; k < 4; k++) {
                result.data[i][j] += m1.data[i][k] * m2.data[k][j];
            }
        }
    }

    return result;
}

Mat4x4 transpose(const Mat4x4& m) {

    Mat4x4 result = Mat4x4({{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0},
                            {0, 0, 0, 0}});

    for (size_t i = 0; i < 4; i++) {
        for (size_t j = 0; j < 4; j++) {
            result.data[i][j] = m.data[j][i];
        }
    }

    return result;
}

float dot_product(const Vertex& v1, const Vertex& v2) {
  return v1.x*v2.x + v1.y*v2.y + v1.z*v1.z;
}

Vertex cross_product(const Vertex& v1, const Vertex& v2) {
   return Vertex(
    v1.y*v2.z - v1.z*v2.y,
    v1.z*v2.x - v1.x*v2.z,
    v1.x*v2.y - v1.y*v2.x);
}

float magnitude(const Vertex& vector) {
  return sqrt(dot_product(vector, vector));
}
