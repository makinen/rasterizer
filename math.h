#ifndef MATH_H
#define MATH_H

struct Mat4x4;
struct Vertex;

// linear algebra functions
Mat4x4 make_OY_rotation_matrix(float degrees);
Mat4x4 make_translation_matrix(float x, float y, float z);
Mat4x4 make_scaling_matrix(float scale);
Vertex4 multiply_MV(const Mat4x4& m, const Vertex4& v);
Mat4x4 multiply_MM4(const Mat4x4& m1, const Mat4x4& m2);
Mat4x4 transpose(const Mat4x4& m);
float dot_product(const Vertex& v1, const Vertex& v2);
Vertex cross_product(const Vertex& v1, const Vertex& v2);
float magnitude(const Vertex& vector);

#endif

