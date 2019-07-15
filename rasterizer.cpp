#include <iostream>
#include <vector>
#include <tuple>
#include <SDL2/SDL.h>
#include "sdl_window.h"
#include "model.h"
#include "math.h"
#include "rasterizer.h"

Rasterizer::Rasterizer(SDLWindow& window): window(window) {
  this->depth_buffer.resize(this->window.getHeight()*this->window.getWidth());
}

void Rasterizer::render_scene(const Camera& camera, const std::vector<Instance>& instances,
                              std::vector<Light>& lights) {

  std::fill(this->depth_buffer.begin(), this->depth_buffer.end(), -1000);

  // TODO do this in the constructor of the Camera
  Vertex pos = camera.position*-1;
  Mat4x4 camera_position = make_translation_matrix(pos.x, pos.y, pos.z);
  Mat4x4 camera_orientation = transpose(camera.orientation);
  Mat4x4 camera_matrix = multiply_MM4(camera_orientation, camera_position);

  for (auto &instance: instances) {
    Mat4x4 transform = multiply_MM4(camera_matrix, instance.transform);

    std::vector<Vertex> transformed_vertices = std::vector<Vertex>();
    std::vector<Vertex> transformed_normals = std::vector<Vertex>();

    // transform the vertices before rendering
    for (auto &vertex : instance.model.vertices) {
      Vertex4 transformedH = multiply_MV(transform, Vertex4(vertex, 1));
      Vertex transformed = Vertex(transformedH.x, transformedH.y, transformedH.z);
      transformed_vertices.push_back(transformed);
    }

    for (auto &normal : instance.model.normals) {
      Vertex4 transformedH = multiply_MV(transform, Vertex4(normal, 1)); 
      Vertex transformed = Vertex(transformedH.x, transformedH.y, transformedH.z);
      transformed_normals.push_back(transformed);
    }

    //std::vector<Triangle> visible = this->clip(camera, instance.model, transform);
    /*
       std::cout << visible.size() << std::endl;
       std::cout << "visible: " << visible.size()<< std::endl;
       std::cout << "=================================================" << std::endl;
     */
    //if (visible.size() == 0) {
      //continue;
    //}

    //for (auto &triangle : visible) {
    for (auto &triangle : instance.model.triangles) {
      this->render_triangle(triangle, transformed_vertices, transformed_normals, camera, instance, lights);
    }
  }
}

// Interpolates the pixels on a line between the i0 and i1
std::vector<float> Rasterizer::interpolate(const float i0, const float d0, const float i1, const float d1) {

  std::vector<float> values = std::vector<float>();

  if (i0 == i1) {
    values.push_back(d0);
  } else {

    int independent_count = i1-i0;

    float a = (float)(d1 - d0) / (i1 - i0);
    float d = d0;

    for (int i = 0; i <= independent_count; i++) {
      values.push_back(d);
      d = d + a;
    }
  }
  return values;
}

// sort vertices of a triangle so that v0.y <= v1.y <= v2.y
std::tuple<std::array<int, 3>, std::array<int, 6>, std::array<int, 3>>
Rasterizer::sort(const Triangle& triangle, const std::vector<Vertex>& vertices,
                 const std::vector<Vertex>& normals) {

  std::array<int, 3> indexes = triangle.indexes;
  std::array<int, 6> uvs = triangle.uvs;
  std::array<int, 3> normal_indexes = triangle.normal_indexes;

  Point P0 = project_vertex(Vertex4(vertices[indexes[0]], 1));
  Point P1 = project_vertex(Vertex4(vertices[indexes[1]], 1));
  Point P2 = project_vertex(Vertex4(vertices[indexes[2]], 1));

  if (P1.y < P0.y) {
    int tmp = indexes[0];
    indexes[0] = indexes[1];
    indexes[1] = tmp;
    int tmp2[] = {uvs[0], uvs[1]};
    uvs[0] = uvs[2];
    uvs[1] = uvs[3];
    uvs[2] = tmp2[0];
    uvs[3] = tmp2[1];
    normal_indexes = {normal_indexes[1], normal_indexes[0], normal_indexes[2]};
  };

  P0 = project_vertex(Vertex4(vertices[indexes[0]], 1));
  P1 = project_vertex(Vertex4(vertices[indexes[1]], 1));

  if (P2.y < P0.y) {
    int tmp = indexes[0];
    indexes[0] = indexes[2];
    indexes[2] = tmp;
    int tmp2[] = {uvs[0], uvs[1]};
    uvs[0] = uvs[4];
    uvs[1] = uvs[5];
    uvs[4] = tmp2[0];
    uvs[5] = tmp2[1];

    normal_indexes = {normal_indexes[2], normal_indexes[1], normal_indexes[0]};
  }

  if (P2.y < P1.y) {
    int tmp = indexes[1];
    indexes[1] = indexes[2];
    indexes[2] = tmp;
    int tmp2[] = {uvs[2], uvs[3]};
    uvs[2] = uvs[4];
    uvs[3] = uvs[5];
    uvs[4] = tmp2[0];
    uvs[5] = tmp2[1];
    normal_indexes = {normal_indexes[0], normal_indexes[2], normal_indexes[1]};
  }

  return std::make_tuple(indexes, uvs, normal_indexes);
}

// Renders the transformed vertices of the given triangle
void Rasterizer::render_triangle(const Triangle& triangle, std::vector<Vertex>& vertices,
                                 std::vector<Vertex>& normals, const Camera& camera,
                                 const Instance& instance, std::vector<Light>& lights) {
 
  auto sorted = this->sort(triangle, vertices, vertices);

  std::array<int, 3> indexes = std::get<0>(sorted);
  std::array<int, 6> uvs = std::get<1>(sorted);
  std::array<int, 3> normal_indexes = std::get<2>(sorted);

  // z values are be inverted because the interpolate function expects a linear function
  // and will give wrong values for a non-linear function
  float P0_z = 1.0 / vertices[indexes[0]].z;
  Point P0 = project_vertex(Vertex4(vertices[indexes[0]], 1));

  float P1_z = 1.0 / vertices[indexes[1]].z;
  Point P1 = project_vertex(Vertex4(vertices[indexes[1]], 1));

  float P2_z = 1.0 / vertices[indexes[2]].z;
  Point P2 = project_vertex(Vertex4(vertices[indexes[2]], 1));

  float intensity = 1;

  // =================================================================
  // # Compute the x coordinates of the edges of triangle
  // =================================================================
  std::vector<float> x01 = this->interpolate(P0.y, P0.x, P1.y, P1.x);
  std::vector<float> x12 = this->interpolate(P1.y, P1.x, P2.y, P2.x);
  std::vector<float> x02 = this->interpolate(P0.y, P0.x, P2.y, P2.x);
  std::vector<float> x012 = std::vector<float>();

  // remove the last one which is the same as the first in x12
  x01.pop_back();
  x012.reserve(x01.size() + x02.size());
  x012.insert(x012.end(), x01.begin(), x01.end());
  x012.insert(x012.end(), x12.begin(), x12.end());

  // =================================================================
  // # GOURAUD - computes lightning at the vertices and interpolates
  // # TODO PHONG
  // =================================================================
  float i0 = compute_illumination(vertices[indexes[0]], normals[0], camera, lights);
  float i1 = compute_illumination(vertices[indexes[1]], normals[1], camera, lights);
  float i2 = compute_illumination(vertices[indexes[2]], normals[2], camera, lights);

  std::vector<float> i01 = this->interpolate(P0.y, i0, P1.y, i1);
  std::vector<float> i12 = this->interpolate(P1.y, i1, P2.y, i2);
  std::vector<float> i02 = this->interpolate(P0.y, i0, P2.y, i2);

  i01.pop_back();
  std::vector<float> i012 = std::vector<float>();
  i012.reserve(i01.size() + i02.size());
  i012.insert(i012.end(), i01.begin(), i01.end());
  i012.insert(i012.end(), i12.begin(), i12.end());

  // =================================================================
  // # Z-buffer
  // =================================================================
  std::vector<float> iz01 = this->interpolate(P0.y, P0_z, P1.y, P1_z);
  std::vector<float> iz12 = this->interpolate(P1.y, P1_z, P2.y, P2_z);
  std::vector<float> iz02 = this->interpolate(P0.y, P0_z, P2.y, P2_z);
  std::vector<float> iz012 = std::vector<float>();
  iz01.pop_back();
  iz012.reserve(iz01.size() + iz02.size());
  iz012.insert(iz012.end(), iz01.begin(), iz01.end());
  iz012.insert(iz012.end(), iz12.begin(), iz12.end());


  // =================================================================
  // # texture coordinates
  // =================================================================
  std::vector<float> uz01 = this->interpolate(P0.y, uvs[0] / vertices[indexes[0]].z,
                                              P1.y, uvs[2] / vertices[indexes[1]].z);

  std::vector<float> uz12 = this->interpolate(P1.y, uvs[2] / vertices[indexes[1]].z,
                                              P2.y, uvs[4] / vertices[indexes[2]].z);

  std::vector<float> uz02 = this->interpolate(P0.y, uvs[0] / vertices[indexes[0]].z,
                                              P2.y, uvs[4] / vertices[indexes[2]].z);

  std::vector<float> uz012 = std::vector<float>();

  uz01.pop_back();
  uz012.reserve(uz01.size() + uz02.size());
  uz012.insert(uz012.end(), uz01.begin(), uz01.end());
  uz012.insert(uz012.end(), uz12.begin(), uz12.end());

  std::vector<float> vz01 = this->interpolate(P0.y, uvs[1] / vertices[indexes[0]].z,
                                              P1.y, uvs[3] / vertices[indexes[1]].z);

  std::vector<float> vz12 = this->interpolate(P1.y, uvs[3] / vertices[indexes[1]].z,
                                              P2.y, uvs[5] / vertices[indexes[2]].z);

  std::vector<float> vz02 = this->interpolate(P0.y, uvs[1] / vertices[indexes[0]].z,
                                              P2.y, uvs[5] / vertices[indexes[2]].z);

  std::vector<float> vz012 = std::vector<float>();

  vz01.pop_back();
  vz012.reserve(vz01.size() + vz02.size());
  vz012.insert(vz012.end(), vz01.begin(), vz01.end());
  vz012.insert(vz012.end(), vz12.begin(), vz12.end());

  // draw horizontal segments
  for (int y = P0.y; y < P2.y; y++) {

    float xl = std::floor(x012.at(y - P0.y));
    float xr = std::floor(x02.at(y - P0.y));

    float zl = iz012.at(y - P0.y);
    float zr = iz02.at(y - P0.y);

    float il = i012.at(y - P0.y);
    float ir = i02.at(y - P0.y);

    float uzl = uz012.at(y - P0.y);
    float uzr = uz02.at(y - P0.y);
    float vzl = vz012.at(y - P0.y);
    float vzr = vz02.at(y - P0.y);

    // start from the other side if x12 is left and x012 is right
    if (x02.at(x02.size()/2) < x012.at(x012.size()/2)) {
      xr = std::floor(x012.at(y - P0.y));
      xl = std::floor(x02.at(y - P0.y));
      zr = iz012.at(y - P0.y);
      zl = iz02.at(y - P0.y);

      ir = i012.at(y - P0.y);
      il = i02.at(y - P0.y);

      uzl = uz02.at(y - P0.y);
      uzr = uz012.at(y - P0.y);
      vzl = vz02.at(y - P0.y);
      vzr = vz012.at(y - P0.y);
    }

    // Z-buffer
    std::vector<float> zscan = this->interpolate(xl, zl, xr, zr);

    // GOURAUD
    std::vector<float> iscan = this->interpolate(xl, il, xr, ir);

    // Textures
    std::vector<float> uzscan = std::vector<float>();
    std::vector<float> vzscan = std::vector<float>();

    if (triangle.texture) {
      uzscan = this->interpolate(xl, uzl, xr, uzr);
      vzscan = this->interpolate(xl, vzl, xr, vzr);
    }

    for (int x = xl; x < xr; x++) {

      // Convert first. (0,0) is at the top left corner but projected coordinates
      // assume it's at the centre of the view
      int xx = this->window.getWidth()/2 + x;
      int yy = this->window.getHeight()/2 - y-1;

      if (this->depth_buffer[yy*this->window.getWidth()+xx] < zscan.at(x-xl)) {

        //} if (shading_model == SM_GOURAUD) {
        intensity = iscan.at(x-xl);
        //}

        RGBColor color = triangle.color;

        if (triangle.texture) {
          float u = uzscan[x - xl] / zscan[x - xl];
          float v = vzscan[x - xl] / zscan[x - xl];

          int iu = std::floor(u*triangle.texture->get_width());
          int iv = std::floor(v*triangle.texture->get_height());

          std::array<int, 3> color = triangle.texture->get_texel(iv, iu);

          int red = std::round(color[0]*intensity);
          int green = std::round(color[1]*intensity);
          int blue = std::round(color[2]*intensity);
          this->window.draw_pixel(yy, xx, red, green, blue);

        } else {

          int red = std::round(triangle.color.red*intensity);
          int green = std::round(triangle.color.green*intensity);
          int blue = std::round(triangle.color.blue*intensity);
          this->window.draw_pixel(yy, xx, red, green, blue);
        }

        this->depth_buffer[yy*this->window.getWidth()+xx] = zscan.at(x-xl);
      }

    }
  }
}

Point Rasterizer::viewport_to_canvas(const float y, const float x) {
  // Converts the viewport coordinates to canvas coordinates
  // which can be drawn on the 2D surface

  int viewport_size = 1;
  int cy = y*this->window.getHeight()/viewport_size;
  int cx = x*this->window.getWidth()/viewport_size;

  return Point(cy, cx);
}

Point Rasterizer::project_vertex(const Vertex4& v) {

  int projection_plane_z = 1;

  float x = v.x * projection_plane_z / v.z;
  float y = v.y * projection_plane_z / v.z;

  return this->viewport_to_canvas(y, x);
}

float Rasterizer::compute_illumination(const Vertex& vertex, const Vertex& normal, const Camera& camera,
                                       std::vector<Light>& lights) {

  float illumination = 0;
  for (auto &light: lights) {

    Vertex light_vector;
    if (light.type == Light::type::AMBIENT) {

      // ambient light has only intensity. It contributes some light to the every point in the scene
      illumination += light.intensity;
      continue;

    } else if (light.type == Light::type::DIRECTIONAL) {

      Vertex4 light_vectorH = multiply_MV(transpose(camera.orientation), Vertex4(light.vector, 1));
      light_vector = Vertex(light_vectorH.x, light_vectorH.y, light_vectorH.z);

    } else if (light.type == Light::type::POINT) {

      Vertex pos = camera.position*-1;
      Mat4x4 translation_matrix = make_translation_matrix(pos.x, pos.y, pos.z);
      Mat4x4 camera_mat = multiply_MM4(transpose(camera.orientation), translation_matrix);

      // transformed light
      Vertex4 lightH = multiply_MV(camera_mat, Vertex4(light.vector, 0));
      Vertex transformed_light = Vertex(lightH.x, lightH.y, lightH.z);

      light_vector = transformed_light + (vertex * -1); // light.vector - vertex
    }

    // calculate reflection

    // diffuse light is the soft light reflected back to the scene
    // equally in every direction
    float cos_alpha = dot_product(light_vector, normal)
      / (magnitude(light_vector) * magnitude(normal));

    if (cos_alpha > 0) {
      illumination += cos_alpha * light.intensity;
    }

    // calculate specular component
    // TODO FIX this
    /*
    Vertex reflected = (normal * (2*dot_product(normal, light_vector))) + light_vector*-1;
    Vertex view = camera.position + (vertex*-1);
    float cos_beta = dot_product(reflected, view);
    
    if (cos_beta > 0) {
      int specular = 50;
      illumination += pow(cos_beta, specular) * light.intensity;
    }
    */
  }

  return illumination;
}

std::vector<Triangle> 
Rasterizer::clip(const Camera& camera, const Model& model, const Mat4x4& transform,
                 const std::vector<Vertex>& transformed_vertices) {

  // this vector contains the visible triangles
  std::vector<Triangle> visible_triangles = std::vector<Triangle>();

  // Transform the bounding sphere
  Vertex4 centerH = Vertex4(model.bounds_center.x, model.bounds_center.y,
                            model.bounds_center.z, 1);
  Vertex4 transformed_centerH = multiply_MV(transform, centerH);
  Vertex transformed_center = Vertex(transformed_centerH.x, transformed_centerH.y,
                                     transformed_centerH.z);

  // if the whole model is behind a clipping plane discard it and return no triangles
  float radius2 = model.bounds_radius*model.bounds_radius;
  for (auto &plane: camera.clipping_planes) {

    // calculate the distance from the clipping plane to the center point
    float distance2 = dot_product(plane.normal, transformed_center) + plane.distance;

    // If the signed distance is negative it's at least partly behind the plane.
    // If the distance is greater than the radius the whole sphere is behind
    // the clipping plane.
    if (distance2 < -radius2) {
      std::cout << "Instance behind a clipping plane. distance2 < -radius2" << std::endl;
      return visible_triangles;
    }
  }


  // clip triangles behind a clipping plane
  // TODO if a triangle is only partly behind a clipping plane create a new vertex
  int i = 0;
  for (auto &triangle: model.triangles) {
    i ++;

    float visible = true;

    for (auto &plane: camera.clipping_planes) {
      int inside = 0;
      if (dot_product(plane.normal, transformed_vertices[triangle.indexes[0]]) + plane.distance > 0) {
        inside++;
      }
      if (dot_product(plane.normal, transformed_vertices[triangle.indexes[1]]) + plane.distance > 0) {
        inside++;
      }
      if (dot_product(plane.normal, transformed_vertices[triangle.indexes[2]]) + plane.distance > 0) {
        inside++;
      }

      // The triangle is fully in front of the plane.
      if (inside != 3) {
        visible = false;
      }

    }
    if (visible) {
      visible_triangles.push_back(triangle);
    }
  }

    /*
     std::cout << "visible triangle count:" << std::endl;
     std::cout << visible_triangles.size() << std::endl;
     std::cout << "=============================================" << std::endl;
   */
  return visible_triangles;
}
