//
// LICENSE:
//
// Copyright (c) 2016 -- 2020 Fabio Pellacini
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#include <yocto/yocto_bvh.h>
#include <yocto/yocto_common.h>
#include <yocto/yocto_commonio.h>
#include <yocto/yocto_geometry.h>
#include <yocto/yocto_image.h>
#include <yocto/yocto_math.h>
#include <yocto/yocto_mesh.h>
#include <yocto/yocto_parallel.h>
#include <yocto/yocto_sceneio.h>
#include <yocto/yocto_shape.h>
#include <yocto_gui/yocto_imgui.h>
#include <yocto_gui/yocto_shade.h>
#include <yocto_gui/yocto_window.h>

#include <unordered_map>
#include <unordered_set>

#include "boolsurf_utils.h"
#include "ext/earcut.hpp"

using namespace yocto;

#include <deque>

#ifdef _WIN32
#undef near
#undef far
#endif

namespace yocto {
void print_obj_camera(sceneio_camera* camera);
};

// Application state
struct app_state {
  // loading parameters
  string filename = "shape.obj";
  string name     = "";

  // options
  shade_params drawgl_prms = {};

  // scene
  generic_shape* ioshape = new generic_shape{};
  shape_bvh      bvh     = {};

  // boolmesh info
  bool_mesh mesh = bool_mesh{};

  vector<mesh_polygon> polygons = {mesh_polygon{}};
  vector<mesh_point>   points   = {};  // Click inserted points

  // rendering state
  shade_scene*  glscene        = new shade_scene{};
  shade_camera* glcamera       = nullptr;
  shade_shape*  mesh_shape     = nullptr;
  shade_shape*  edges_shape    = nullptr;
  shade_shape*  vertices_shape = nullptr;

  shade_material* mesh_material   = nullptr;
  shade_material* edges_material  = nullptr;
  shade_material* points_material = nullptr;
  shade_material* paths_material  = nullptr;
  shade_material* isecs_material  = nullptr;

  vector<shade_material*> cell_materials = {};
  vector<shade_instance*> instances      = {};

  //(marzia) Useful while debugging!
  // unordered_map<int, vector<int>> patch_in        = {};
  // unordered_map<int, vector<int>> patch_out       = {};
  // int                             current_polygon = 1;

  gui_widgets widgets = {};

  ~app_state() {
    if (glscene) delete glscene;
    if (ioshape) delete ioshape;
  }
};

void load_shape(app_state* app, const string& filename) {
  app->filename = filename;
  app->name     = path_filename(app->filename);
  auto error    = ""s;
  if (!load_shape(app->filename, *app->ioshape, error)) {
    printf("Error loading shape: %s\n", error.c_str());
    return;
  }

  // Transformint quad mesh into triangle mesh
  if (app->ioshape->quads.size()) {
    app->ioshape->triangles = quads_to_triangles(app->ioshape->quads);
    app->ioshape->quads     = {};
  }

  app->mesh = init_mesh(app->ioshape);
  app->bvh  = make_triangles_bvh(app->mesh.triangles, app->mesh.positions, {});
}

// TODO(fabio): move this function to math
frame3f camera_frame(float lens, float aspect, float film = 0.036) {
  auto camera_dir  = normalize(vec3f{0, 0.5, 1});
  auto bbox_radius = 2.0f;
  auto camera_dist = bbox_radius * lens / (film / aspect);
  return lookat_frame(camera_dir * camera_dist, {0, 0, 0}, {0, 1, 0});
}

void update_path_shape(shade_shape* shape, const bool_mesh& mesh,
    const geodesic_path& path, float radius, float offset = 0,
    bool thin = false) {
  auto positions = path_positions(
      path, mesh.triangles, mesh.positions, mesh.adjacencies);

  if (thin) {
    set_positions(shape, positions);
    shape->shape->elements = ogl_element_type::line_strip;
    set_instances(shape, {}, {});
    return;
  }

  auto froms = vector<vec3f>();
  auto tos   = vector<vec3f>();
  froms.reserve(positions.size() - 1);
  tos.reserve(positions.size() - 1);
  for (int i = 0; i < positions.size() - 1; i++) {
    auto from = positions[i];
    auto to   = positions[i + 1];
    if (from == to) continue;
    froms.push_back(from);
    tos.push_back(to);
  }

  auto cylinder = make_uvcylinder({4, 1, 1}, {radius, 1});
  for (auto& p : cylinder.positions) {
    p.z = p.z * 0.5 + 0.5;
  }

  set_quads(shape, cylinder.quads);
  set_positions(shape, cylinder.positions);
  set_normals(shape, cylinder.normals);
  set_texcoords(shape, cylinder.texcoords);
  set_instances(shape, froms, tos);
}

void update_path_shape(shade_shape* shape, const bool_mesh& mesh,
    const mesh_path& path, float radius, float offset = 0, bool thin = false) {
  auto positions = vector<vec3f>(path.points.size());
  for (int i = 0; i < positions.size(); i++) {
    positions[i] = eval_position(
        mesh.triangles, mesh.positions, path.points[i]);
  }

  if (offset > 0) {
    // auto mesh_points = convert_mesh_path(mesh.triangles, mesh.adjacencies,
    // path.strip, path.lerps, path.start, path.end);
    auto pos              = positions;
    int  num_subdivisions = 8;
    for (int i = 0; i < num_subdivisions; i++) {
      auto pos = positions;
      for (int i = 0; i < pos.size(); i++) {
        auto a       = (i - 1 + (int)pos.size()) % pos.size();
        auto c       = (i + 1) % pos.size();
        positions[i] = (positions[a] + positions[c]) / 2;
      }
    }
    for (int i = 0; i < pos.size(); i++) {
      auto a  = (i - 1 + (int)pos.size()) % pos.size();
      auto b  = i;
      auto c  = (i + 1) % pos.size();
      auto n0 = eval_normal(mesh.triangles, mesh.normals, path.points[b]);
      auto v  = pos[b] - pos[a];
      auto w  = pos[c] - pos[b];
      positions[i] += offset * normalize(cross(n0, v)) / 2;
      positions[i] += offset * normalize(cross(n0, w)) / 2;
    }
  }

  if (thin) {
    set_positions(shape, positions);
    shape->shape->elements = ogl_element_type::line_strip;
    set_instances(shape, {}, {});
    return;
  }

  auto froms = vector<vec3f>();
  auto tos   = vector<vec3f>();
  froms.reserve(positions.size() - 1);
  tos.reserve(positions.size() - 1);
  for (int i = 0; i < positions.size() - 1; i++) {
    auto from = positions[i];
    auto to   = positions[i + 1];
    if (from == to) continue;
    froms.push_back(from);
    tos.push_back(to);
  }

  auto cylinder = make_uvcylinder({4, 1, 1}, {radius, 1});
  for (auto& p : cylinder.positions) {
    p.z = p.z * 0.5 + 0.5;
  }

  set_quads(shape, cylinder.quads);
  set_positions(shape, cylinder.positions);
  set_normals(shape, cylinder.normals);
  set_texcoords(shape, cylinder.texcoords);
  set_instances(shape, froms, tos);
}

void init_edges_and_vertices_shapes_and_points(
    app_state* app, bool thin = false) {
  auto edges = get_edges(app->mesh.triangles, {});
  auto froms = vector<vec3f>();
  auto tos   = vector<vec3f>();
  froms.reserve(edges.size());
  tos.reserve(edges.size());
  float avg_edge_length = 0;
  for (auto& edge : edges) {
    auto from = app->mesh.positions[edge.x];
    auto to   = app->mesh.positions[edge.y];
    froms.push_back(from);
    tos.push_back(to);
    avg_edge_length += length(from - to);
  }
  avg_edge_length /= edges.size();
  auto cylinder_radius = 0.01f * avg_edge_length;

  if (thin) {
    set_quads(app->edges_shape, {});
    set_positions(app->edges_shape, {{0, 0, -0.5}, {0, 0, 0.5}});
    set_lines(app->edges_shape, {{0, 1}});
    set_normals(app->edges_shape, {});
    set_texcoords(app->edges_shape, {});
  } else {
    auto cylinder = make_uvcylinder({8, 1, 1}, {cylinder_radius, 1});
    for (auto& p : cylinder.positions) {
      p.z = p.z * 0.5 + 0.5;
    }
    set_quads(app->edges_shape, cylinder.quads);
    set_positions(app->edges_shape, cylinder.positions);
    set_normals(app->edges_shape, cylinder.normals);
    set_texcoords(app->edges_shape, cylinder.texcoords);
  }
  set_instances(app->edges_shape, froms, tos);

  auto vertices_radius = 3.0f * cylinder_radius;
  auto vertices        = make_sphere(3, vertices_radius);
  set_quads(app->vertices_shape, vertices.quads);
  set_positions(app->vertices_shape, vertices.positions);
  set_normals(app->vertices_shape, vertices.normals);
  set_texcoords(app->vertices_shape, vertices.texcoords);
  set_instances(app->vertices_shape, app->mesh.positions);
  // app->vertices_shape  = add_shape(glscene, {}, {}, {}, vertices.quads,
  //     vertices.positions, vertices.normals, vertices.texcoords, {});
  // set_instances(vertices_shape, app->mesh.positions);
}

void init_glscene(app_state* app, shade_scene* glscene, const bool_mesh& mesh,
    progress_callback progress_cb) {
  // handle progress
  auto progress = vec2i{0, 4};

  // init scene
  init_scene(glscene, true);

  // camera
  if (progress_cb) progress_cb("convert camera", progress.x++, progress.y);
  app->glcamera = add_camera(glscene, camera_frame(0.050, 16.0f / 9.0f, 0.036),
      0.050, 16.0f / 9.0f, 0.036);
  app->glcamera->focus = length(app->glcamera->frame.o);

  // material
  // TODO(giacomo): Replace this with a proper colormap.
  if (progress_cb) progress_cb("convert material", progress.x++, progress.y);
  app->mesh_material = add_material(
      glscene, {0, 0, 0}, {0.5, 0.5, 0.9}, 1, 0, 0.4);
  app->edges_material = add_material(
      glscene, {0, 0, 0}, {0.4, 0.4, 1}, 1, 0, 0.4);
  app->points_material = add_material(glscene, {0, 0, 0}, {0, 0, 1}, 1, 0, 0.4);
  app->paths_material  = add_material(glscene, {0, 0, 0}, {1, 1, 1}, 1, 0, 0.4);
  app->isecs_material  = add_material(glscene, {0, 0, 0}, {0, 1, 0}, 1, 0, 0.4);
  auto colors          = vector<vec3f>{
      {0, 0, 0},  // REMEMBER IT
      {1, 0, 0},
      {0, 1, 0},
      {0, 0, 1},
      {0, 1, 1},
      {1, 1, 0},
      {1, 0, 1},
      {0.5, 0, 0},
      {0, 0.5, 0},
      {0, 0, 0.5},
      {0, 0.5, 0.5},
      {0.5, 0.5, 0},
      {0.5, 0, 0.5},
  };
  app->cell_materials.resize(colors.size());
  for (int i = 0; i < colors.size(); i++) {
    app->cell_materials[i] = add_material(
        glscene, {0, 0, 0}, colors[i], 1, 0, 0.4);
  }

  // shapes
  if (progress_cb) progress_cb("convert shape", progress.x++, progress.y);
  app->mesh_shape = add_shape(glscene, {}, {}, app->mesh.triangles, {},
      app->mesh.positions, app->mesh.normals, {}, {}, true);

  if (!is_initialized(get_normals(app->mesh_shape))) {
    app->drawgl_prms.faceted = true;
  }
  set_instances(app->mesh_shape, {}, {});

  app->edges_shape    = add_shape(glscene);
  app->vertices_shape = add_shape(glscene);
  add_instance(glscene, identity3x4f, app->mesh_shape, app->mesh_material);
  add_instance(
      glscene, identity3x4f, app->edges_shape, app->edges_material, true);
  add_instance(
      glscene, identity3x4f, app->vertices_shape, app->points_material, true);
  init_edges_and_vertices_shapes_and_points(app);
}

// draw with shading
void draw_widgets(app_state* app, const gui_input& input) {
  auto widgets = &app->widgets;
  begin_imgui(widgets, "boolsurf", {0, 0}, {320, 720});

  if (begin_header(widgets, "view")) {
    auto  glmaterial = app->mesh_material;
    auto& params     = app->drawgl_prms;
    draw_checkbox(widgets, "faceted", params.faceted);
    // continue_line(widgets);
    draw_checkbox(widgets, "lines", app->glscene->instances[1]->hidden, true);
    // continue_line(widgets);
    draw_checkbox(widgets, "points", app->glscene->instances[2]->hidden, true);
    draw_coloredit(widgets, "color", glmaterial->color);
    draw_slider(widgets, "resolution", params.resolution, 0, 4096);
    draw_combobox(
        widgets, "lighting", (int&)params.lighting, shade_lighting_names);
    draw_checkbox(widgets, "wireframe", params.wireframe);
    continue_line(widgets);
    draw_checkbox(widgets, "double sided", params.double_sided);
    // draw_slider(widgets, "exposure", params.exposure, -10, 10);
    // draw_slider(widgets, "gamma", params.gamma, 0.1f, 4);
    // draw_slider(widgets, "near", params.near, 0.01f, 1.0f);
    // draw_slider(widgets, "far", params.far, 1000.0f, 10000.0f);
    end_header(widgets);
  }
  if (begin_header(widgets, "inspect")) {
    draw_label(widgets, "shape", app->name);
    draw_label(widgets, "filename", app->filename);
    auto ioshape = app->ioshape;
    draw_label(widgets, "points", std::to_string(ioshape->points.size()));
    draw_label(widgets, "lines", std::to_string(ioshape->lines.size()));
    draw_label(widgets, "triangles", std::to_string(ioshape->triangles.size()));
    draw_label(widgets, "quads", std::to_string(ioshape->quads.size()));
    draw_label(widgets, "positions", std::to_string(ioshape->positions.size()));
    draw_label(widgets, "normals", std::to_string(ioshape->normals.size()));
    draw_label(widgets, "texcoords", std::to_string(ioshape->texcoords.size()));
    draw_label(widgets, "colors", std::to_string(ioshape->colors.size()));
    draw_label(widgets, "radius", std::to_string(ioshape->radius.size()));
    draw_label(widgets, "quads pos", std::to_string(ioshape->quadspos.size()));
    draw_label(
        widgets, "quads norm", std::to_string(ioshape->quadsnorm.size()));
    draw_label(widgets, "quads texcoord",
        std::to_string(ioshape->quadstexcoord.size()));
    end_header(widgets);
  }

  end_imgui(widgets);
}

// draw with shape
void draw_scene(const app_state* app, const gui_input& input) {
  draw_scene(app->glscene, app->glcamera, input.framebuffer_viewport,
      app->drawgl_prms);
}

void update_camera(app_state* app, const gui_input& input) {
  if (is_active(&app->widgets)) return;

  // handle mouse and keyboard for navigation
  if ((input.mouse_left || input.mouse_right) && !input.modifier_alt) {
    auto dolly  = 0.0f;
    auto pan    = zero2f;
    auto rotate = zero2f;
    if (input.mouse_left && !input.modifier_shift)
      rotate = (input.mouse_pos - input.mouse_last) / 100.0f;
    if (input.mouse_right)
      dolly = (input.mouse_pos.x - input.mouse_last.x) / 100.0f;
    if (input.mouse_left && input.modifier_shift)
      pan = (input.mouse_pos - input.mouse_last) / 100.0f;
    pan.x    = -pan.x;
    rotate.y = -rotate.y;

    std::tie(app->glcamera->frame, app->glcamera->focus) = camera_turntable(
        app->glcamera->frame, app->glcamera->focus, rotate, dolly, pan);
  }
};

void drop(app_state* app, const gui_input& input) {
  if (input.dropped.size()) {
    app->filename = input.dropped[0];
    clear_scene(app->glscene);
    load_shape(app, app->filename);
    init_glscene(app, app->glscene, app->mesh, {});
    return;
  }
}

shape_intersection intersect_shape(
    const app_state* app, const gui_input& input) {
  auto mouse_uv = vec2f{input.mouse_pos.x / float(input.window_size.x),
      input.mouse_pos.y / float(input.window_size.y)};
  auto ray      = camera_ray(app->glcamera->frame, app->glcamera->lens,
      app->glcamera->aspect, app->glcamera->film, mouse_uv);

  auto isec = intersect_triangles_bvh(
      app->bvh, app->mesh.triangles, app->mesh.positions, ray);

  return isec;
}

void draw_sphere(shade_scene* scene, const bool_mesh& mesh,
    shade_material* material, const vector<vec3f>& pos, float dim) {
  auto sphere = make_sphere(4, dim);

  auto shape = add_shape(scene, {}, {}, {}, sphere.quads, sphere.positions,
      sphere.normals, sphere.texcoords, {});
  set_instances(shape, pos);
  add_instance(scene, identity3x4f, shape, material, false);
}

void draw_mesh_point(shade_scene* scene, const bool_mesh& mesh,
    shade_material* material, const mesh_point& point, float dim) {
  auto pos = eval_position(mesh.triangles, mesh.positions, point);
  draw_sphere(scene, mesh, material, {pos}, dim);
}

geodesic_path compute_path(const mesh_polygon& polygon,
    const vector<mesh_point> points, const bool_mesh& mesh) {
  auto size  = polygon.points.size();
  auto start = polygon.points[size - 2];
  auto end   = polygon.points[size - 1];
  auto path = compute_geodesic_path(mesh, points[start], points[end]);  // check
  return path;
}

shade_instance* draw_path(shade_scene* scene, const bool_mesh& mesh,
    shade_material* material, const geodesic_path& path, float radius) {
  auto shape = add_shape(scene);
  update_path_shape(shape, mesh, path, radius);
  add_instance(scene, identity3x4f, shape, material, false);
  return add_instance(scene, identity3x4f, shape, material, false);
}

void draw_intersections(shade_scene* scene, const bool_mesh& mesh,
    shade_material* material, const vector<int>& isecs) {
  auto pos = vector<vec3f>(isecs.size());
  for (auto i = 0; i < isecs.size(); i++) {
    auto v = mesh.triangles[isecs[i]];
    pos[i] = (mesh.positions[v.x] + mesh.positions[v.y] + mesh.positions[v.z]) /
             3.0f;
  }

  draw_sphere(scene, mesh, material, pos, 0.0015f);
}

void draw_segment(shade_scene* scene, const bool_mesh& mesh,
    shade_material* material, const vec3f& start, const vec3f& end,
    float radius = 0.0006f) {
  auto cylinder = make_uvcylinder({4, 1, 1}, {radius, 1});
  for (auto& p : cylinder.positions) {
    p.z = p.z * 0.5 + 0.5;
  }

  auto shape = add_shape(scene);
  add_instance(scene, identity3x4f, shape, material, false);
  set_quads(shape, cylinder.quads);
  set_positions(shape, cylinder.positions);
  set_normals(shape, cylinder.normals);
  set_texcoords(shape, cylinder.texcoords);
  set_instances(shape, {start}, {end});
}

void draw_mesh_segment(shade_scene* scene, const bool_mesh& mesh,
    shade_material* material, const mesh_segment& segment,
    float radius = 0.0012f) {
  auto start = mesh_point{segment.face, segment.start};
  auto end   = mesh_point{segment.face, segment.end};

  draw_mesh_point(scene, mesh, material, start, radius);
  draw_mesh_point(scene, mesh, material, end, radius);

  auto pos_start = eval_position(mesh.triangles, mesh.positions, start);
  auto pos_end   = eval_position(mesh.triangles, mesh.positions, end);

  draw_segment(scene, mesh, material, pos_start, pos_end, radius / 2);
}

void draw_arrangement(shade_scene* scene, const bool_mesh& mesh,
    const vector<shade_material*>& material, const vector<mesh_point>& points,
    vector<cell_polygon>& cells) {
  for (auto p = 0; p < cells.size(); p++) {
    auto& polygon = cells[p];
    auto  mat     = material[p % material.size()];
    auto  path    = mesh_path{};
    for (auto n = 0; n < polygon.points.size() - 1; n++) {
      auto& start = points[polygon.points[n]];
      auto& end   = points[polygon.points[n + 1]];

      auto geo_path = compute_geodesic_path(mesh, start, end);
      append(path.points,
          convert_mesh_path(mesh.triangles, mesh.adjacencies, geo_path.strip,
              geo_path.lerps, geo_path.start, geo_path.end)
              .points);

      // auto segments = mesh_segments(mesh.triangles, geo_path.strip,
      //    geo_path.lerps, geo_path.start, geo_path.end);
      // update_mesh_polygon(polygon, segments);
    }
    auto shape = add_shape(scene);
    // TODO: Make this proportional to avg_edge_length
    float offset = 0.002f;
    update_path_shape(shape, mesh, path, 0.0010f, offset);
    add_instance(scene, identity3x4f, shape, mat, false);
  }
}

void mouse_input(app_state* app, const gui_input& input) {
  if (is_active(&app->widgets)) return;

  if (input.modifier_alt &&
      input.mouse_left.state == gui_button::state::releasing) {
    auto isec = intersect_shape(app, input);
    if (isec.hit) {
      if (app->polygons.size() == 1) app->polygons.push_back(mesh_polygon{});
      if (is_closed(app->polygons.back()))
        app->polygons.push_back(mesh_polygon{});

      auto& polygon = app->polygons.back();
      auto  point   = mesh_point{isec.element, isec.uv};

      if (polygon.points.size()) {
        auto& last_point = app->points[polygon.points.back()];
        if ((point.face == last_point.face) && (point.uv == last_point.uv))
          return;
      }

      app->points.push_back(point);
      polygon.points.push_back(app->points.size() - 1);

      draw_mesh_point(
          app->glscene, app->mesh, app->paths_material, point, 0.0008f);

      if (polygon.points.size() > 1) {
        auto geo_path = compute_path(polygon, app->points, app->mesh);
        auto instance = draw_path(
            app->glscene, app->mesh, app->paths_material, geo_path, 0.0005f);
        app->instances.push_back(instance);

        auto segments = mesh_segments(app->mesh.triangles, geo_path.strip,
            geo_path.lerps, geo_path.start, geo_path.end);

        update_mesh_polygon(polygon, segments);
      }
    }
  }
}

void set_patch_shape(shade_shape* shape, const bool_mesh& mesh,
    const vector<int>& faces, const float distance) {
  auto positions = vector<vec3f>(faces.size() * 3);
  for (int i = 0; i < faces.size(); i++) {
    auto [a, b, c]       = mesh.triangles[faces[i]];
    positions[3 * i + 0] = mesh.positions[a] + distance * mesh.normals[a];
    positions[3 * i + 1] = mesh.positions[b] + distance * mesh.normals[b];
    positions[3 * i + 2] = mesh.positions[c] + distance * mesh.normals[c];
  }
  set_positions(shape, positions);
  set_instances(shape, {});
  shape->shape->elements = ogl_element_type::triangles;
}

auto add_patch_shape(app_state* app, const vector<int>& faces,
    const vec3f& color, const float distance) {
  auto patch_shape    = add_shape(app->glscene, {}, {}, {}, {}, {}, {}, {}, {});
  auto patch_material = add_material(
      app->glscene, {0, 0, 0}, color, 1, 0, 0.4);  // @Leak
  patch_material->opacity = 0.3;
  add_instance(app->glscene, identity3x4f, patch_shape, patch_material);
  set_patch_shape(patch_shape, app->mesh, faces, distance);
  return patch_shape;
}

void do_the_thing(app_state* app) {
  // Hashgrid from triangle idx to <polygon idx, edge_idx, segment idx,
  // segment start uv, segment end uv> to handle intersections and
  // self-intersections

  auto hashgrid          = unordered_map<int, vector<hashgrid_entry>>();
  auto intersections     = unordered_map<vec2i, vector<intersection>>();
  auto triangle_segments = unordered_map<int, vector<triangle_segment>>{};

  for (auto p = 0; p < app->polygons.size(); p++) {
    auto& polygon = app->polygons[p];
    for (auto s = 0; s < polygon.segments.size(); s++) {
      auto& segment = polygon.segments[s];
      hashgrid[segment.face].push_back({p, s, segment.start, segment.end});
    }
  }

  for (auto& [face, entries] : hashgrid) {
    for (auto i = 0; i < entries.size() - 1; i++) {
      auto& AB = entries[i];
      for (auto j = i + 1; j < entries.size(); j++) {
        auto& CD = entries[j];

        auto l = intersect_segments(AB.start, AB.end, CD.start, CD.end);
        if (l.x <= 0.0f || l.x >= 1.0f || l.y <= 0.0f || l.y >= 1.0f) continue;

        //        C
        //        |
        // A -- point -- B
        //        |
        //        D

        auto uv       = lerp(CD.start, CD.end, l.y);
        auto point    = mesh_point{face, uv};
        auto point_id = (int)app->points.size();
        app->points.push_back(point);

        auto idx = (int)app->mesh.positions.size();
        auto pos = eval_position(
            app->mesh.triangles, app->mesh.positions, point);
        app->mesh.positions.push_back(pos);

        intersections[{AB.polygon, AB.segment}].push_back({idx, l.x});
        intersections[{CD.polygon, CD.segment}].push_back({idx, l.y});
      }
    }
  }

  for (auto pid = 0; pid < app->polygons.size(); pid++) {
    auto& segments = app->polygons[pid].segments;
    auto  first_id = (int)app->mesh.positions.size();
    auto  id       = first_id;
    for (auto s = 0; s < segments.size(); s++) {
      auto& segment = segments[s];
      auto  uv      = segment.start;
      auto  pos     = eval_position(
          app->mesh.triangles, app->mesh.positions, {segment.face, uv});
      app->mesh.positions.push_back(pos);

      if (intersections.find({pid, s}) != intersections.end()) {
        auto& isecs = intersections[{pid, s}];
        sort(isecs.begin(), isecs.end(),
            [](auto& a, auto& b) { return a.lerp < b.lerp; });

        for (auto& [id1, l] : isecs) {
          auto uv1 = lerp(segment.start, segment.end, l);
          triangle_segments[segment.face].push_back({pid, id, id1, uv, uv1});
          uv = uv1;
          id = id1;
        };
      }

      auto id1 = (int)app->mesh.positions.size();
      if (s == segments.size() - 1) id1 = first_id;

      triangle_segments[segment.face].push_back(
          {pid, id, id1, uv, segment.end});
      id = id1;
    }
  }

  //(marzia) collapse the triangle_segments iterations
  // Not now, they're useful while debugging
  auto face_edgemap = unordered_map<vec2i, vec2i>{};
  for (auto& [face, segments] : triangle_segments) {
    auto [a, b, c] = app->mesh.triangles[face];
    auto nodes     = vector<vec2f>{{0, 0}, {1, 0}, {0, 1}};
    auto indices   = vector<int>{a, b, c};

    for (auto s = 0; s < segments.size(); s++) {
      auto& [p, id, id1, uv, uv1] = segments[s];
      if (find_idx(indices, id) == -1) {
        nodes.push_back(uv);
        indices.push_back(id);
      }

      if (find_idx(indices, id1) == -1) {
        nodes.push_back(uv1);
        indices.push_back(id1);
      }

      auto edge          = make_edge_key({id, id1});
      face_edgemap[edge] = {-1, -1};
    }

    auto triangles = triangulate(nodes);

    for (auto i = 0; i < triangles.size(); i++) {
      auto& [x, y, z] = triangles[i];
      auto i0         = indices[x];
      auto i1         = indices[y];
      auto i2         = indices[z];

      auto triangle_idx = app->mesh.triangles.size();
      app->mesh.triangles.push_back({i0, i1, i2});

      update_face_edgemap(face_edgemap, {i0, i1}, triangle_idx);
      update_face_edgemap(face_edgemap, {i1, i2}, triangle_idx);
      update_face_edgemap(face_edgemap, {i2, i0}, triangle_idx);
    }

    app->mesh.triangles[face] = {0, 0, 0};
  }

  for (auto& [face, segments] : triangle_segments) {
    for (auto i = 0; i < segments.size(); i++) {
      auto& [p, id, id1, uv, uv1] = segments[i];
      auto edge                   = vec2i{id, id1};
      auto edge_key               = make_edge_key(edge);

      auto faces      = face_edgemap[edge_key];
      auto& [a, b, c] = app->mesh.triangles[faces.x];
      if ((edge == vec2i{b, a}) || (edge == vec2i{c, b}) ||
          (edge == vec2i{a, c})) {
        swap(faces.x, faces.y);
      }

      if (faces.x != -1) app->polygons[p].inner_faces.push_back(faces.x);
      if (faces.y != -1) app->polygons[p].outer_faces.push_back(faces.y);
    }
  }

  // Removing face duplicates
  for (auto i = 1; i < app->polygons.size(); i++) {
    auto& inner = app->polygons[i].inner_faces;
    auto& outer = app->polygons[i].outer_faces;

    sort(inner.begin(), inner.end());
    inner.erase(unique(inner.begin(), inner.end()), inner.end());

    sort(outer.begin(), outer.end());
    outer.erase(unique(outer.begin(), outer.end()), outer.end());
  }

  //(marzia) Why do we need this?
  for (auto& ist : app->instances) ist->hidden = true;

  app->mesh.normals = compute_normals(app->mesh.triangles, app->mesh.positions);
  app->mesh.adjacencies = face_adjacencies(app->mesh.triangles);
  set_positions(app->mesh_shape, app->mesh.positions);
  set_triangles(app->mesh_shape, app->mesh.triangles);
  set_normals(app->mesh_shape, app->mesh.normals);
  init_edges_and_vertices_shapes_and_points(app);

  auto tags          = compute_face_tags(app->mesh, app->polygons);
  auto face_polygons = unordered_map<int, vector<int>>();

  auto cells      = vector<vector<int>>();
  auto cell_faces = unordered_map<int, vector<int>>();

  for (auto p = 1; p < app->polygons.size(); p++) {
    auto check = [&](int face, int polygon) {
      return find_in_vec(tags[face], polygon) == -1;
    };

    auto start_out   = app->polygons[p].outer_faces;
    auto visited_out = flood_fill(app->mesh, start_out, -p, check);
    for (auto o : visited_out) face_polygons[o].push_back(-p);

    auto start_in   = app->polygons[p].inner_faces;
    auto visited_in = flood_fill(app->mesh, start_in, p, check);
    for (auto i : visited_in) face_polygons[i].push_back(p);

    // auto color_out = app->cell_materials[(2 * p)]->color;
    // auto color_in  = app->cell_materials[(2 * p) - 1]->color;

    // app->patch_out[p].push_back(app->glscene->instances.size());
    // add_patch_shape(app, visited_out, color_out, 0.0002f * p);

    // app->patch_in[p].push_back(app->glscene->instances.size());
    // add_patch_shape(app, visited_in, color_in, 0.00025f * p);
  }

  // Inverting face_polygons map
  for (auto& [face, polygons] : face_polygons) {
    auto idx = find_idx(cells, polygons);
    if (idx == -1) {
      idx = (int)cells.size();
      cells.push_back(polygons);
    }

    cell_faces[idx].push_back(face);
  }

  for (auto i = 0; i < cells.size(); i++) {
    printf("Cell: %d -> ", i);
    for (auto c : cells[i]) printf("%d ", c);
    printf("\n\t Faces: %d \n", cell_faces[i].size());

    auto color = app->cell_materials[i + 1]->color;
    add_patch_shape(app, cell_faces[i], color, 0.00025f * (i + 1));
  }

  // Previous Implementation
  // auto graph = compute_graph(
  //     app->points.size(), edge_map, counterclockwise);
  // // print_graph(graph);

  // auto edge_info  = compute_edge_info(edge_map, app->polygons);
  // auto components = compute_connected_components(graph);

  // for (auto& component : components) {
  //   auto cells = compute_cells(
  //       component, app->points, app->mesh, app->polygons.size());
  //   // draw_arrangement(app->glscene, app->mesh, app->cell_materials,
  //   //     app->points, arrangement);

  //   print_graph(component);
  //   print_cells(cells);

  //   auto dual_graph = compute_dual_graph(cells, edge_info);
  //   // print_dual_graph(dual_graph);
  //   auto outer_face = compute_outer_face(dual_graph);

  //   visit_dual_graph(dual_graph, cells, outer_face);
  // }

  // Boolean operation example
  // auto ids = vector<int>(arrangement.size(), 0);
  // for (auto i = 0; i < arrangement.size(); i++)
  //   if (!arrangement[i].embedding[1]) ids[i] = 1;

  // polygon_and(arrangement, ids, 0);
  // // polygon_or(arrangement, ids, 2);

  // auto result = vector<cell_polygon>();
  // for (auto i = 0; i < ids.size(); i++) {
  //   if (ids[i]) {
  //     result.push_back(arrangement[i]);
  //     for (auto a : arrangement[i].points) printf("%d ", a);
  //   }
  //   printf("\n");
  // }

  // draw_arrangement(
  //     app->glscene, app->mesh, app->cell_materials, app->points,
  //     result);
}

void key_input(app_state* app, const gui_input& input) {
  for (auto idx = 0; idx < input.key_buttons.size(); idx++) {
    auto button = input.key_buttons[idx];
    if (button.state != gui_button::state::pressing) continue;

    switch (idx) {
      case (int)gui_key('I'): {
        do_the_thing(app);
      } break;
        // case (int)gui_key('L'): {
        //   for (auto& [_, patches] : app->patch_in)
        //     for (auto p : patches)
        //       app->glscene->instances[p]->hidden =
        //           !app->glscene->instances[p]->hidden;
        // } break;
        // case (int)gui_key('R'): {
        //   for (auto& [_, patches] : app->patch_out)
        //     for (auto p : patches)
        //       app->glscene->instances[p]->hidden =
        //           !app->glscene->instances[p]->hidden;
        // } break;
        // case (int)gui_key('N'): {
        //   for (auto& [key, patches] : app->patch_in)
        //     for (auto p : patches)
        //       app->glscene->instances[p]->hidden =
        //           (key == app->current_polygon) ? false : true;

        //   for (auto& [key, patches] : app->patch_out)
        //     for (auto p : patches)
        //       app->glscene->instances[p]->hidden =
        //           (key == app->current_polygon) ? false : true;

        //   app->current_polygon = (app->current_polygon + 1) %
        //                          app->polygons.size();
        // } break;

      case (int)gui_key('C'): {
        auto old_camera = app->glcamera;
        app->points.clear();
        app->polygons.clear();
        app->polygons.push_back(mesh_polygon{});
        load_shape(app, app->filename);
        clear_scene(app->glscene);
        init_glscene(app, app->glscene, app->mesh, {});
        app->glcamera = old_camera;
      } break;

      case (int)gui_key::enter: {
        auto& polygon = app->polygons.back();
        if (polygon.points.size() < 3 || is_closed(polygon)) return;

        auto point = polygon.points.front();
        polygon.points.push_back(point);

        auto geo_path = compute_path(polygon, app->points, app->mesh);
        draw_path(
            app->glscene, app->mesh, app->paths_material, geo_path, 0.0005f);

        auto segments = mesh_segments(app->mesh.triangles, geo_path.strip,
            geo_path.lerps, geo_path.start, geo_path.end);

        update_mesh_polygon(polygon, segments);

        break;
      }
    }
  }
}

void update_app(const gui_input& input, void* data) {
  auto app = (app_state*)data;

  update_camera(app, input);
  mouse_input(app, input);
  key_input(app, input);

  drop(app, input);

  draw_scene(app, input);
  draw_widgets(app, input);
}

int main(int argc, const char* argv[]) {
  auto app         = new app_state{};
  auto filename    = "tests/_data/shapes/bunny.obj"s;
  auto camera_name = ""s;

  // parse command line
  auto cli = make_cli("yboolsurf", "views shapes inteactively");
  add_option(cli, "--camera", camera_name, "Camera name.");
  add_option(
      cli, "--resolution,-r", app->drawgl_prms.resolution, "Image resolution.");
  add_option(cli, "--lighting", app->drawgl_prms.lighting, "Lighting type.",
      shade_lighting_names);
  add_option(cli, "shape", filename, "Shape filename", true);
  parse_cli(cli, argc, argv);

  auto window = new gui_window{};
  init_window(window, {1280 + 320, 720}, "boolsurf", true);
  window->user_data = app;

  app->filename = filename;
  load_shape(app, filename);

  init_glscene(app, app->glscene, app->mesh, {});
  set_ogl_blending(true);
  app->widgets = create_imgui(window);

  run_ui(window, update_app);

  // clear
  clear_scene(app->glscene);

  // done
  return 0;
}
