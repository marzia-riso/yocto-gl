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

#include "boolsurf_utils.h"

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

  vector<mesh_polygon> cells  = {};
  vector<mesh_point>   points = {};  // Click inserted points
  // vector<isec_polygon> intersections = {};

  // rendering state
  shade_scene*            glscene         = new shade_scene{};
  shade_camera*           glcamera        = nullptr;
  shade_material*         mesh_material   = nullptr;
  shade_material*         edges_material  = nullptr;
  shade_material*         points_material = nullptr;
  shade_material*         paths_material  = nullptr;
  shade_material*         isecs_material  = nullptr;
  vector<shade_material*> cell_materials  = {};

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

  // Move somewhere else (?)
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
  auto mesh_shape = add_shape(glscene, {}, {}, app->mesh.triangles, {},
      app->mesh.positions, app->mesh.normals, {}, {}, true);
  if (!is_initialized(get_normals(mesh_shape))) {
    app->drawgl_prms.faceted = true;
  }
  set_instances(mesh_shape, {}, {});

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
  auto cylinder_radius = 0.05f * avg_edge_length;
  auto cylinder        = make_uvcylinder({8, 1, 1}, {cylinder_radius, 1});
  for (auto& p : cylinder.positions) {
    p.z = p.z * 0.5 + 0.5;
  }
  auto edges_shape = add_shape(glscene, {}, {}, {}, cylinder.quads,
      cylinder.positions, cylinder.normals, cylinder.texcoords, {});
  set_instances(edges_shape, froms, tos);

  auto vertices_radius = 3.0f * cylinder_radius;
  auto vertices        = make_sphere(3, vertices_radius);
  auto vertices_shape  = add_shape(glscene, {}, {}, {}, vertices.quads,
      vertices.positions, vertices.normals, vertices.texcoords, {});
  set_instances(vertices_shape, app->mesh.positions);

  // shapes
  if (progress_cb) progress_cb("convert instance", progress.x++, progress.y);
  add_instance(glscene, identity3x4f, mesh_shape, app->mesh_material);
  add_instance(glscene, identity3x4f, edges_shape, app->edges_material, true);
  add_instance(
      glscene, identity3x4f, vertices_shape, app->points_material, true);

  // done
  if (progress_cb) progress_cb("convert done", progress.x++, progress.y);
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
    load_shape(app, input.dropped[0]);
    clear_scene(app->glscene);
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

void draw_path(shade_scene* scene, const bool_mesh& mesh,
    shade_material* material, const geodesic_path& path, float radius) {
  auto shape = add_shape(scene);
  update_path_shape(shape, mesh, path, radius);
  add_instance(scene, identity3x4f, shape, material, false);
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
    shade_material* material, const mesh_segment& segment) {
  auto start = mesh_point{segment.face, segment.start};
  auto end   = mesh_point{segment.face, segment.end};

  draw_mesh_point(scene, mesh, material, start, 0.0016f);
  draw_mesh_point(scene, mesh, material, end, 0.0016f);

  auto pos_start = eval_position(mesh.triangles, mesh.positions, start);
  auto pos_end   = eval_position(mesh.triangles, mesh.positions, end);

  auto froms = vector<vec3f>();
  froms.push_back(pos_start);
  auto tos = vector<vec3f>();
  tos.push_back(pos_end);

  auto radius   = 0.0010f;
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
  set_instances(shape, froms, tos);
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
      if (!app->cells.size()) app->cells.push_back(mesh_polygon{});
      if (is_closed(app->cells.back())) app->cells.push_back(mesh_polygon{});

      auto& polygon = app->cells.back();
      auto  point   = mesh_point{isec.element, isec.uv};

      app->points.push_back(point);
      polygon.points.push_back(app->points.size() - 1);

      draw_mesh_point(
          app->glscene, app->mesh, app->paths_material, point, 0.0010f);

      if (polygon.points.size() > 1) {
        auto geo_path = compute_path(polygon, app->points, app->mesh);
        draw_path(
            app->glscene, app->mesh, app->paths_material, geo_path, 0.0005f);

        auto segments = mesh_segments(app->mesh.triangles, geo_path.strip,
            geo_path.lerps, geo_path.start, geo_path.end);

        update_mesh_polygon(polygon, segments);
      }
    }
  }
}

void key_input(app_state* app, const gui_input& input) {
  for (auto idx = 0; idx < input.key_buttons.size(); idx++) {
    auto button = input.key_buttons[idx];
    if (button.state != gui_button::state::pressing) continue;

    switch (idx) {
      case (int)gui_key('I'): {
        // Hashgrid from triangle idx to <polygon idx, edge_idx, segment idx,
        // segment start uv, segment end uv> to handle intersections and
        // self-intersections
        // auto hashgrid = unordered_map<int, vector<hashgrid_entry>>();

        // deterministic access for debug
        auto hashgrid = vector<vector<hashgrid_entry>>(
            app->mesh.triangles.size());
        auto edge_map = unordered_map<vec2i, vector<intersection_node>>();
        auto counterclockwise = unordered_map<int, bool>();

        for (auto p = 0; p < app->cells.size(); p++) {
          auto& polygon = app->cells[p];
          for (auto e = 0; e < polygon.edges.size(); e++) {
            auto& edge       = polygon.edges[e];
            auto  end_points = get_edge_points(app->cells, app->points, p, e);
            edge_map[end_points].push_back({end_points.x, {-1, -1}, 0, 0.0f});
            edge_map[end_points].push_back(
                {end_points.y, {-1, -1}, (int)(edge.size() - 1), 1.0f});

            for (auto s = 0; s < edge.size(); s++) {
              auto& segment = edge[s];
              hashgrid[segment.face].push_back(
                  {p, e, s, segment.start, segment.end});
            }
          }
        }

        // for (auto& [face, value] : hashgrid) {
        for (auto f = 0; f < hashgrid.size(); f++) {
          auto& value = hashgrid[f];
          if (value.size() < 2) continue;
          for (auto i = 0; i < value.size() - 1; i++) {
            auto& segmentAB = value[i];
            for (auto j = i + 1; j < value.size(); j++) {
              auto& segmentCD = value[j];

              auto AB = get_edge_points(app->cells, app->points,
                  segmentAB.polygon_id, segmentAB.edge_id);
              auto CD = get_edge_points(app->cells, app->points,
                  segmentCD.polygon_id, segmentCD.edge_id);

              auto l = intersect_segments(segmentAB.start, segmentAB.end,
                  segmentCD.start, segmentCD.end);

              if (l.x <= 0.0f || l.x >= 1.0f || l.y <= 0.0f || l.y >= 1.0f) {
                continue;
              }

              auto uv       = lerp(segmentCD.start, segmentCD.end, l.y);
              auto point    = mesh_point{f, uv};
              auto point_id = (int)app->points.size();

              auto orientation = cross(segmentAB.end - segmentAB.start,
                  segmentCD.end - segmentCD.start);
              assert(orientation != 0);
              auto ccwise = orientation > 0;

              // Flip orientation when self-intersecting.
              if (segmentAB.polygon_id == segmentCD.polygon_id) {
                ccwise = !ccwise;
              }
              counterclockwise[point_id] = ccwise;

              //        C
              //        |
              // A -- point -- B
              //        |
              //        D

              edge_map[AB].push_back({point_id, CD, segmentAB.segment_id, l.x});
              edge_map[CD].push_back({point_id, AB, segmentCD.segment_id, l.y});

              app->points.push_back(point);

              draw_mesh_point(
                  app->glscene, app->mesh, app->isecs_material, point, 0.0020f);
            }
          }
        }

        auto graph = compute_graph(
            app->points.size(), edge_map, counterclockwise);
        print_graph(graph);

        auto faces = compute_graph_faces(graph);
        print_faces(faces);

        // Remove Outer Face (?)
        auto arrangement = compute_arrangement(faces);
        draw_arrangement(app->glscene, app->mesh, app->cell_materials,
            app->points, arrangement);

        auto dual_graph = compute_dual_graph(arrangement);
        print_graph(dual_graph);

        //(marzia) Predefined starting point (should be outer face idx)
        visit_dual_graph(dual_graph, 1);

      } break;
      case (int)gui_key('C'): {
        app->points.clear();
        app->cells.clear();
        load_shape(app, app->filename);
        clear_scene(app->glscene);
        init_glscene(app, app->glscene, app->mesh, {});
      } break;

      case (int)gui_key::enter: {
        auto& polygon = app->cells.back();
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
