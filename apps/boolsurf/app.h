// TODO(giacomo): review usless includes
#include <yocto/yocto_bvh.h>
#include <yocto/yocto_common.h>
#include <yocto/yocto_commonio.h>
#include <yocto/yocto_geometry.h>
#include <yocto/yocto_image.h>
#include <yocto/yocto_math.h>
#include <yocto/yocto_mesh.h>
#include <yocto/yocto_sceneio.h>
#include <yocto/yocto_shape.h>
#include <yocto_gui/yocto_imgui.h>
#include <yocto_gui/yocto_shade.h>
#include <yocto_gui/yocto_window.h>

#include <unordered_map>
#include <unordered_set>

#include "boolsurf_utils.h"
#include "ext/earcut.hpp"
#include "io.h"
#include "render.h"

using namespace yocto;

// Application state
struct app_state {
  // loading parameters
  string    filename      = "";
  string    test_filename = "";
  bool_test test          = {};

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
  shade_scene*    glscene           = new shade_scene{};
  shade_camera*   glcamera          = nullptr;
  shade_instance* mesh_instance     = nullptr;
  shade_instance* edges_instance    = nullptr;
  shade_instance* vertices_instance = nullptr;

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

void init_edges_and_vertices_shapes_and_points(
    app_state* app, bool thin = true) {
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

  auto edges_shape = app->edges_instance->shape;
  if (thin) {
    set_quads(edges_shape, {});
    set_positions(edges_shape, app->mesh.positions);
    set_lines(edges_shape, edges);
    set_normals(edges_shape, {});
    set_texcoords(edges_shape, {});
    set_instances(edges_shape, {});
    // edges_shape->shape->elements = ogl_element_type::line_strip;
  } else {
    auto cylinder = make_uvcylinder({8, 1, 1}, {cylinder_radius, 1});
    for (auto& p : cylinder.positions) {
      p.z = p.z * 0.5 + 0.5;
    }
    set_quads(edges_shape, cylinder.quads);
    set_positions(edges_shape, cylinder.positions);
    set_normals(edges_shape, cylinder.normals);
    set_texcoords(edges_shape, cylinder.texcoords);
    set_instances(edges_shape, froms, tos);
  }

  auto vertices_shape  = app->vertices_instance->shape;
  auto vertices_radius = 3.0f * cylinder_radius;
  auto vertices        = make_sphere(3, vertices_radius);
  set_quads(vertices_shape, vertices.quads);
  set_positions(vertices_shape, vertices.positions);
  set_normals(vertices_shape, vertices.normals);
  set_texcoords(vertices_shape, vertices.texcoords);
  set_instances(vertices_shape, app->mesh.positions);
  // vertices_shape  = add_shape(glscene, {}, {}, {}, vertices.quads,
  //     vertices.positions, vertices.normals, vertices.texcoords, {});
  // set_instances(vertices_shape, app->mesh.positions);
}

void init_glscene(app_state* app, shade_scene* glscene, const bool_mesh& mesh,
    progress_callback progress_cb) {
  // handle progress
  auto progress = vec2i{0, 4};

  // init scene
  init_scene(glscene, true);

  // TODO(fabio): move this function to math
  auto camera_frame = [](float lens, float aspect, float film = 0.036) {
    auto camera_dir  = normalize(vec3f{0, 0.5, 1});
    auto bbox_radius = 2.0f;
    auto camera_dist = bbox_radius * lens / (film / aspect);
    return lookat_frame(camera_dir * camera_dist, {0, 0, 0}, {0, 1, 0});
  };

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

  set_unlit(app->edges_material, true);
  set_unlit(app->paths_material, true);

  auto colors = vector<vec3f>{
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
  auto mesh_shape = add_shape(glscene, {}, {}, app->mesh.triangles, {},
      app->mesh.positions, app->mesh.normals, {}, {}, true);

  if (!is_initialized(get_normals(mesh_shape))) {
    app->drawgl_prms.faceted = true;
  }
  set_instances(mesh_shape, {}, {});

  auto edges_shape    = add_shape(glscene);
  auto vertices_shape = add_shape(glscene);
  app->mesh_instance  = add_instance(
      glscene, identity3x4f, mesh_shape, app->mesh_material);
  app->edges_instance = add_instance(
      glscene, identity3x4f, edges_shape, app->edges_material, true);
  app->vertices_instance = add_instance(
      glscene, identity3x4f, vertices_shape, app->points_material, true);
  init_edges_and_vertices_shapes_and_points(app);
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

shade_instance* add_patch_shape(app_state* app, const vector<int>& faces,
    const vec3f& color, const float distance) {
  auto patch_shape    = add_shape(app->glscene, {}, {}, {}, {}, {}, {}, {}, {});
  auto patch_material = add_material(
      app->glscene, {0, 0, 0}, color * 0.1, 1, 0, 0.4);  // @Leak
  set_patch_shape(patch_shape, app->mesh, faces, distance);
  return add_instance(app->glscene, identity3x4f, patch_shape, patch_material);
}
