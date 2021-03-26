#include <boolsurf/boolsurf.h>
#include <boolsurf/boolsurf_io.h>
#include <yocto/yocto_cli.h>
#include <yocto/yocto_sampling.h>
#include <yocto/yocto_sceneio.h>
#include <yocto/yocto_shape.h>
#include <yocto/yocto_trace.h>

#include "serialize/serialize.h"
using namespace yocto;

mesh_point intersect_mesh(const bool_mesh& mesh, const shape_bvh& bvh,
    const scene_camera& camera, const vec2f& uv) {
  auto ray = camera_ray(
      camera.frame, camera.lens, camera.aspect, camera.film, uv);
  auto isec = intersect_triangles_bvh(bvh, mesh.triangles, mesh.positions, ray);
  return {isec.element, isec.uv};
}

bool_state make_test_state(const bool_test& test, const bool_mesh& mesh,
    const shape_bvh& bvh, const scene_camera& camera, float svg_size) {
  auto state = bool_state{};

  auto polygons = vector<vector<vec2f>>{};
  for (auto& test_polygon : test.polygons) {
    auto& polygon = polygons.emplace_back();

    auto area = 0.0f;
    for (int p = 0; p < test_polygon.size(); p++) {
      auto point_idx = test_polygon[p];
      auto next_idx  = test_polygon[(p + 1) % test_polygon.size()];

      auto& point = test.points_in_screenspace[point_idx];
      auto& next  = test.points_in_screenspace[next_idx];
      area += cross(next, point);

      polygon.push_back(point);
    }

    if (area < 0) std::reverse(polygon.begin(), polygon.end());
  }

  auto bbox = bbox2f{};
  for (auto& polygon : polygons) {
    for (auto& p : polygon) {
      bbox = merge(bbox, p);
    }
  }

  for (auto& polygon : polygons) {
    for (auto& p : polygon) {
      p = (p - center(bbox)) / max(size(bbox));
    }
  }

  for (auto& polygon : polygons) {
    state.polygons.push_back({});
    auto polygon_id = (int)state.polygons.size() - 1;

    for (auto uv : polygon) {
      uv.x /= camera.film;                    // input.window_size.x;
      uv.y /= (camera.film / camera.aspect);  // input.window_size.y;
      uv *= svg_size;
      uv += vec2f{0.5, 0.5};

      auto point = intersect_mesh(mesh, bvh, camera, uv);
      if (point.face == -1) continue;

      // Add point to state.
      state.polygons[polygon_id].points.push_back((int)state.points.size());
      state.points.push_back(point);
    }

    if (state.polygons[polygon_id].points.empty()) {
      continue;
    }

    recompute_polygon_segments(mesh, state, state.polygons[polygon_id]);
  }

  return state;
}

scene_camera make_camera(const bool_mesh& mesh) {
  auto bbox_size = size(mesh.bbox);
  auto z         = zero3f;
  if (bbox_size.x < bbox_size.y && bbox_size.x < bbox_size.z) {
    z = {1, 0, 0};
  }
  if (bbox_size.y < bbox_size.x && bbox_size.y < bbox_size.z) {
    z = {0, 1, 0};
  }
  if (bbox_size.z < bbox_size.x && bbox_size.z < bbox_size.y) {
    z = {0, 0, 1};
  }

  // auto x = vec3f{1, 0, 0};
  auto x = zero3f;
  if (bbox_size.x > bbox_size.y && bbox_size.x > bbox_size.z) {
    x = {1, 0, 0};
  }
  if (bbox_size.y > bbox_size.x && bbox_size.y > bbox_size.z) {
    x = {0, 1, 0};
  }
  if (bbox_size.z > bbox_size.x && bbox_size.z > bbox_size.y) {
    x = {0, 0, 1};
  }

  auto up = -cross(x, z);
  // if (up == z) up = {1, 0, 0};
  auto camera  = scene_camera{};
  camera.frame = lookat_frame(3 * z, zero3f, up);
  return camera;
}

int main(int num_args, const char* args[]) {
  auto test_filename   = ""s;
  auto output_filename = "data/render.png"s;
  auto model_filename  = ""s;
  auto svg_filename    = ""s;
  auto color_shapes    = false;

  // parse command line
  auto cli = make_cli("test", "test boolsurf algorithms");
  add_argument(
      cli, "input", test_filename, "Input test filename (.json).", {}, false);
  add_option(cli, "output", output_filename, "Output image filename (.png).");
  add_option(cli, "model", model_filename, "Input model filename.");
  add_option(cli, "svg", svg_filename, "Input svg filename.");

  add_option(cli, "color-shapes", color_shapes, "Color shapes.");
  parse_cli(cli, num_args, args);

  printf("color-shapes: %d\n", (int)color_shapes);

  auto test = bool_test{};
  if (test_filename.size() && !load_test(test, test_filename)) {
    print_fatal("Error loading test " + test_filename);
  }

  if (model_filename.size()) test.model = model_filename;

  // Init mesh.
  auto error = string{};
  auto mesh  = bool_mesh{};
  {
    if (!load_shape(test.model, mesh, error)) {
      printf("%s\n", error.c_str());
      print_fatal("Error loading model " + test_filename);
    }
    init_mesh(mesh);
    printf("triangles: %d\n", (int)mesh.triangles.size());
    printf("positions: %d\n\n", (int)mesh.positions.size());
  }
  auto bvh = make_triangles_bvh(mesh.triangles, mesh.positions, {});

  // Init bool_state
  auto state  = bool_state{};
  auto camera = scene_camera{};
#if 1
  camera = make_camera(mesh);

  // state  = state_from_test(mesh, test);

  state = make_test_state(test, mesh, bvh, camera, 0.005);

  test.operations.push_back({1, 2, bool_operation::Type::op_difference});
  test.operations.push_back({3, 4, bool_operation::Type::op_difference});
  test.operations.push_back({8, 5, bool_operation::Type::op_difference});
  test.operations.push_back(
      {6, 9, bool_operation::Type::op_symmetrical_difference});

#else

  state  = state_from_test(mesh, test);
  camera = test.camera;
#endif

  {
    auto timer = print_timed("[compute_cells]");
    compute_cells(mesh, state);
    compute_shapes(state);
  }

  save_tree_png(state, "graph.png", "", color_shapes);

  for (auto& operation : test.operations) {
    compute_bool_operation(state, operation);
  }

  auto scene = scene_model{};
  scene.cameras.push_back(camera);

  for (int i = 0; i < state.cells.size(); i++) {
    auto& cell = state.cells[i];

    auto& instance     = scene.instances.emplace_back();
    instance.material  = (int)scene.materials.size();
    auto& material     = scene.materials.emplace_back();
    material.color     = get_cell_color(state, i, color_shapes);
    material.type      = scene_material_type::glossy;
    material.roughness = 0.5;
    instance.shape     = (int)scene.shapes.size();
    auto& shape        = scene.shapes.emplace_back();

    // TODO(giacomo): Too many copies of positions.
    shape.positions = mesh.positions;
    for (auto face : cell.faces) {
      shape.triangles.push_back(mesh.triangles[face]);
    }
    shape.normals = compute_normals(shape);
  }

#if 0
  // auto light_material      = add_material(scene);
  // light_material.emission = {40, 40, 40};

  // auto light_shape       = add_shape(scene);
  // auto quad_shape        = make_rect({1, 1}, {0.2, 0.2});
  // light_shape->quads     = quad_shape.quads;
  // light_shape->positions = quad_shape.positions;
  // light_shape->normals   = quad_shape.normals;
  // light_shape->texcoords = quad_shape.texcoords;

  // for (auto p : {vec3f{-2, 2, 2}, vec3f{2, 2, 1}, vec3f{0, 2, -2}}) {
  //   auto ist      = add_instance(scene);
  //   ist->frame    = lookat_frame(p, {0, 0.5, 0}, {0, 1, 0}, true);
  //   ist->shape    = light_shape;
  //   ist->material = light_material;
  // }

#endif
  auto params    = trace_params{};
  params.sampler = trace_sampler_type::eyelight;
  params.samples = 1;
  auto image     = trace_image(scene, params);
  save_image(output_filename, image, error);
}
