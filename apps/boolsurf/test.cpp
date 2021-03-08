#include <yocto/yocto_cli.h>
#include <yocto/yocto_sampling.h>
#include <yocto/yocto_sceneio.h>
#include <yocto/yocto_shape.h>
#include <yocto/yocto_trace.h>

#include "boolsurf.h"
#include "boolsurf_io.h"
using namespace yocto;

mesh_point intersect_mesh(const bool_mesh& mesh, const shape_bvh& bvh,
    const scene_camera& camera, const vec2f& uv) {
  auto ray = camera_ray(
      camera.frame, camera.lens, camera.aspect, camera.film, uv);
  auto isec = intersect_triangles_bvh(bvh, mesh.triangles, mesh.positions, ray);
  return {isec.element, isec.uv};
}

bool_state make_test_state(const bool_mesh& mesh, const shape_bvh& bvh,
    const scene_camera& camera, float svg_size) {
  auto state    = bool_state{};
  auto polygons = vector<vector<vec2f>>{
      // B
      {{344.261, 488.09}, {435.116, 100.957}, {603.936, 129.097},
          {638.062, 169.8}, {647.917, 208.993}, {646.561, 252.953},
          {629.785, 276.726}, {610.792, 303.154}, {583.55, 316.923},
          {609.525, 332.67}, {628.794, 365.505}, {631.995, 400.703},
          {626.094, 452.98}, {601.427, 487.932}, {537.858, 511.936},
          {450.002, 514.445}},

      {{386.142, 290.879}, {433.53, 297.812}, {448.881, 292.352},
          {459.982, 274.828}, {459.747, 246.494}, {451.827, 228.84},
          {434.914, 221.268}, {392.334, 210.984}},
  };

  // ,{{344.261, 488.09}, {435.116, 100.957}, {603.936, 129.097},
  //     {638.062, 169.8}, {647.917, 208.993}, {646.561, 252.953},
  //     {629.785, 276.726}, {610.792, 303.154}, {583.55, 316.923},
  //     {609.525, 332.67}, {628.794, 365.505}, {631.995, 400.703},
  //     {626.094, 452.98}, {601.427, 487.932}, {537.858, 511.936},
  //     {450.002, 514.445}}};

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
      if(state.polygons[polygon_id].points.empty()) {
          continue;
      }
    update_polygon(state, mesh, polygon_id);
  }
  return state;
}

int main(int num_args, const char* args[]) {
  auto test_filename   = "data/tests/test.json"s;
  auto output_filename = "data/render.png"s;
  auto model_filename  = ""s;

  // parse command line
  auto cli = make_cli("test", "test boolsurf algorithms");
  add_argument(cli, "input", test_filename, "Input test filename (.json).");
  add_option(cli, "output", output_filename, "Output image filename (.png).");
  add_option(cli, "model", model_filename, "Input model filename.");
  parse_cli(cli, num_args, args);

  auto test = bool_test{};
  if (!load_test(test, test_filename)) {
    print_fatal("Error loading test " + test_filename);
  }
  if (model_filename.size()) test.model = model_filename;

  auto          mesh = bool_mesh{};
  vector<vec2f> texcoords;
  vector<vec3f> colors;
  string        error;

  if (!load_shape(test.model, mesh, error)) {
    printf("%s\n", error.c_str());
    print_fatal("Error loading model " + test_filename);
  }
  init_mesh(mesh);
  printf("triangles: %d\n", (int)mesh.triangles.size());
  printf("positions: %d\n\n", (int)mesh.positions.size());

  auto bvh = make_triangles_bvh(mesh.triangles, mesh.positions, {});

  // auto state = state_from_test(mesh, test);
  auto state = make_test_state(mesh, bvh, test.camera, 0.005);

  {
    auto timer = print_timed("[compute_cells]");
    compute_cells(mesh, state);
    compute_shapes(state);
  }

  for (auto& operation : test.operations) {
    compute_bool_operation(state, operation);
  }

  auto scene = scene_scene{};
  scene.cameras.push_back(test.camera);

  for (int i = 0; i < state.cells.size(); i++) {
    auto& cell = state.cells[i];

    auto& instance    = scene.instances.emplace_back();
    instance.material = (int)scene.materials.size();
    auto& material    = scene.materials.emplace_back();
    auto  shape_id    = 0;
    for (int s = (int)state.shapes.size() - 1; s >= 0; s--) {
      if (state.shapes[s].cells.count(i)) {
        shape_id = s;
        break;
      }
    }

    material.color     = get_color(shape_id);
    material.type      = material_type::plastic;
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
