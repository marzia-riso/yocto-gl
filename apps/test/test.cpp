#include <boolsurf/boolsurf.h>
#include <boolsurf/boolsurf_io.h>
#include <yocto/yocto_cli.h>
#include <yocto/yocto_sampling.h>
#include <yocto/yocto_sceneio.h>
#include <yocto/yocto_shape.h>
#include <yocto/yocto_trace.h>

#include "serialize/serialize.h"
using namespace yocto;

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

  if (test.screenspace) {
    camera = make_camera(mesh);
    state  = make_test_state(test, mesh, bvh, camera, 0.005);
    // test.operations.push_back({1, 2, bool_operation::Type::op_difference});
    // test.operations.push_back({3, 4, bool_operation::Type::op_difference});
    // test.operations.push_back({8, 5, bool_operation::Type::op_difference});
    // test.operations.push_back(
    //     {6, 9, bool_operation::Type::op_symmetrical_difference});
  } else {
    state  = state_from_test(mesh, test);
    camera = test.camera;
  }

  {
    auto timer = print_timed("[compute_cells]");
    compute_cells(mesh, state);
    compute_shapes(state);
  }

  auto graph_dir      = path_dirname(output_filename);
  auto graph_filename = path_basename(output_filename) + string("_graph.png");
  auto graph_outfile  = path_join(graph_dir, graph_filename);

  save_tree_png(state, graph_outfile.c_str(), "", color_shapes);

  if (color_shapes) {
    for (auto& operation : test.operations) {
      compute_bool_operation(state, operation);
    }
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
