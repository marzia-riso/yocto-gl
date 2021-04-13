#include <boolsurf/boolsurf.h>
#include <boolsurf/boolsurf_io.h>
#include <yocto/yocto_cli.h>
#include <yocto/yocto_sampling.h>
#include <yocto/yocto_sceneio.h>
#include <yocto/yocto_shape.h>
#include <yocto/yocto_trace.h>

#include "serialize/serialize.h"
using namespace yocto;

void save_image(const string& output_filename, const bool_mesh& mesh,
    const bool_state& state, const scene_camera& camera, bool color_shapes,
    int spp) {
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

  auto params    = trace_params{};
  auto error     = string{};
  params.sampler = trace_sampler_type::eyelight;
  params.samples = spp;
  auto image     = trace_image(scene, params);
  save_image(output_filename, image, error);
}

int main(int num_args, const char* args[]) {
  auto test_filename   = ""s;
  auto output_filename = "data/render.png"s;
  auto spp             = 4;
  auto model_filename  = ""s;
  auto svg_filename    = ""s;
  auto svg_subdivs     = 2;
  auto drawing_size    = 0.005f;
  auto color_shapes    = false;

  // parse command line
  auto cli = make_cli("test", "test boolsurf algorithms");
  add_argument(
      cli, "input", test_filename, "Input test filename (.json).", {}, false);
  add_option(cli, "output", output_filename, "Output image filename (.png).");
  add_option(cli, "spp", spp, "Samples per pixel.");
  add_option(cli, "model", model_filename, "Input model filename.");
  add_option(cli, "svg", svg_filename, "Input svg filename.");
  add_option(cli, "svg-subdivs", svg_subdivs, "Svg subdivisions.");
  add_option(cli, "drawing-size", drawing_size, "Size of mapped drawing.");
  add_option(cli, "color-shapes", color_shapes, "Color shapes.");
  parse_cli(cli, num_args, args);

  if (!test_filename.size()) print_fatal("No input filename");

  auto test      = bool_test{};
  auto extension = path_extension(test_filename);
  if (extension == ".svg") {
    auto script_path = normalize_path("scripts/svg_parser.py"s);
    auto output      = normalize_path("data/tests/tmp.json"s);
    auto cmd         = "python3 "s + script_path + " "s + test_filename + " "s +
               output + " "s + to_string(svg_subdivs);
    auto ret_value = system(cmd.c_str());
    if (ret_value != 0) print_fatal("Svg conversion failed " + test_filename);

    test_filename = output;
  }

  if (!load_test(test, test_filename)) {
    print_fatal("Error loading test " + test_filename);
  }

  if (model_filename.size()) test.model = model_filename;

  // Init mesh.
  auto error         = string{};
  auto mesh          = bool_mesh{};
  auto mesh_original = bool_mesh{};
  {
    if (!load_shape(test.model, mesh, error)) {
      printf("%s\n", error.c_str());
      print_fatal("Error loading model " + test_filename);
    }
    init_mesh(mesh);
    printf("triangles: %d\n", (int)mesh.triangles.size());
    printf("positions: %d\n\n", (int)mesh.positions.size());
    mesh_original = mesh;
  }
  auto bvh = make_triangles_bvh(mesh.triangles, mesh.positions, {});

  // Init bool_state
  auto state = bool_state{};

  if (test.screenspace) {
    int seed = 0;
    while (true) {
      bool repeat = false;
      test.camera = make_camera(mesh, seed++);
      state       = make_test_state(test, mesh, bvh, test.camera, drawing_size);
      printf("%s\n", "make_test_state");
      compute_cells(mesh, state);
      compute_shapes(state);
      auto zero              = vector<int>(state.cells[0].labels.size(), 0);
      auto ambient_num_faces = 0;
      for (auto& cell : state.cells) {
        if (cell.labels != zero) continue;
        if (ambient_num_faces < cell.faces.size()) {
          ambient_num_faces = (int)cell.faces.size();
        }
      }
      printf("ambient_num_faces: %d\n", ambient_num_faces);

      for (auto& cell : state.cells) {
        if (cell.faces.size() > ambient_num_faces) {
          repeat = true;
          break;
        }
      }

      if (!repeat && seed > 3) break;
      mesh = mesh_original;
    }
  } else {
    state = state_from_test(mesh, test, 0.005);
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

  save_image(output_filename, mesh, state, test.camera, color_shapes, spp);
}
