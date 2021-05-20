#include <boolsurf/boolsurf.h>
#include <boolsurf/boolsurf_io.h>
#include <yocto/yocto_cli.h>
#include <yocto/yocto_sampling.h>
#include <yocto/yocto_sceneio.h>
#include <yocto/yocto_shape.h>
#include <yocto/yocto_trace.h>

#include "serialize/serialize.h"
using namespace yocto;

void save_image(const string& output_image_filename, const bool_mesh& mesh,
    const bool_state& state, const scene_camera& camera, bool color_shapes,
    int spp) {
  if (output_image_filename == "no-output") return;

  auto scene = make_scene(mesh, state, camera, color_shapes);

  auto params    = trace_params{};
  auto error     = string{};
  params.sampler = trace_sampler_type::eyelight;
  params.samples = spp;
  auto image     = trace_image(scene, params);
  save_image(output_image_filename, image, error);
}

void save_image(
    const string& output_image_filename, const scene_model& scene, int spp) {
  if (output_image_filename == "no-output") return;
  auto params    = trace_params{};
  auto error     = string{};
  params.sampler = trace_sampler_type::eyelight;
  params.samples = spp;
  auto image     = trace_image(scene, params);
  save_image(output_image_filename, image, error);
}

bool check_ambient_cell(const bool_state& state) {
  if (state.cells.size() == 1) return false;

  // Getting max faces in ambient cell
  auto zero               = vector<int>(state.labels[0].size(), 0);
  auto ambient_cell_faces = 0;
  for (int c = 0; c < state.cells.size(); c++) {
    auto& cell = state.cells[c];
    if (state.labels[c] != zero) continue;
    ambient_cell_faces = max(ambient_cell_faces, (int)cell.faces.size());
  }

  printf("ambient_cell_faces: %d\n", ambient_cell_faces);
  for (auto& cell : state.cells) {
    if (cell.faces.size() > ambient_cell_faces) return false;
  }

  return true;
}

int main(int num_args, const char* args[]) {
  auto test_filename         = ""s;
  auto output_image_filename = "data/render.png"s;
  auto output_scene_filename = ""s;
  auto spp                   = 4;
  auto model_filename        = ""s;
  auto svg_filename          = ""s;
  auto svg_subdivs           = 2;
  auto drawing_size          = 0.005f;
  auto color_shapes          = false;

  // parse command line
  auto cli = make_cli("test", "test boolsurf algorithms");
  add_argument(
      cli, "input", test_filename, "Input test filename (.json).", {}, false);
  add_option(cli, "output_image", output_image_filename,
      "Output image filename (.png).");
  add_option(cli, "output_scene", output_scene_filename, "");
  add_option(cli, "spp", spp, "Samples per pixel.");
  add_option(cli, "model", model_filename, "Input model filename.");
  add_option(cli, "svg", svg_filename, "Input svg filename.");
  add_option(cli, "svg-subdivs", svg_subdivs, "Svg subdivisions.");
  add_option(cli, "drawing-size", drawing_size, "Size of mapped drawing.");
  add_option(cli, "color-shapes", color_shapes, "Color shapes.");
  parse_cli(cli, num_args, args);

  if (!test_filename.size()) {
    print_fatal("No input filename");
  } else {
    printf("test filename: %s\n", test_filename.c_str());
  }

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

  int  seed = 0;
  auto rng  = make_rng(seed);
  auto stop = false;

  // Init bool_state
  auto state = bool_state{};
  if (test.screenspace) {
    while (!stop) {
      state         = {};
      auto uv       = vec2f{0.5, 0.5};
      auto cam      = scene_camera{};
      auto eye      = sample_sphere(rand2f(rng)) * 4;
      auto position = vec3f{0, 0, 0};
      cam.frame     = lookat_frame(eye, position, {0, 1, 0});
      cam.focus     = length(eye - position);

      auto center = intersect_mesh(mesh, cam, uv);
      test.camera = make_camera(mesh, seed);

      add_polygons(state, mesh, test.camera, test, center, drawing_size, false);
      test.camera = cam;

      try {
        auto timer = print_timed("[compute_cells]");
        compute_cells(mesh, state);
        compute_shapes(state);
        stop = check_ambient_cell(state);

      } catch (const std::exception&) {
        stop = true;
      }

      if (stop) break;
      mesh = mesh_original;
    }
  } else {
    state = state_from_test(mesh, test, 0.005, false);
    compute_cells(mesh, state);
    compute_shapes(state);
  }

  // Saving output scene
  auto scene = make_scene(mesh, state, test.camera, color_shapes);
  if (output_scene_filename.size()) {
    // for (auto& shape : scene.shapes) {
    //   save_shape(shape, output_scene_filename + "/")
    // }
    save_scene(output_scene_filename, scene, error);
  }

  // Saving render and cell adjacency graph
  save_image(
      output_image_filename, mesh, state, test.camera, color_shapes, spp);
  auto graph_dir      = path_dirname(output_image_filename);
  auto graph_filename = path_basename(output_image_filename) +
                        string("_graph.png");
  auto graph_outfile = path_join(graph_dir, graph_filename);
  save_tree_png(state, graph_outfile.c_str(), "", color_shapes);

  if (color_shapes) {
    for (auto& operation : test.operations) {
      compute_bool_operation(state, operation);
    }
  }
}
