#include <boolsurf/boolsurf.h>
#include <boolsurf/boolsurf_io.h>
#include <yocto/yocto_cli.h>
#include <yocto/yocto_sampling.h>
#include <yocto/yocto_sceneio.h>
#include <yocto/yocto_shape.h>
#include <yocto/yocto_trace.h>

#include "serialize/serialize.h"
using namespace yocto;

struct test_stats {
  string model_filename      = ""s;
  int    model_triangles     = 0;
  int    genus               = 0;
  double triangulation_ms    = 0.0;
  double flood_fill_ms       = 0.0;
  double labelling_ms        = 0.0;
  double boolean_ms          = 0.0;
  double total_ms            = 0.0;
  int    polygons            = 0;
  int    control_points      = 0;
  int    added_points        = 0;
  int    sliced_triangles    = 0;
  int    added_triangles     = 0;
  int    graph_nodes         = 0;
  int    graph_edges         = 0;
  int    graph_cycles        = 0;
  int    graph_ambient_cells = 0;
};

void save_image(const string& output_image_filename, const bool_mesh& mesh,
    const bool_state& state, const scene_camera& camera, bool color_shapes,
    bool save_edges, bool save_polygons, float line_width, int spp) {
  if (output_image_filename == "no-output") return;
  auto scene = make_scene(
      mesh, state, camera, color_shapes, save_edges, save_polygons, line_width);

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

void save_test(const bool_state& state, const scene_camera& camera,
    const string& filename) {
  auto test     = bool_test{};
  test.points   = state.points;
  test.polygons = {{}};
  for (auto& mesh_polygon : state.polygons) {
    if (mesh_polygon.points.size()) {
      test.polygons.push_back(mesh_polygon.points);
    }
  }
  test.camera.frame    = camera.frame;
  test.camera.lens     = camera.lens;
  test.camera.aspect   = camera.aspect;
  test.camera.film     = camera.film;
  test.camera.aperture = camera.aperture;
  test.camera.focus    = camera.focus;

  save_test(test, filename);
}

int compute_mesh_genus(const bool_mesh& mesh) {
  auto edges = hash_set<vec2i>();
  for (auto& [a, b, c] : mesh.triangles) {
    edges.insert(make_edge_key({a, b}));
    edges.insert(make_edge_key({b, c}));
    edges.insert(make_edge_key({c, a}));
  }

  auto genus =
      (2 - mesh.triangles.size() - mesh.positions.size() + edges.size()) / 2;
  return genus;
}

int main(int num_args, const char* args[]) {
  auto test_filename         = ""s;
  auto output_image_filename = "data/render.png"s;
  auto output_scene_filename = ""s;
  auto output_json_filename  = ""s;
  auto spp                   = 4;
  auto model_filename        = ""s;
  auto svg_filename          = ""s;
  auto svg_subdivs           = 2;
  auto drawing_size          = 0.01f;
  auto color_shapes          = false;
  auto save_edges            = false;
  auto save_polygons         = false;
  auto line_width            = 0.003f;

  auto stats_filename = ""s;
  auto append_stats   = false;
  auto stats          = test_stats{};

  // parse command line
  auto cli = make_cli("test", "test boolsurf algorithms");
  add_argument(
      cli, "input", test_filename, "Input test filename (.json).", {}, false);
  add_option(cli, "output_image", output_image_filename,
      "Output image filename (.png).");
  add_option(cli, "output_scene", output_scene_filename, "");
  add_option(cli, "output_test", output_json_filename, "");
  add_option(cli, "spp", spp, "Samples per pixel.");
  add_option(cli, "model", model_filename, "Input model filename.");
  add_option(cli, "svg", svg_filename, "Input svg filename.");
  add_option(cli, "svg-subdivs", svg_subdivs, "Svg subdivisions.");
  add_option(cli, "drawing-size", drawing_size, "Size of mapped drawing.");
  add_option(cli, "line-width", line_width, "Size of polygons.");

  add_option(cli, "color-shapes", color_shapes, "Color shapes.");
  add_option(cli, "save-edges", save_edges, "Save mesh edges in scene.");
  add_option(cli, "save-polygons", save_polygons, "Save polygons in scene.");

  add_option(cli, "stats", stats_filename, "output stats");
  add_option(cli, "append-stats", append_stats, "append statistics");
  parse_cli(cli, num_args, args);

  test_filename = normalize_path(test_filename);
  if (!test_filename.size()) print_fatal("No input filename");

  string ioerror;
  if (output_scene_filename.size()) {
    output_scene_filename = normalize_path(output_scene_filename);
    if (!make_directory(path_dirname(output_scene_filename), ioerror))
      print_fatal(ioerror);
    if (!make_directory(
            path_join(path_dirname(output_scene_filename), "shapes"), ioerror))
      print_fatal(ioerror);
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
  test.model           = normalize_path(test.model);
  stats.model_filename = test.model;

  if (stats_filename.size() && !append_stats) {
    stats_filename  = normalize_path(stats_filename);
    auto stats_file = fopen(stats_filename.c_str(), "w");
    fprintf(stats_file,
        "model, model_triangles, genus, triangulation_ms, flood_fill_ms, labelling_ms, boolean_ms, total_ms, polygons, control_points, added_points, sliced_triangles, added_triangles, graph_nodes, graph_edges, graph_cycles, graph_ambient_cells\n");
    fclose(stats_file);
  }

  // Init mesh.
  auto error = string{};
  auto mesh  = bool_mesh{};
  {
    if (!load_shape(test.model, mesh, error)) {
      printf("%s\n", error.c_str());
      print_fatal("Error loading model " + test_filename);
    }

    init_mesh(mesh);
    stats.model_triangles = (int)mesh.triangles.size();
    printf("triangles: %d\n", (int)mesh.triangles.size());
    printf("positions: %d\n\n", (int)mesh.positions.size());
  }

  auto bvh = make_triangles_bvh(mesh.triangles, mesh.positions, {});

  // Init bool_state
  auto state = bool_state{};
  if (test.screenspace) {
    state = state_from_screenspace_test(mesh, test, drawing_size, false);
  } else {
    state = state_from_test(mesh, test, drawing_size, false);
  }

  if (output_json_filename.size()) {
    save_test(state, test.camera, output_json_filename);
  }

  for (auto& bool_shape : state.bool_shapes) {
    for (auto& polygon : bool_shape.polygons) {
      if (polygon.points.empty()) continue;

      stats.polygons += 1;
      stats.control_points += polygon.points.size();

      for (auto& edge : polygon.edges) {
        stats.added_points += edge.size();
      }
    }
  }

  stats.control_points += (int)state.isecs_generators.size();
  stats.genus = compute_mesh_genus(mesh);

  // Execute triangulation
  auto triangulation_timer = simple_timer{};
  slice_mesh(mesh, state);

  stats.triangulation_ms = elapsed_nanoseconds(triangulation_timer) /
                           pow(10, 6);
  stats.total_ms += stats.triangulation_ms;
  stats.sliced_triangles = (int)mesh.triangulated_faces.size();
  for (auto& [face, triangles] : mesh.triangulated_faces)
    stats.added_triangles += triangles.size();

  // Flood-fill for graph creation
  auto flood_fill_timer = simple_timer{};
  state.cells           = make_mesh_cells(mesh.adjacencies, mesh.borders);
  update_virtual_adjacencies(state.cells, mesh.borders);

  stats.flood_fill_ms = elapsed_nanoseconds(flood_fill_timer) / pow(10, 6);
  stats.total_ms += stats.flood_fill_ms;

  stats.graph_nodes = state.cells.size();
  for (auto& cell : state.cells) stats.graph_edges += cell.adjacency.size();
  stats.graph_edges /= 2;

  // Label propagation
  auto labelling_timer = simple_timer{};
  compute_cell_labels(state);
  stats.graph_cycles        = (int)state.cycles.size();
  stats.graph_ambient_cells = (int)state.ambient_cells.size();
  stats.labelling_ms        = elapsed_nanoseconds(labelling_timer) / pow(10, 6);
  stats.total_ms += stats.labelling_ms;
  printf("Total labelling time: %f\n", stats.labelling_ms);
  compute_shapes(state);

  // Saving output scene
  auto scene = make_scene(mesh, state, test.camera, color_shapes, save_edges,
      save_polygons, line_width);
  if (output_scene_filename.size()) {
    save_scene(output_scene_filename, scene, error);
  }

  // Saving render and cell adjacency graph
  save_image(output_image_filename, mesh, state, test.camera, color_shapes,
      save_edges, save_polygons, line_width, spp);

  // auto graph_dir      = path_dirname(output_image_filename);
  // auto graph_filename = path_basename(output_image_filename) +
  //                       string("_graph.png");
  // auto graph_outfile = path_join(graph_dir, graph_filename);
  // save_tree_png(state, graph_outfile.c_str(), "", color_shapes);

  auto booleans_timer = simple_timer{};
  for (auto& operation : test.operations) {
    compute_bool_operation(state, operation);
  }
  // compute_shape_borders(mesh, state);

  stats.boolean_ms = elapsed_nanoseconds(booleans_timer) / pow(10, 6);
  stats.total_ms += stats.boolean_ms;

  // output timings and stats:
  // model, model_triangles, genus,
  // triangulation_secs, flood_fill_secs, labelling_secs, boolean_secs,
  // total_secs, polygons, control_points, added_points, sliced_triangles,
  // added_triangles, graph_nodes, graph_edges, graph_cycles,
  // graph_ambient_cells
  if (stats_filename.size()) {
    auto stats_file = fopen(stats_filename.c_str(), "a");
    fprintf(stats_file,
        "%s, %d, %d, %f, %f, %f, %f, %f, %d, %d, %d, %d, %d, %d, %d, %d, %d\n",
        stats.model_filename.c_str(), stats.model_triangles, stats.genus,
        stats.triangulation_ms, stats.flood_fill_ms, stats.labelling_ms,
        stats.boolean_ms, stats.total_ms, stats.polygons, stats.control_points,
        stats.added_points, stats.sliced_triangles, stats.added_triangles,
        stats.graph_nodes, stats.graph_edges, stats.graph_cycles,
        stats.graph_ambient_cells);
    fclose(stats_file);
  }
}
