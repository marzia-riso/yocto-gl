#include "app.h"

using namespace yocto;

#include <deque>

#ifdef _WIN32
#undef near
#undef far
#endif

#ifdef MY_DEBUG
void debug_draw(app_state* app, int face, const vector<vec2i>& edges,
    const string& header = "") {
  static int count = 0;

  auto& is = debug_indices[face];

  //  auto       e     = vec2i{find_idx(is, edge_key.x), find_idx(is,
  //  edge_key.y)};

  auto base = app->test_filename;
  if (base == "") base = "data/tests/no-name.json";
  auto ext0 = ".triangulation" + to_string(count) + ".png";
  if (header.size()) {
    ext0 = "-" + header + ext0;
  }
  //  auto edges = vector<vec2i>{};
  //  for (auto& s : segments) {
  //    auto e = vec2i{find_idx(is, s.start_vertex), find_idx(is,
  //    s.end_vertex)}; edges.push_back({e.x, e.y});
  //  }
  debug_edges[face] = edges;

  // save_triangulation(replace_extension(base, ext0), face);

  save_test(app, "data/tests/crash.json");
  count += 1;
}

void debug_cells(app_state* app) {
  printf("Debugging cell: %d\n", app->current_patch);
  for (auto i = 0; i < app->cell_patches.size(); i++) {
    auto idx = app->cell_patches[i];

    app->glscene->instances[idx]->hidden = (i == app->current_patch) ? false
                                                                     : true;
  }

  app->current_patch = (app->current_patch + 1) % app->cell_patches.size();
}

void debug_borders(app_state* app) {
  printf("Debugging cell: %d\n", app->current_border);
  for (auto i = 0; i < app->state.polygons.size(); i++) {
    app->state.polygons[i].inner_shape->hidden = (i != app->current_border);
    app->state.polygons[i].outer_shape->hidden = (i != app->current_border);
  }

  app->current_border = (app->current_border + 1) % app->state.polygons.size();
}
#endif

// draw with shading
void draw_widgets(app_state* app, const gui_input& input) {
  auto widgets = &app->widgets;
  begin_imgui(widgets, "boolsurf", {0, 0}, {320, 720});

  if (draw_filedialog_button(widgets, "load test", true, "load file",
          app->test_filename, false, "data/tests/", "test.json", "*.json")) {
    load_test(app->test, app->test_filename);
    init_from_test(app);
  }
  continue_line(widgets);

  static auto filename = ""s;
  if (draw_filedialog_button(widgets, "save test", true, "save file", filename,
          true, "data/tests", "test.json", "*.json")) {
    save_test(app, filename);
  }

  static auto view_triangulation = false;
  draw_checkbox(widgets, "view triangulation", view_triangulation);
  if (view_triangulation) {
    // static ogl_texture* texture = new ogl_texture{};
    // ImGui::Begin("Triangulation viewer");
    // auto [x, y] = ImGui::GetWindowSize();
    // // auto size   = yocto::min(yocto::min(x, y), 1024);

    // // ImGui::Text("pointer = %p", texture);
    // auto face = app->last_clicked_point.face;
    // draw_triangulation(texture, face);
    // ImGui::Image((void*)texture->texture_id, {800, 800}, {0, 1}, {1, 0});
    // ImGui::End();
  }

  if (begin_header(widgets, "view")) {
    auto  glmaterial = app->mesh_material;
    auto& params     = app->drawgl_prms;
    // draw_checkbox(widgets, "faceted", params.faceted);
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
  if (begin_header(widgets, "Mesh")) {
    draw_label(widgets, "filename", app->model_filename);
    draw_label(
        widgets, "triangles", std::to_string(app->mesh.triangles.size()));
    draw_label(
        widgets, "positions", std::to_string(app->mesh.positions.size()));
    end_header(widgets);
  }

  if (begin_header(widgets, "face infos")) {
    auto face = app->last_clicked_point.face;
    draw_label(widgets, "face", std::to_string(face));

    auto [v0, v1, v2] = app->mesh.triangles[face];
    draw_label(widgets, "verts",
        "(" + to_string(v0) + ", " + to_string(v1) + ", " + to_string(v2) +
            ")");

    auto [a0, a1, a2] = app->mesh.adjacencies[face];
    draw_label(widgets, "adjs",
        "(" + to_string(a0) + ", " + to_string(a1) + ", " + to_string(a2) +
            ")");

    if (!app->mesh.border_tags.empty()) {
      auto [t0, t1, t2] = app->mesh.border_tags[face];
      draw_label(widgets, "tags",
          "(" + to_string(t0) + ", " + to_string(t1) + ", " + to_string(t2) +
              ")");
    }
    end_header(widgets);
  }

  if (app->selected_cell >= 0 && begin_header(widgets, "cell info", true)) {
    auto& cell = app->state.cells[app->selected_cell];
    draw_label(widgets, "cell", to_string(app->selected_cell));
    draw_label(widgets, "faces", to_string(cell.faces.size()));

    auto s = ""s;
    for (auto& [cell_id, _] : cell.adjacency) s += to_string(cell_id) + " ";
    draw_label(widgets, "adj", s);

    s = ""s;
    for (auto p = 1; p < cell.labels.size(); p++)
      s += to_string(cell.labels[p]) + " ";
    draw_label(widgets, "label", s);

    end_header(widgets);
  }

  if (app->selected_shape >= 0 && begin_header(widgets, "shape info", true)) {
    auto& shape_id = app->selected_shape;
    auto& shape    = app->state.shapes[shape_id];

    draw_label(widgets, "shape", to_string(shape_id));

    auto s = ""s;
    for (auto cell : shape.cells) s += to_string(cell) + " ";
    draw_label(widgets, "cells", s);

    if (draw_button(widgets, "Bring forward")) {
      if (shape_id < app->state.shapes.size() - 1) {
        swap(app->state.shapes[shape_id], app->state.shapes[shape_id + 1]);
        shape_id += 1;
        //        update_shapes(app); // TODO(giacomo): fix
        // set_default_shapes(app);
        update_cell_colors(app);
      }
    }
    continue_line(widgets);
    if (draw_button(widgets, "Bring back")) {
      if (shape_id >= 2) {
        swap(app->state.shapes[shape_id], app->state.shapes[shape_id - 1]);
        shape_id -= 1;
        // update_shapes(app);  // TODO(giacomo): fix
        // set_default_shapes(app);
        update_cell_colors(app);
      }
    }

    if (draw_coloredit(widgets, "color", shape.color)) {
      update_cell_colors(app);
    }
    end_header(widgets);
  }

  auto ff = [&](int i) { return to_string(i); };
  draw_combobox(
      widgets, "a", app->operation.shape_a, (int)app->state.shapes.size(), ff);
  draw_combobox(
      widgets, "b", app->operation.shape_b, (int)app->state.shapes.size(), ff);

  auto op = (int)app->operation.type;
  draw_combobox(widgets, "operation", op, bool_operation::type_names);
  app->operation.type = (bool_operation::Type)op;
  if (draw_button(widgets, "Apply")) {
    commit_state(app);
    compute_bool_operation(app->state, app->operation);
    app->test.operations += app->operation;
    update_cell_colors(app);
    app->operation = {};
  }

  end_imgui(widgets);
}

inline bool is_down(gui_button button) {
  return button.state == gui_button::state::down ||
         button.state == gui_button::state::pressing;
}

inline bool is_down(const gui_input& input, gui_key key) {
  return is_down(input.key_buttons[(int)key]);
}

void mouse_input(app_state* app, const gui_input& input) {
  if (is_active(&app->widgets)) return;

  if (input.mouse_left.state != gui_button::state::releasing) return;

  auto [isec, isec_original] = intersect_shapes(app, input);
  if (!isec.hit) return;
  auto point              = mesh_point{isec.element, isec.uv};
  app->last_clicked_point = point;

  auto point_original = mesh_point{isec_original.element, isec_original.uv};
  app->last_clicked_point_original = point_original;

  for (int i = 0; i < app->state.cells.size(); i++) {
    auto& cell = app->state.cells[i];
    auto  it   = find_idx(cell.faces, point.face);
    if (it != -1) {
      app->selected_cell = i;
      break;
    }
  }

  if (app->selected_cell != -1) {
    for (int s = (int)app->state.shapes.size() - 1; s >= 0; s--) {
      auto& shape = app->state.shapes[s];
      //      if (find_idx(shape.cells, app->selected_cell) != -1) {
      if (shape.cells.count(app->selected_cell) != 0) {
        app->selected_shape = s;
        if (is_down(input, gui_key::left_control)) {
          if (app->operation.shape_a == -1) {
            app->operation.shape_a = s;
          } else if (app->operation.shape_b == -1) {
            app->operation.shape_b = s;
          }
        }

        break;
      }
    }
  }

#ifdef MY_DEBUG
  debug_restart = true;
#endif

  if (input.modifier_alt) {
    commit_state(app);

    // Add point index to last polygon.
    auto polygon_id = (int)app->state.polygons.size() - 1;
    app->state.polygons[polygon_id].points.push_back(
        (int)app->state.points.size());

    // Add point to state.
    app->state.points.push_back(point);

    // TODO(giacomo): recomputing all paths of the polygon at every click is
    // bad
    update_polygon(app, polygon_id);
  }
}

void key_input(app_state* app, const gui_input& input) {
  if (is_active(&app->widgets)) return;

  for (auto idx = 0; idx < input.key_buttons.size(); idx++) {
    auto button = input.key_buttons[idx];
    if (button.state != gui_button::state::pressing) continue;

    switch (idx) {
      case (int)gui_key('Z'): {
        undo_state(app);
      } break;

      case (int)gui_key('Y'): {
        redo_state(app);
      } break;

      case (int)gui_key('B'): {
        auto& mesh           = app->mesh;
        auto  control_points = vector<mesh_point>{};
        auto& polygon        = app->state.polygons.back();
        for (auto& p : polygon.points) {
          control_points.push_back(app->state.points[p]);
        }
        auto points = compute_bezier_path(mesh.dual_solver, mesh.triangles,
            mesh.positions, mesh.adjacencies, control_points);

        polygon.points   = {};
        polygon.segments = {};
        for (auto& p : points) {
          polygon.points += (int)app->state.points.size();
          app->state.points += p;
        }

        update_polygons(app);

      } break;

      case (int)gui_key('I'): {
#ifdef MY_DEBUG
        debug_triangles.clear();
        debug_nodes.clear();
        debug_indices.clear();
#endif
        // remove trailing empty polygons.
        while (app->state.polygons.back().points.empty()) {
          app->state.polygons.pop_back();
        }

        compute_cells(app->mesh, app->state);

#ifdef MY_DEBUG
        save_tree_png(app->state, app->test_filename);
#endif

        compute_shapes(app->state);
        for (auto& op : app->test.operations) {
          compute_bool_operation(app->state, op);
        }

        app->cell_shapes.resize(app->state.cells.size());
        for (int i = 0; i < app->state.cells.size(); i++) {
          app->cell_shapes[i] = add_patch_shape(app, {}, vec3f{});
        }

        update_cell_shapes(app);
        update_cell_colors(app);

        for (auto& polygon : app->state.polygons) {
          if (polygon.polyline_shape) {
            polygon.polyline_shape->hidden = true;
          }
        }

        // update bvh
        app->bvh = make_triangles_bvh(
            app->mesh.triangles, app->mesh.positions, {});

        // update gpu data
        set_positions(app->mesh_instance->shape, app->mesh.positions);
        set_triangles(app->mesh_instance->shape, app->mesh.triangles);
        app->mesh.normals = compute_normals(app->mesh);
        set_normals(app->mesh_instance->shape, app->mesh.normals);
        init_edges_and_vertices_shapes_and_points(app);
        app->mesh_instance->hidden = true;

      } break;

      case (int)gui_key('S'): {
        app->state = {};
        auto svg   = load_svg(app->svg_filename);
        init_from_svg(
            app->state, app->mesh, app->last_clicked_point, svg, app->svg_size);
        update_polygons(app);
      } break;

#ifdef MY_DEBUG
      case (int)gui_key('N'): {
        debug_cells(app);
      } break;

      case (int)gui_key('F'): {
        auto add = [&](int face, int neighbor) -> bool {
          for (int k = 0; k < 3; k++) {
            if (app->mesh.border_tags[face][k] == 0) continue;
            if (find_in_vec(app->mesh.border_tags[neighbor],
                    -app->mesh.border_tags[face][k]) != -1)
              return false;
          }
          return true;
        };
        auto start = app->last_clicked_point.face;

        if (debug_restart) {
          debug_visited = vector<bool>(app->mesh.adjacencies.size(), false);
          debug_stack   = {start};
          debug_result.clear();
          debug_restart = false;
        }
        flood_fill_debug(app->mesh, {start}, add);
        auto visited = debug_result;

        for (int i = 0; i < visited.size(); i++) {
          auto tag = app->mesh.border_tags[visited[i]];
          auto adj = app->mesh.adjacencies[visited[i]];
          // printf("%d: tag(%d %d %d) adj(%d %d %d)\n", visited[i], tag[0],
          //     tag[1], tag[2], adj[0], adj[1], adj[2]);
        }

        if (app->temp_patch) {
          set_patch_shape(app->temp_patch->shape, app->mesh, visited);
        } else {
          app->temp_patch = add_patch_shape(app, visited, app->materials.blue);
        }
        app->temp_patch->depth_test = ogl_depth_test::always;
      } break;

      case (int)gui_key('G'): {
        auto add = [&](int face, int neighbor) -> bool {
          for (int k = 0; k < 3; k++) {
            if (app->mesh.border_tags[face][k] == 0) continue;
            if (find_in_vec(app->mesh.border_tags[neighbor],
                    -app->mesh.border_tags[face][k]) != -1)
              return false;
          }
          return true;
        };
        auto start   = app->last_clicked_point.face;
        auto visited = flood_fill(app->mesh, {start}, add);

        if (app->temp_patch) {
          set_patch_shape(app->temp_patch->shape, app->mesh, visited);
        } else {
          app->temp_patch = add_patch_shape(app, visited, app->materials.blue);
        }
        app->temp_patch->depth_test = ogl_depth_test::always;
      } break;
#endif

      case (int)gui_key('C'): {
        auto old_camera = app->glcamera;
        app->state.points.clear();
        app->state.polygons.clear();
        app->state.polygons.push_back(mesh_polygon{});
        load_shape(app, app->model_filename);
        clear_scene(app->glscene);
        init_glscene(app, app->glscene, app->mesh);
        app->glcamera = old_camera;
      } break;

      case (int)gui_key::enter: {
        if (app->state.polygons.back().points.size() > 2) {
          commit_state(app);
          app->state.polygons.push_back({});
        }
      } break;
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
  auto input       = ""s;
  auto window      = new gui_window{};

  window->msaa = 8;

  // parse command line
  auto cli = make_cli("yboolsurf", "views shapes inteactively");
  add_option(cli, "--camera", camera_name, "Camera name.");
  add_option(
      cli, "--resolution,-r", app->drawgl_prms.resolution, "Image resolution.");
  add_option(cli, "--lighting", app->drawgl_prms.lighting, "Lighting type.",
      shade_lighting_names);
  add_argument(cli, "input", input,
      "Input filename. Either a model or a json test file");
  add_option(cli, "--msaa", window->msaa, "Multisample anti-aliasing.");
  add_option(cli, "--test", app->test_filename, "Test filename.");
  add_option(cli, "svg", app->svg_filename, "Svg filename.");
  add_option(cli, "svg-size", app->svg_size, "Svg size.");
  parse_cli(cli, argc, argv);

  init_window(window, {1280 + 320, 720}, "boolsurf", true);

  window->user_data = app;

  auto extension = path_extension(input);
  if (extension == ".json") {
    app->test_filename = input;
    load_test(app->test, input);
    app->model_filename = app->test.model;
  } else {
    app->model_filename = input;
  }

  load_shape(app, app->model_filename);
  app->test.model = app->model_filename;

  init_glscene(app, app->glscene, app->mesh);
  if (window->msaa > 1) set_ogl_msaa();
  set_ogl_blending(true);

  app->widgets = create_imgui(window);
  app->window  = window;

  if (app->test_filename != "") {
    init_from_test(app);
  }

  app->state.polygons.push_back({});
  run_ui(window, update_app);

  // clear
  clear_scene(app->glscene);

  // done
  return 0;
}
