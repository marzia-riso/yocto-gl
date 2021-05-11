#include "app.h"

using namespace yocto;

#include <deque>

#include "../../libs/yocto_gui/ext/imgui/imgui.h"

#ifdef MY_DEBUG
void debug_draw(app_state* app, int face, const string& header = "") {
  static int count = 0;

  auto base = app->test_filename;
  if (base == "") base = "data/tests/no-name.json";
  auto ext0 = ".triangulation" + to_string(count) + ".png";
  if (header.size()) {
    ext0 = "-" + header + ext0;
  }

  // save_triangulation(replace_extension(base, ext0), face);

  save_test(app, app->state, "data/tests/crash.json");
  count += 1;
}

void debug_cells(app_state* app) {
  for (auto i = 0; i <= app->current_cell; i++)
    app->cell_shapes[i]->material->color = get_cell_color(
        app->state, i, app->color_shapes);

  for (auto i = app->current_cell + 1; i < app->cell_shapes.size(); i++)
    app->cell_shapes[i]->material->color = get_cell_color(
        app->state, 0, app->color_shapes);

  app->current_cell = (app->current_cell + 1) % app->cell_shapes.size();
}

void debug_cell_flood_fill(app_state* app) {
  for (auto i = 0; i < app->cell_patches.size(); i++) {
    auto idx = app->cell_patches[i];

    app->glscene->instances[idx]->hidden = (i == app->current_patch) ? false
                                                                     : true;
  }

  app->current_patch = (app->current_patch + 1) % app->cell_patches.size();
}

// void debug_borders(app_state* app) {
//   printf("Debugging cell: %d\n", app->current_border);
//   for (auto i = 0; i < app->state.polygons.size(); i++) {
//     app->state.polygons[i].inner_shape->hidden = (i != app->current_border);
//     app->state.polygons[i].outer_shape->hidden = (i != app->current_border);
//   }

//   app->current_border = (app->current_border + 1) %
//   app->state.polygons.size();
// }
#endif

void add_polygons(app_state* app, bool_test test, const mesh_point& center,
    bool screenspace) {
  auto& state    = app->state;
  auto& mesh     = app->mesh;
  auto& camera   = app->camera;
  auto  polygons = app->temp_test.polygons_screenspace;

  for (auto& polygon : polygons) {
    for (auto& uv : polygon) {
      uv *= app->svg_size;
      uv.x = -uv.x;
    }
  }

  auto get_projected_point = [&](vec2f uv) {
    uv.x /= camera.film;                    // input.window_size.x;
    uv.y /= (camera.film / camera.aspect);  // input.window_size.y;
    uv.x = -uv.x;
    uv += vec2f{0.5, 0.5};
    auto cam      = scene_camera{};
    auto position = eval_position(app->mesh, center);
    auto normal   = eval_normal(app->mesh, center);
    auto eye      = position + normal * 0.2;
    cam.frame     = lookat_frame(eye, position, {0, 1, 0});
    cam.focus     = length(eye - position);
    return intersect_mesh(app->mesh, cam, uv);
  };
  auto get_mapped_point = [&](vec2f uv) {
    uv /= camera.film;
    auto path     = straightest_path(mesh, center, uv);
    path.end.uv.x = clamp(path.end.uv.x, 0.0f, 1.0f);
    path.end.uv.y = clamp(path.end.uv.y, 0.0f, 1.0f);
    return path.end;
  };

  for (auto& polygon : polygons) {
    state.polygons.push_back({});
    auto polygon_id = (int)state.polygons.size() - 1;

    for (auto uv : polygon) {
      auto point = screenspace ? get_projected_point(uv) : get_mapped_point(uv);
      // print("point", point);
      if (point.face == -1) continue;

      // Add point to state.
      state.polygons[polygon_id].points.push_back((int)state.points.size());
      state.points.push_back(point);
    }

    if (state.polygons[polygon_id].points.size() <= 2) {
      assert(0);
      state.polygons[polygon_id].points.clear();
      continue;
    }
    // recompute_polygon_segments(mesh, state, state.polygons[polygon_id]);
  }

  for (auto p = app->last_svg.previous_polygons; p < app->state.polygons.size();
       p++) {
    auto& polygon = app->state.polygons[p];
    add_polygon_shape(app, polygon, p);
  }
}

void load_svg(app_state* app) {
  auto& last_svg             = app->last_svg;
  last_svg.previous_polygons = (int)app->state.polygons.size() - 1;

  auto script_path = normalize_path("scripts/svg_parser.py"s);
  auto test_json   = normalize_path("data/tests/tmp.json"s);
  auto cmd = "python3 "s + script_path + " "s + app->svg_filename + " "s +
             test_json + " "s + "2";

  printf("%s\n", cmd.c_str());
  auto ret_value = system(cmd.c_str());
  if (ret_value != 0) print_fatal("Svg conversion failed " + app->svg_filename);

  app->temp_test = bool_test{};
  load_test(app->temp_test, test_json);
}

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

  static auto test_filename = ""s;
  if (draw_filedialog_button(widgets, "save test", true, "save file",
          test_filename, true, "data/tests", "test.json", "*.json")) {
    save_test(app, app->state, test_filename);
  }

  continue_line(widgets);
  if (draw_filedialog_button(widgets, "load svg", true, "load svg",
          app->svg_filename, false, "data/svgs/", "test.svg", "*.svg")) {
    load_svg(app);
  }

  if (draw_button(widgets, "draw svg")) {
    commit_state(app);
    add_polygons(
        app, app->temp_test, app->last_clicked_point, app->project_points);
    update_polygons(app);
  }
  continue_line(widgets);
  draw_checkbox(widgets, "project points", app->project_points);

  if (draw_slider(widgets, "svg_size", app->svg_size, 0.0, 0.1)) {
    app->state.polygons.resize(app->last_svg.previous_polygons);
    update_svg(app);
  };

  if (draw_slider(widgets, "svg_subdivisions", app->svg_subdivs, 2, 16)) {
    app->state.polygons.resize(app->last_svg.previous_polygons);
    update_svg(app);
  };

  static auto scene_filename = "data/scenes/"s;
  draw_textinput(widgets, "scene", scene_filename);

  if (draw_button(widgets, "save scene")) {
    auto scene = make_scene(
        app->mesh, app->state, app->camera, app->color_shapes);
    auto error = string{};
    assert(make_directory(scene_filename, error));
    assert(make_directory(path_join(scene_filename, "shapes"), error));
    save_scene(path_join(scene_filename, "scene.json"), scene, error);
  }

  static auto view_triangulation = false;
  draw_checkbox(widgets, "view triangulation", view_triangulation);
  if (view_triangulation) {
    static ogl_texture* texture = new ogl_texture{};
    ImGui::Begin("Triangulation viewer");
    //    auto [x, y] = ImGui::GetWindowSize();
    // auto size   = yocto::min(yocto::min(x, y), 1024);

    // ImGui::Text("pointer = %p", texture);
    auto face = app->last_clicked_point_original.face;
    auto size = vec2i{1200, 800};
    draw_triangulation(texture, face, size * 4);
    ImGui::Image((void*)texture->texture_id, {float(size.x), float(size.y)},
        {0, 1}, {1, 0});
    ImGui::End();
  }

  // if (begin_header(widgets, "view"))
  {
    auto  glmaterial = app->mesh_material;
    auto& params     = app->drawgl_prms;
    draw_checkbox(widgets, "edges", app->glscene->instances[1]->hidden, true);
    continue_line(widgets);
    draw_checkbox(widgets, "points", app->glscene->instances[2]->hidden, true);
    continue_line(widgets);

    if (draw_checkbox(widgets, "polygons", app->show_polygons)) {
      for (auto i : app->polygon_shapes) {
        i->hidden = !app->show_polygons;
      }
      if (app->show_polygons) update_polygons(app);
    }

    if (!app->glscene->instances[1]->hidden) {
      draw_coloredit(widgets, "edges color", app->edges_material->color);
    }
    if (!app->glscene->instances[2]->hidden) {
      draw_coloredit(widgets, "points color", app->points_material->color);
    }
    draw_coloredit(widgets, "mesh color", glmaterial->color);
    // draw_slider(widgets, "resolution", params.resolution, 0, 4096);
    // draw_combobox(
    //     widgets, "lighting", (int&)params.lighting, shade_lighting_names);
    draw_checkbox(widgets, "wireframe", params.wireframe);
    continue_line(widgets);
    draw_checkbox(widgets, "double sided", params.double_sided);

    if (app->hashgrid_shape) {
      draw_checkbox(widgets, "hashgrid", app->hashgrid_shape->hidden, true);
    }
    if (app->border_faces_shapes.size()) {
      if (draw_checkbox(widgets, "border faces",
              app->border_faces_shapes[0]->hidden, true)) {
        for (auto& shape : app->border_faces_shapes) {
          shape->hidden = app->border_faces_shapes[0]->hidden;
        }
      }
    }
    // end_header(widgets);
  }

  draw_slider(widgets, "num polygons", app->num_sampled_polygons, 0, 500);
  if (draw_button(widgets, "bomb polygons")) {
    commit_state(app);
    load_svg(app);
    auto vertices = sample_vertices_poisson(
        app->mesh.graph, app->num_sampled_polygons);
    auto points = vector<mesh_point>{};
    for (auto& v : vertices) {
      for (int i = 0; i < app->mesh.triangles.size(); i++) {
        auto tr = app->mesh.triangles[i];
        if (tr.x == v) {
          points += mesh_point{i, {1.0 / 3, 1.0 / 3}};
          break;
        }
        if (tr.y == v) {
          points += mesh_point{i, {1.0 / 3, 1.0 / 3}};
          break;
        }
        if (tr.z == v) {
          points += mesh_point{i, {1.0 / 3, 1.0 / 3}};
          break;
        }
      }
    }

    auto num_polygons = app->state.polygons.size();
    for (int i = 0; i < points.size(); i++) {
      auto& point = points[i];
      add_polygons(app, app->temp_test, point, app->project_points);
    }
    update_polygons(app);
  }

  if (begin_header(widgets, "mesh info")) {
    draw_label(widgets, "filename", app->model_filename);
    draw_label(
        widgets, "triangles", std::to_string(app->mesh.triangles.size()));
    draw_label(
        widgets, "positions", std::to_string(app->mesh.positions.size()));
    end_header(widgets);
  }

  if (app->last_clicked_point.face >= 0 &&
      begin_header(widgets, "face info", true)) {
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

    if (!app->mesh.borders.tags.empty()) {
      auto [t0, t1, t2] = app->mesh.borders.tags[face];
      draw_label(widgets, "tags",
          "(" + to_string(t0) + ", " + to_string(t1) + ", " + to_string(t2) +
              ")");
    }
    end_header(widgets);
  }

  if (app->selected_cell >= 0 && begin_header(widgets, "cell info", true)) {
    auto& cell    = app->state.cells[app->selected_cell];
    auto  cell_id = app->selected_cell;
    draw_label(widgets, "cell", to_string(app->selected_cell));
    draw_label(widgets, "faces", to_string(cell.faces.size()));

    auto s = ""s;
    for (auto& [cell_id, _] : cell.adjacency) s += to_string(cell_id) + " ";
    draw_label(widgets, "adj", s);

    s = ""s;
    for (auto p = 1; p < app->state.labels[cell_id].size(); p++)
      s += to_string(app->state.labels[cell_id][p]) + " ";
    draw_label(widgets, "label", s);

    draw_coloredit(
        widgets, "color", app->cell_shapes[cell_id]->material->color);

    end_header(widgets);
  }

  if (app->selected_shape >= 0 && begin_header(widgets, "shape info", true)) {
    auto& shape_id   = app->selected_shape;
    auto& shape      = app->state.shapes[shape_id];
    auto  sorting_id = find_idx(app->state.shapes_sorting, shape_id);

    draw_label(widgets, "shape", to_string(shape_id));

    auto s = ""s;
    for (auto cell : shape.cells) s += to_string(cell) + " ";
    draw_label(widgets, "cells", s);

    if (draw_button(widgets, "Bring forward")) {
      if (sorting_id < app->state.shapes.size() - 1) {
        swap(app->state.shapes_sorting[sorting_id],
            app->state.shapes_sorting[sorting_id + 1]);
        update_cell_colors(app);
      }
    }
    continue_line(widgets);
    if (draw_button(widgets, "Bring back")) {
      if (sorting_id >= 2) {
        swap(app->state.shapes_sorting[sorting_id],
            app->state.shapes_sorting[sorting_id - 1]);
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

  if (draw_button(widgets, "Apply difference")) {
    commit_state(app);

    // auto indices = vector<int>(app->state.shapes.size() - 1);
    // for (auto i = 1; i < indices.size() + 1; i++) indices[(i - 1)] = i;

    // compute_symmetrical_difference(app->state, indices);
    // update_cell_colors(app);
    for (int i = 0; i < app->state.cells.size(); i++) {
      auto k = sum(app->state.labels[i]);
      if (k % 2 == 1) {
        // app->cell_shapes[i]->material->color = {1, 0, 0};
      } else {
        app->cell_shapes[i]->material->color = {1, 1, 1};
      }
    }
  }

  if (draw_button(widgets, "Clear operations")) {
    commit_state(app);
    app->operation = {};
    app->test.operations.clear();
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

  auto point = intersect_mesh(app, input);
  if (point.face == -1) return;
  app->last_clicked_point = point;

  auto point_original              = intersect_mesh_original(app, input);
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
    for (int s = (int)app->state.shapes_sorting.size() - 1; s >= 0; s--) {
      auto  shape_id = app->state.shapes_sorting[s];
      auto& shape    = app->state.shapes[shape_id];
      if (!shape.is_root) continue;
      //      if (find_idx(shape.cells, app->selected_cell) != -1) {
      if (shape.cells.count(app->selected_cell) != 0) {
        app->selected_shape = shape_id;
        if (is_down(input, gui_key::left_control)) {
          if (app->operation.shape_a == -1) {
            app->operation.shape_a = shape_id;
          } else if (app->operation.shape_b == -1) {
            app->operation.shape_b = shape_id;
          }
        }

        break;
      }
    }
  }

#ifdef MY_DEBUG
  debug_restart() = true;
#endif

  if (input.modifier_alt) {
    commit_state(app);

    // Add point index to last polygon.
    auto polygon_id = (int)app->state.polygons.size() - 1;

    app->state.polygons[polygon_id].points.push_back(
        (int)app->state.points.size());

    // Add point to state.
    app->state.points.push_back(point);
    auto polygon_points = app->state.polygons[polygon_id].points.size();

    // TODO(giacomo): recomputing all paths of the polygon at every click is
    // bad
    update_polygon(app, polygon_id, polygon_points - 2);
  }
}

void key_input(app_state* app, const gui_input& input) {
  if (is_active(&app->widgets)) return;

  for (auto idx = 0; idx < input.key_buttons.size(); idx++) {
    auto button = input.key_buttons[idx];
    if (button.state != gui_button::state::pressing) continue;

    if (input.modifier_shift) {
      float dir = 0;
      if (idx == (int)gui_key::up) dir = 0.75f;
      if (idx == (int)gui_key::down) dir = 1.5f;
      if (dir != 0) {
        auto point = app->last_clicked_point;
        if (point.face == -1) {
          point = intersect_mesh(app->mesh, app->camera, {0.5, 0.5});
        }
        if (point.face != -1) {
          auto pos      = eval_position(app->mesh, point);
          auto eye      = app->glcamera->frame.o;
          auto forward  = normalize(pos - eye);
          auto distance = length(eye - pos);
          distance *= dir;
          distance += app->glcamera->near;
          eye = pos - forward * distance;

          app->glcamera->frame = lookat_frame(eye, pos, {0, 1, 0});
          app->glcamera->focus = length(eye - pos);
          app->camera.frame    = app->glcamera->frame;
          app->camera.lens     = app->glcamera->lens;
          app->camera.film     = app->glcamera->film;
          app->camera.aspect   = app->glcamera->aspect;
          app->camera.aperture = app->glcamera->aperture;
          app->camera.focus    = app->glcamera->focus;
        }
      }
    }

    switch (idx) {
      case (int)gui_key::up: {
        app->glcamera->frame.o += app->glcamera->frame.y * 0.001;
      } break;
      case (int)gui_key::down: {
        app->glcamera->frame.o -= app->glcamera->frame.y * 0.001;
      } break;
      case (int)gui_key::left: {
        app->glcamera->frame.o -= app->glcamera->frame.x * 0.001;
      } break;
      case (int)gui_key::right: {
        app->glcamera->frame.o += app->glcamera->frame.x * 0.001;
      } break;

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

        polygon.points = {};
        polygon.edges  = {};
        for (auto& p : points) {
          polygon.points += (int)app->state.points.size();
          app->state.points += p;
        }

        update_polygons(app);

      } break;

      case (int)gui_key('I'): {
#ifdef MY_DEBUG
        debug_triangles().clear();
        debug_nodes().clear();
        debug_indices().clear();
#endif
        // remove trailing empty polygons.
        while (app->state.polygons.back().points.empty()) {
          app->state.polygons.pop_back();
        }

        compute_cells(app->mesh, app->state);

        // #ifdef MY_DEBUG
        save_tree_png(app->state, app->test_filename, "", app->color_shapes);
        // #endif

        compute_shapes(app->state);

        if (!app->color_hashgrid) {
          app->cell_shapes.resize(app->state.cells.size());
          for (int i = 0; i < app->state.cells.size(); i++) {
            app->cell_shapes[i] = add_patch_shape(app, {}, vec3f{});
          }

          update_cell_shapes(app);
          update_cell_colors(app);

          // for (auto p = 0; p < app->state.polygons.size(); p++) {
          //   app->polygon_shapes[p]->hidden = true;
          // }
          app->mesh_instance->hidden = true;
        }

        // update bvh
        app->mesh.bvh = make_triangles_bvh(
            app->mesh.triangles, app->mesh.positions, {});

        // update gpu data
        set_positions(app->mesh_instance->shape, app->mesh.positions);
        set_triangles(app->mesh_instance->shape, app->mesh.triangles);
        // TODO(giacomo): serve?
        app->mesh.normals = compute_normals(app->mesh);
        set_normals(app->mesh_instance->shape, app->mesh.normals);
        init_edges_and_vertices_shapes_and_points(app);

        if (app->color_hashgrid) {
          auto faces = vector<int>();
          for (auto& [face, _] : app->mesh.triangulated_faces) {
            faces.push_back(face);
          }
          app->hashgrid_shape = add_patch_shape(
              app, faces, app->mesh_material->color * 0.65);
          app->hashgrid_shape->depth_test = ogl_depth_test::always;
          app->glscene->instances += app->polygon_shapes;

          // auto inner_faces      = vector<int>{};
          // auto outer_faces      = vector<int>{};
          auto border_faces_map = hash_map<hash_set<int>, vector<int>>{};
          for (int i = 0; i < app->mesh.borders.tags.size(); i++) {
            auto tag     = app->mesh.borders.tags[i];
            auto tag_set = hash_set<int>{};
            if (tag.x < 0) tag_set.insert(-tag.x);
            if (tag.y < 0) tag_set.insert(-tag.y);
            if (tag.z < 0) tag_set.insert(-tag.z);
            if (tag_set.empty()) continue;
            border_faces_map[tag_set].push_back(i);
          }
          for (auto& [set, faces] : border_faces_map) {
            auto color = vec3f{0, 0, 0};
            for (auto& polygon_id : set) {
              color += get_color(polygon_id);
            }
            color /= set.size();
            app->border_faces_shapes += add_patch_shape(app, faces, color);
            app->border_faces_shapes.back()->depth_test =
                ogl_depth_test::always;
          }
        }
      } break;

      case (int)gui_key('O'): {
        // for (auto& op : app->test.operations) {
        //   compute_bool_operation(app->state, op);
        // }

        update_cell_colors(app);
        compute_shape_borders(app->mesh, app->state);

        auto state   = bool_state{};
        state.points = app->state.points;

        for (auto& shape : app->state.shapes) {
          if (!shape.is_root) continue;
          for (auto& border : shape.border_points) {
            auto& polygon = state.polygons.emplace_back();
            for (auto v : border) {
              auto id = app->state.control_points.at(v);
              polygon.points.push_back(id);
            }
          }
        }

        save_test(app, state, "data/tests/border_tmp.json");
        return;

        app->mesh = app->mesh_original;

        // for (auto& polygon : state.polygons) {
        //   recompute_polygon_segments(app->mesh, app->state, polygon);
        // }

        // for (auto& polygon : app->state.polygons) {
        //   clear_shape(polygon.polyline_shape->shape);
        // }

        // app->state = state;
        // for (int i = 0; i < app->state.polygons.size(); i++) {
        //   auto& polygon = app->state.polygons[i];
        //   set_polygon_shape(app->glscene, app->mesh, polygon, i);
        // }
        return;
      } break;

      case (int)gui_key('S'): {
      } break;

#ifdef MY_DEBUG
      case (int)gui_key('N'): {
        debug_cells(app);
      } break;

      case (int)gui_key('F'): {
        auto add = [&](int face, int neighbor) -> bool {
          for (int k = 0; k < 3; k++) {
            if (app->mesh.borders.tags[face][k] == 0) continue;
            if (find_in_vec(app->mesh.borders.tags[neighbor],
                    -app->mesh.borders.tags[face][k]) != -1)
              return false;
          }
          return true;
        };
        auto start = app->last_clicked_point.face;

        if (debug_restart()) {
          debug_visited() = vector<bool>(app->mesh.adjacencies.size(), false);
          debug_stack()   = {start};
          debug_result().clear();
          debug_restart() = false;
        }
        flood_fill_debug(app->mesh, {start}, add);
        auto visited = debug_result();

        for (int i = 0; i < visited.size(); i++) {
          auto tag = app->mesh.borders.tags[visited[i]];
          auto adj = app->mesh.adjacencies[visited[i]];
          // printf("%d: tag(%d %d %d) adj(%d %d %d)\n", visited[i], tag[0],
          //     tag[1], tag[2], adj[0], adj[1], adj[2]);
        }

        if (app->temp_patch) {
          set_patch_shape(app->temp_patch->shape, app->mesh, visited);
        } else {
          app->temp_patch = add_patch_shape(app, visited, {0, 1, 0});
        }
        app->temp_patch->depth_test = ogl_depth_test::always;
      } break;

      case (int)gui_key('G'): {
        auto add = [&](int face, int neighbor) -> bool {
          for (int k = 0; k < 3; k++) {
            if (app->mesh.borders.tags[face][k] == 0) continue;
            if (find_in_vec(app->mesh.borders.tags[neighbor],
                    -app->mesh.borders.tags[face][k]) != -1)
              return false;
          }
          return true;
        };
        auto start   = app->last_clicked_point.face;
        auto visited = flood_fill(app->mesh, {start}, add);

        if (app->temp_patch) {
          set_patch_shape(app->temp_patch->shape, app->mesh, visited);
        } else {
          app->temp_patch = add_patch_shape(app, visited, {0, 1, 0});
        }
        app->temp_patch->depth_test = ogl_depth_test::always;
      } break;

#endif

      case (int)gui_key('C'): {
        auto old_camera = app->glcamera;
        app->state      = {};
        app->state.polygons.push_back({});
        reset_mesh(app->mesh);
        clear_scene(app->glscene);
        init_glscene(app, app->glscene, app->mesh);
        app->glcamera = old_camera;
      } break;

      case (int)gui_key::enter: {
        if (app->state.polygons.back().points.size() > 2) {
          commit_state(app);
          auto& polygon = app->state.polygons.emplace_back();
          add_polygon_shape(app, polygon, (int)app->state.polygons.size() - 1);
        }
      }

      break;
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
  auto app            = new app_state{};
  auto camera_name    = ""s;
  auto input          = ""s;
  auto model_filename = ""s;
  auto window         = new gui_window{};

  window->msaa = 8;

  // parse command line
  auto cli = make_cli("yboolsurf", "views shapes inteactively");
  add_option(cli, "camera", camera_name, "Camera name.");
  add_argument(cli, "input", input,
      "Input filename. Either a model or a json test file");
  add_option(cli, "model", model_filename, "Input model filename.");
  // add_option(cli, "msaa", window->msaa, "Multisample anti-aliasing.");
  add_option(cli, "svg", app->svg_filename, "Svg filename.");
  add_option(cli, "svg-subdivs", app->svg_subdivs, "Svg subdivisions.");

  // add_option(cli, "svg-size", app->svg_size, "Svg size.");
  add_option(cli, "drawing-size", app->svg_size, "Size of mapped drawing.");
  add_option(cli, "color-shapes", app->color_shapes, "Color shapes.");
  add_option(cli, "color-hashgrid", app->color_hashgrid, "Color hashgrid.");
  parse_cli(cli, argc, argv);

  init_window(window, {1280 + 320, 720}, "boolsurf", true);

  window->user_data = app;

  if (app->svg_filename.size()) {
  }

  auto extension = path_extension(input);
  if (extension == ".svg") {
    auto script_path = normalize_path("scripts/svg_parser.py"s);
    auto output      = normalize_path("data/tests/tmp.json"s);
    auto cmd = "python3 "s + script_path + " "s + input + " "s + output + " "s +
               to_string(app->svg_subdivs);

    auto ret_value = system(cmd.c_str());
    if (ret_value != 0) print_fatal("Svg conversion failed " + input);
    app->test_filename = output;
    load_test(app->test, output);
  } else if (extension == ".json") {
    app->test_filename = input;
    load_test(app->test, input);
    app->model_filename = app->test.model;
  } else {
    app->model_filename = input;
  }

  if (model_filename != "") {
    app->model_filename = model_filename;
  }

  load_shape(app, app->model_filename);
  app->test.model = app->model_filename;  // To save it later.

  init_glscene(app, app->glscene, app->mesh);
  if (window->msaa > 1) set_ogl_msaa();
  set_ogl_blending(true);

  app->widgets = create_imgui(window);
  app->window  = window;

  if (app->test_filename != "") {
    init_from_test(app);
  }
  app->state.polygons.push_back({});
  add_polygon_shape(app, {}, 0);
  add_polygon_shape(app, app->state.polygons.back(), 1);
  app->history       = {app->state};
  app->history_index = 0;

  run_ui(window, update_app);

  // clear
  clear_scene(app->glscene);

  // done
  return 0;
}
