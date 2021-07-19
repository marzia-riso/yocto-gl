#include "app.h"

using namespace yocto;

#include <deque>

#include "../../libs/yocto_gui/ext/imgui/imgui.h"

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

void add_polygons(app_state* app, bool_test& test, const mesh_point& center,
    bool screenspace, bool straight_up = true) {
  add_polygons(app->state, app->mesh, app->camera, test, center,
      app->drawing_size, screenspace, straight_up);

  auto  last_shape = app->state.bool_shapes.size() - 1;
  auto& shape      = app->state.bool_shapes[last_shape];
  add_shape_shape(app, last_shape);
}

void load_svg(app_state* app) {
  auto script_path = normalize_path("scripts/svg_parser.py"s);
  auto test_json   = normalize_path("data/tests/tmp.json"s);
  auto cmd = "python3 "s + script_path + " "s + app->svg_filename + " "s +
             test_json + " "s + to_string(app->svg_subdivs);

  printf("%s\n", cmd.c_str());
  auto ret_value = system(cmd.c_str());
  if (ret_value != 0) print_fatal("Svg conversion failed " + app->svg_filename);

  app->temp_test = bool_test{};
  load_test(app->temp_test, test_json);
}

// draw with shading
void draw_svg_gui(gui_widgets* widgets, app_state* app) {
  if (draw_filedialog_button(widgets, "load", true, "load svg",
          app->svg_filename, false, "data/svgs/", "test.svg", "*.svg")) {
    load_svg(app);
  }

  if (app->svg_filename.empty()) return;
  continue_line(widgets);

  if (draw_button(widgets, "draw")) {
    commit_state(app);
    printf("Polygons: %d\n", app->state.polygons.size());
    add_polygons(
        app, app->temp_test, app->last_clicked_point, app->project_points);
    update_polygons(app);
  }

  continue_line(widgets);
  draw_checkbox(widgets, "project points", app->project_points);
  draw_label(widgets, "filename##svg-filename", app->svg_filename);

  if (draw_slider(widgets, "size##svg_size", app->drawing_size, 0.0, 0.1)) {
    // app->state.bool_shapes.resize((int)app->state.bool_shapes.size() - 1);
    // app->shape_shapes.resize((int)app->shape_shapes.size() - 1);
    update_svg(app);
  };

  if (draw_slider(widgets, "subdivs##svg_subdivs", app->svg_subdivs, 0, 16)) {
    app->state.bool_shapes.resize((int)app->state.bool_shapes.size() - 1);
    app->shape_shapes.resize((int)app->shape_shapes.size() - 1);
    update_svg(app);
  };

  draw_slider(widgets, "num polygons", app->num_sampled_polygons, 0, 500);
  if (draw_button(widgets, "bomb polygons")) {
    commit_state(app);
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

    // auto num_polygons = app->state.polygons.size();
    for (int i = 0; i < points.size(); i++) {
      auto& point = points[i];
      add_polygons(app, app->temp_test, point, app->project_points);
    }
    update_polygons(app);
  }

  save_test(app, app->state, "data/tests/bomb_polygon.json");
}

void bezier_last_segment(app_state* app) {
  commit_state(app);
  auto& mesh           = app->mesh;
  auto  control_points = vector<mesh_point>(
      app->state.points.end() - 4, app->state.points.end());
  auto bezier = compute_bezier_path(mesh.dual_solver, mesh.triangles,
      mesh.positions, mesh.adjacencies, control_points, 4);

  auto  polygon_id = (int)app->state.polygons.size() - 1;
  auto& polygon    = app->state.polygons.back();
  app->state.points.resize(app->state.points.size() - 4);
  polygon.points.resize(polygon.points.size() - 4);
  for (int i = 0; i < bezier.size(); i++) {
    polygon.points.push_back(app->state.points.size() + i);
  }
  app->state.points += bezier;
  // TODO (MARZIA): ricordati di fixare questo
  // update_polygon(app, polygon_id);
}

void draw_widgets(app_state* app, const gui_input& input) {
  auto widgets = &app->widgets;
  begin_imgui(widgets, "boolsurf", {0, 0}, {320, 720});
  draw_label(widgets, "selected", std::to_string(app->selected_point));

  if (draw_filedialog_button(widgets, "load test", true, "load file",
          app->test_filename, false, "data/tests/", "test.json", "*.json")) {
    load_test(app->test, app->test_filename);
    init_from_test(app);
  }
  continue_line(widgets);

  auto dir      = path_dirname(app->output_test_filename);
  auto filename = path_basename(app->output_test_filename) + string(".json");

  if (draw_filedialog_button(widgets, "save test", true, "save file",
          app->output_test_filename, true, dir, filename, "*.json")) {
    save_test(app, app->state, app->output_test_filename);
  }

  static auto scene_filename = "data/scenes/"s;
  draw_textinput(widgets, "##scene-name", scene_filename);
  continue_line(widgets);

  if (draw_button(widgets, "save scene")) {
    auto& cell_colors = app->test.cell_colors;
    for (int i = 0; i < cell_colors.size(); i++) {
      cell_colors[i] = app->cell_shapes[i]->material->color;
    }

    auto error = string{};
    if (!make_directory(scene_filename, error)) {
      printf("%s\n", error.c_str());
    }

    if (!make_directory(path_join(scene_filename, "shapes"), error)) {
      printf("%s\n", error.c_str());
    }

    // Saving output scene
    auto scene = make_scene(app->mesh, app->state, app->camera,
        app->color_shapes, app->save_edges, app->save_polygons, app->line_width,
        cell_colors);

    // auto scene = make_debug_scene(app->mesh, app->state, app->camera);
    save_scene(path_join(scene_filename, "scene.json"), scene, error);
  }

  if (begin_header(widgets, "SVG", app->svg_filename.size())) {
    draw_svg_gui(widgets, app);
    end_header(widgets);
  }

  if (begin_header(widgets, "view")) {
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

    // if (draw_checkbox(widgets, "arrows", app->show_arrows)) {
    //   for (auto i : app->arrow_shapes) {
    //     i->hidden = !app->show_arrows;
    //   }
    // }

    static auto view_triangulation = false;
    continue_line(widgets);
    draw_checkbox(widgets, "debug", view_triangulation);
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

    auto ff = [&](int i) { return to_string(i); };
    draw_combobox(widgets, "polygon", app->selected_polygon,
        (int)app->state.polygons.size(), ff);

    if (draw_button(widgets, "Invert polygon")) {
      if (app->selected_polygon >= 1) {
        auto& polygon = app->state.polygons[app->selected_polygon];
        reverse(polygon.points.begin(), polygon.points.end());
        reverse(polygon.edges.begin(), polygon.edges.end());

        for (auto& edge : polygon.edges) {
          reverse(edge.begin(), edge.end());
          for (auto& segment : edge) swap(segment.start, segment.end);
        }
      }
    }

    if (draw_button(widgets, "Delete polygon")) {
      if (app->selected_polygon >= 1) {
        app->state.polygons.erase(
            app->state.polygons.begin() + app->selected_polygon);
        printf("Removing polygon: %d\n", app->selected_polygon);
      }
    }

    if (draw_button(widgets, "Invert all")) {
      for (auto i = 0; i < app->state.polygons.size(); i++) {
        auto& polygon = app->state.polygons[i];
        printf("Reversing polygon: %d\n", i);
        reverse(polygon.points.begin(), polygon.points.end());
        reverse(polygon.edges.begin(), polygon.edges.end());
      }
    }
    end_header(widgets);
  }

  if (app->last_clicked_point.face >= 0 &&
      begin_header(widgets, "face", false)) {
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

  if (app->selected_cell >= 0 && begin_header(widgets, "cell")) {
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

    if (draw_coloredit(widgets, "color", app->test.cell_colors[cell_id])) {
      update_cell_colors(app);
    }

    end_header(widgets);
  }

  if (app->selected_shape >= 0 && begin_header(widgets, "shapes")) {
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

  auto open_booleans = app->selected_shape >= 0;
  if (open_booleans && begin_header(widgets, "booleans")) {
    auto ff = [&](int i) { return to_string(i); };
    draw_combobox(widgets, "a", app->operation.shape_a,
        (int)app->state.shapes.size(), ff);
    draw_combobox(widgets, "b", app->operation.shape_b,
        (int)app->state.shapes.size(), ff);

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

    if (draw_button(widgets, "Apply All")) {
      commit_state(app);
      for (auto& op : app->test.operations) {
        compute_bool_operation(app->state, op);
      }
      update_cell_colors(app);
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

    end_header(widgets);
  }

  if (begin_header(widgets, "mesh")) {
    draw_label(widgets, "filename", app->model_filename);
    draw_label(
        widgets, "triangles", std::to_string(app->mesh.triangles.size()));
    draw_label(
        widgets, "positions", std::to_string(app->mesh.positions.size()));
    end_header(widgets);
  }
  if (draw_button(widgets, "bezier")) {
    bezier_last_segment(app);
  }
  continue_line(widgets);
  if (draw_button(widgets, "bezier polygons")) {
    commit_state(app);
    auto& mesh       = app->mesh;
    auto  polygon_id = app->selected_polygon;
    for (int polygon_id = 0; polygon_id < app->state.polygons.size();
         polygon_id++) {
      auto& polygon = app->state.polygons[polygon_id];
      //    auto  _control_points = vector<mesh_point>(polygon.points.size());
      auto control_points = vector<mesh_point>{};
      for (int i = 0; i < polygon.points.size(); i++) {
        auto point = app->state.points[polygon.points[i]];
        // if (i % 2) {
        //   auto a    = point;
        //   auto b    = control_points.back();
        //   auto path = compute_geodesic_path(app->mesh, a, b);
        //   control_points += eval_geodesic_path(app->mesh, path, 0.5);
        // }
        control_points += point;
      }
      //    for (int i = 0; i < control_points.size(); i++) {
      //      control_points[i] = app->state.points[polygon.points[i]];
      //    }
      auto bezier = compute_bezier_path(mesh.dual_solver, mesh.triangles,
          mesh.positions, mesh.adjacencies, control_points, 4);
      // bezier      = control_points;
      polygon.points.resize(bezier.size());
      for (int i = 0; i < bezier.size(); i++) {
        polygon.points[i] = app->state.points.size() + i;
      }
      app->state.points += bezier;
      // TODO(MARZIA): ricordati di fixare questo
      // update_polygon(app, polygon_id);
    }
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
  if (point.face == -1) {
    app->selected_point = -1;
    return;
  }
  app->last_clicked_point          = point;
  app->last_clicked_point_position = eval_position(app->mesh, point);
  auto point_pos                   = app->last_clicked_point_position;

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

  auto update_selcted_point_shape = [&]() {
    if (app->selected_point == -1) {
      return;
    }
    if (!app->selected_point_shape) {
      app->selected_point_shape = add_instance(app->glscene);
    }
    app->selected_point_shape->hidden = (app->selected_point == -1);
    if (!app->selected_point_shape->shape) {
      app->selected_point_shape->shape = add_shape(app->glscene);
      auto sphere                      = make_sphere(8, 1);
      auto glshape                     = app->selected_point_shape->shape;
      set_quads(glshape, sphere.quads);
      set_positions(glshape, sphere.positions);
      set_instances(glshape, {});
    }
    if (!app->selected_point_shape->material) {
      app->selected_point_shape->material        = add_material(app->glscene);
      app->selected_point_shape->material->color = {0.1, 0, 0.9};
    }
    auto center = eval_position(
        app->mesh, app->state.points[app->selected_point]);
    app->selected_point_shape->frame.o = center;
  };

  if (!input.modifier_ctrl) {
    float min_dist = flt_max;
    for (int i = 0; i < app->state.points.size(); i++) {
      auto radius            = app->selected_point_radius;
      auto control_point_pos = eval_position(app->mesh, app->state.points[i]);
      auto dist              = length(control_point_pos - point_pos);
      if (dist < radius && dist < min_dist) {
        app->selected_point = i;
        min_dist            = dist;
      }
    }
  }

  if (input.modifier_ctrl && app->selected_point != -1) {
    app->state.points[app->selected_point] = point;
    update_polygons(app);
  }
  update_selcted_point_shape();

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

  debug_restart() = true;

  if (input.modifier_alt) {
    commit_state(app);

    auto  shape_id   = (int)app->state.bool_shapes.size() - 1;
    auto& bool_shape = app->state.bool_shapes.back();

    // Add point index to last polygon.
    auto  polygon_id = (int)bool_shape.polygons.size() - 1;
    auto& polygon    = bool_shape.polygons.back();
    polygon.points.push_back((int)app->state.points.size());

    // Add point to state.
    app->state.points.push_back(point);

    auto polygon_points = (int)polygon.points.size();

    update_polygon(app, shape_id, polygon_id, polygon_points - 2);
    // update_polygon(app, polygon_id, polygon_points - 2);
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
        bezier_last_segment(app);

      } break;

      case (int)gui_key('I'): {
        debug_triangles().clear();
        debug_nodes().clear();
        debug_indices().clear();

        // remove trailing empty polygons.
        // TODO (marzia): check this
        // while (app->state.polygons.back().points.empty()) {
        //   app->state.polygons.pop_back();
        // }

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

        // {
        //   auto ist      = add_instance(app->glscene);
        //   ist->shape    = add_shape(app->glscene);
        //   ist->material = add_material(app->glscene);
        //   set_arrow_shapes(ist->shape, {}, {});
        // }

        if (app->color_hashgrid) {
          auto faces = vector<int>();
          for (auto& [face, _] : app->mesh.triangulated_faces) {
            faces.push_back(face);
          }

          app->hashgrid_shape = add_patch_shape(
              app, faces, app->mesh_material->color * 0.65);
          // app->hashgrid_shape->depth_test = ogl_depth_test::always;
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
        printf("Computing operations\n");
        for (auto& op : app->test.operations) {
          compute_bool_operation(app->state, op);
        }

        printf("Computing borders\n");
        update_cell_colors(app);
        compute_shape_borders(app->mesh, app->state);

        auto new_state = compute_border_polygons(app->state);
        save_test(app, new_state, "data/tests/border_polygons.json");
        printf("Saved borders\n");

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

      case (int)gui_key('D'): {
        if (app->state.polygons.size() > 1) {
          app->state.polygons.resize(app->state.polygons.size() - 1);
          update_polygons(app);
        }
      } break;

        // case (int)gui_key('N'): {
        //   debug_cells(app);
        // } break;

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
          // auto tag = app->mesh.borders.tags[visited[i]];
          // auto adj = app->mesh.adjacencies[visited[i]];
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

      case (int)gui_key('C'): {
        auto old_camera = app->glcamera;
        app->state      = {};
        app->state.polygons.push_back({});
        reset_mesh(app->mesh);
        clear_scene(app->glscene);
        init_glscene(app, app->glscene, app->mesh);
        app->glcamera = old_camera;
      } break;

      case (int)gui_key('N'): {
        if (app->state.bool_shapes.empty()) return;

        auto& last_shape = app->state.bool_shapes.back();
        last_shape.polygons.pop_back();

        auto& last_shape_shape = app->shape_shapes.back();
        last_shape_shape.polygons.pop_back();

        auto  shape_id  = (int)app->state.bool_shapes.size();
        auto& new_shape = app->state.bool_shapes.emplace_back();
        new_shape.polygons.push_back({});

        add_shape_shape(app, shape_id);

      } break;

      case (int)gui_key::enter: {
        if (app->state.bool_shapes.empty()) return;
        auto& bool_shape = app->state.bool_shapes.back();

        if (bool_shape.polygons.empty()) return;
        auto& last_polygon = bool_shape.polygons.back();

        if (last_polygon.points.size() > 2) {
          commit_state(app);

          auto shape_id = (int)app->state.bool_shapes.size() - 1;

          auto& polygon = bool_shape.polygons.emplace_back();
          auto  p_shape = get_polygon_shape(app, polygon, shape_id);
          app->shape_shapes[shape_id].polygons.push_back(p_shape);
        }
      }

      break;
    }
  }
}

void update_app(const gui_input& input, void* data) {
  auto app = (app_state*)data;

  //  if ()
  {
    app->selected_point_radius =
        length(app->last_clicked_point_position - app->camera.frame.o) / 200;
    if (app->selected_point_shape) {
      app->selected_point_shape->frame.x.x = app->selected_point_radius;
      app->selected_point_shape->frame.y.y = app->selected_point_radius;
      app->selected_point_shape->frame.z.z = app->selected_point_radius;
    }
  }

  update_camera(app, input);
  mouse_input(app, input);
  key_input(app, input);

  drop(app, input);

  draw_scene(app, input);
  draw_widgets(app, input);
}

int main(int argc, const char* argv[]) {
  auto app            = new app_state{};
  auto input          = ""s;
  auto model_filename = ""s;
  auto window         = new gui_window{};

  window->msaa = 8;

  // parse command line
  auto cli = make_cli("yboolsurf", "views shapes inteactively");
  add_argument(cli, "input", input,
      "Input filename. Either a model or a json test file");
  add_option(cli, "model", model_filename, "Input model filename.");

  add_option(cli, "svg", app->svg_filename, "Svg filename.");
  add_option(cli, "svg-subdivs", app->svg_subdivs, "Svg subdivisions.");
  add_option(cli, "drawing-size", app->drawing_size, "Size of mapped drawing.");

  add_option(cli, "thick-lines", app->thick_lines, "Thick lines.");
  add_option(cli, "line-width", app->line_width, "Thick line width.");

  add_option(cli, "color-shapes", app->color_shapes, "Color shapes.");
  add_option(cli, "save-edges", app->save_edges, "Save mesh edges in scene.");
  add_option(
      cli, "save-polygons", app->save_polygons, "Save polygons in scene.");

  add_option(cli, "color-hashgrid", app->color_hashgrid, "Color hashgrid.");
  add_option(cli, "output-test", app->output_test_filename, "Output test.");

  parse_cli(cli, argc, argv);

  init_window(window, {1280 + 320, 720}, "boolsurf", true);

  window->user_data = app;

  if (app->svg_filename.size()) {
    load_svg(app);
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

    update_polygons(app);
  }

  // app->state.polygons.push_back({});

  // Adding first polygon
  auto bool_shape = shape{};
  bool_shape.polygons.push_back({});
  app->state.bool_shapes.push_back(bool_shape);

  for (auto s = 0; s < app->state.bool_shapes.size(); s++) {
    add_shape_shape(app, s);
  }

  app->history       = {app->state};
  app->history_index = 0;

  run_ui(window, update_app);

  // clear
  clear_scene(app->glscene);

  // done
  return 0;
}
