#include "app.h"
using namespace yocto;

#include <deque>

#ifdef _WIN32
#undef near
#undef far
#endif

void save_test(app_state* app, const string& filename) {
  app->test.points = app->state.points;
  app->test.polygons.clear();
  for (auto& mesh_polygon : app->state.polygons) {
    app->test.polygons.push_back(mesh_polygon.points);
  }
  save_test(app->test, filename);
}

void init_from_test(app_state* app) {
  app->state.polygons.clear();
  auto& points = app->state.points;
  points       = app->test.points;
  for (auto& polygon : app->test.polygons) {
    // Add new polygon to state.
    auto& mesh_polygon  = app->state.polygons.emplace_back();
    mesh_polygon.points = polygon;
  }
  update_polygons(app);
}

void debug_draw(app_state* app, int face, const vector<vec2i>& edges,
    const string& header = "") {
  static int count = 0;
  auto&      is    = debug_indices[face];
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

#include <yocto_gui/ext/imgui/imgui.h>
#include <yocto_gui/ext/imgui/imgui_impl_glfw.h>
#include <yocto_gui/ext/imgui/imgui_impl_opengl3.h>
#include <yocto_gui/ext/imgui/imgui_internal.h>

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
  if (begin_header(widgets, "inspect")) {
    draw_label(widgets, "filename", app->model_filename);
    auto ioshape = app->ioshape;
    draw_label(widgets, "points", std::to_string(ioshape->points.size()));
    draw_label(widgets, "lines", std::to_string(ioshape->lines.size()));
    draw_label(widgets, "triangles", std::to_string(ioshape->triangles.size()));
    draw_label(widgets, "quads", std::to_string(ioshape->quads.size()));
    draw_label(widgets, "positions", std::to_string(ioshape->positions.size()));
    draw_label(widgets, "normals", std::to_string(ioshape->normals.size()));
    draw_label(widgets, "texcoords", std::to_string(ioshape->texcoords.size()));
    draw_label(widgets, "colors", std::to_string(ioshape->colors.size()));
    draw_label(widgets, "radius", std::to_string(ioshape->radius.size()));
    draw_label(widgets, "quads pos", std::to_string(ioshape->quadspos.size()));
    draw_label(
        widgets, "quads norm", std::to_string(ioshape->quadsnorm.size()));
    draw_label(widgets, "quads texcoord",
        std::to_string(ioshape->quadstexcoord.size()));
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

    if (!app->mesh.tags.empty()) {
      auto [t0, t1, t2] = app->mesh.tags[face];
      draw_label(widgets, "tags",
          "(" + to_string(t0) + ", " + to_string(t1) + ", " + to_string(t2) +
              ")");
    }
    end_header(widgets);
  }

  if (app->selected_cell >= 0 && begin_header(widgets, "cell info", true)) {
    auto& cell = app->cells[app->selected_cell];
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
    draw_label(widgets, "polygon", to_string(shape.polygon));
    if (draw_button(widgets, "Bring forward")) {
      if (shape_id < app->state.shapes.size() - 1) {
        swap(app->state.shapes[shape_id], app->state.shapes[shape_id + 1]);
        shape_id += 1;
        //        update_shapes(app); // TODO(giacomo): fix
        set_default_shapes(app);
        update_cell_colors(app);
      }
    }
    continue_line(widgets);
    if (draw_button(widgets, "Bring back")) {
      if (shape_id >= 2) {
        swap(app->state.shapes[shape_id], app->state.shapes[shape_id - 1]);
        shape_id -= 1;
        // update_shapes(app);  // TODO(giacomo): fix
        set_default_shapes(app);
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
    compute_bool_operation(app->state.shapes, app->operation);
    app->test.operations += app->operation;
    update_cell_colors(app);
  }

  end_imgui(widgets);
}

geodesic_path compute_path(const mesh_polygon& polygon,
    const vector<mesh_point>& points, const bool_mesh& mesh) {
  auto size  = polygon.points.size();
  auto start = polygon.points[size - 2];
  auto end   = polygon.points[size - 1];
  auto path = compute_geodesic_path(mesh, points[start], points[end]);  // check
  return path;
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

  for (int i = 0; i < app->cells.size(); i++) {
    auto& cell = app->cells[i];
    auto  it   = find_idx(cell.faces, point.face);
    if (it != -1) {
      app->selected_cell = i;
      break;
    }
  }

  if (app->selected_cell != -1) {
    for (int s = 0; s < app->state.shapes.size(); s++) {
      auto& shape = app->state.shapes[s];
      if (find_idx(shape.cells, app->selected_cell) != -1) {
        app->selected_shape = s;
        break;
      }
    }
  }

  debug_restart = true;

  if (input.modifier_alt) {
    commit_state(app);

    // Add point index to last polygon.
    auto polygon_id = (int)app->state.polygons.size() - 1;
    app->state.polygons[polygon_id].points.push_back(app->state.points.size());

    // Add point to state.
    app->state.points.push_back(point);

    // TODO(giacomo): recomputing all paths of the polygon at every click is
    // bad
    update_polygon(app, polygon_id);
  }
}

vector<vec3i> face_tags(const bool_mesh& mesh, const mesh_hashgrid& hashgrid,
    const unordered_map<vec2i, vec2i>&     face_edgemap,
    const unordered_map<int, vector<int>>& triangulated_faces) {
  auto tags = vector<vec3i>(mesh.triangles.size(), zero3i);

  for (auto& [face, polylines] : hashgrid) {
    for (auto& polyline : polylines) {
      // TODO(giacomo): gestire caso in cui polyline sia chiusa...
      for (auto i = 0; i < polyline.vertices.size() - 1; i++) {
        auto polygon  = polyline.polygon;
        auto edge     = vec2i{polyline.vertices[i], polyline.vertices[i + 1]};
        auto edge_key = make_edge_key(edge);

        auto faces = vec2i{-1, -1};
        auto it    = face_edgemap.find(edge_key);
        if (it == face_edgemap.end()) {
          auto& t_faces = triangulated_faces.at(face);
          for (auto f : t_faces) {
            auto& tr = mesh.triangles[f];
            for (auto k = 0; k < 3; k++) {
              auto e = make_edge_key(get_edge(tr, k));
              if (edge_key == e) {
                auto neigh = mesh.adjacencies[f][k];
                faces      = {f, neigh};
                goto update;
              }
            }
          }
        } else {
          faces = it->second;
        }

      update:
        if (faces.x == -1 || faces.y == -1) {
          auto qualcosa = hashgrid.at(face);
          // debug_draw(app, face, segments);
          auto ff = mesh.adjacencies[face][1];
          // debug_draw(app, ff, segments, "other");
          assert(0);
        }

        // Il triangolo di sinistra ha lo stesso orientamento del poligono.
        auto& [a, b, c] = mesh.triangles[faces.x];

        auto [inner, outer] = faces;
        auto k              = find_in_vec(mesh.adjacencies[inner], outer);
        assert(k != -1);
        tags[inner][k] = -polygon;

        auto kk = find_in_vec(mesh.adjacencies[outer], inner);
        assert(kk != -1);
        tags[outer][kk] = +polygon;

        // Controlliamo che l'edge si nello stesso verso del poligono. Se non
        // e' cosi, invertiamo.
        if ((edge == vec2i{b, a}) || (edge == vec2i{c, b}) ||
            (edge == vec2i{a, c})) {
          tags[inner][k] *= -1;
          tags[outer][kk] *= -1;
          swap(faces.x, faces.y);  // if DRAW_BORDER_FACES
        }

        // #if DRAW_BORDER_FACES
        //         if (faces.x != -1)
        //           state.polygons[polygon].inner_faces.push_back(faces.x);
        //         if (faces.y != -1)
        //           state.polygons[polygon].outer_faces.push_back(faces.y);
        // #endif
      }
    }
  }
  return tags;
}

void triangulate(bool_mesh& mesh, unordered_map<vec2i, vec2i>& face_edgemap,
    unordered_map<int, vector<int>>& triangulated_faces,
    const mesh_hashgrid&             hashgrid) {
  for (auto& [face, polylines] : hashgrid) {
    auto [a, b, c] = mesh.triangles[face];

    // Nodi locali al triangolo.
    auto nodes = vector<vec2f>{{0, 0}, {1, 0}, {0, 1}};

    // Mappa i nodi locali ai vertici della mesh.
    auto indices = vector<int>{a, b, c};

    // Lista di edge-vincolo locali
    auto edges = vector<vec2i>();

    // Mappa che va da lato del triangolo k = 1, 2, 3 e a lista di nodi e lerp
    // corrispondenti su quel lato (serve per creare ulteriori vincoli)
    auto edgemap = array<vector<pair<int, float>>, 3>{};

    // Scorriamo su tutti i nodi che compongono le polilinee
    for (auto& polyline : polylines) {
      for (auto i = 0; i < polyline.points.size(); i++) {
        auto uv     = polyline.points[i];
        auto vertex = polyline.vertices[i];

        // Aggiungiamo un nuovo vertice se non è già presente nella lista dei
        // nodi
        auto local_vertex = find_idx(indices, vertex);
        if (local_vertex == -1) {
          indices.push_back(vertex);
          nodes.push_back(uv);
          local_vertex = (int)indices.size() - 1;
        }

        // Se non stiamo processando il primo nodo allora consideriamo anche
        // il nodo precedente e creiamo gli archi
        if (i != 0) {
          auto vertex_start       = polyline.vertices[i - 1];
          auto uv_start           = polyline.points[i - 1];
          auto local_vertex_start = find_idx(indices, vertex_start);

          // Se i nodi sono su un lato k != -1 di un triangolo allora li
          // salviamo nella edgemap
          auto [k, l] = get_mesh_edge(uv_start);
          if (k != -1) {
            edgemap[k].push_back({local_vertex_start, l});
          }

          tie(k, l) = get_mesh_edge(uv);
          if (k != -1) {
            edgemap[k].push_back({local_vertex, l});
          }

          // Se l'arco che ho trovato è un arco originale della mesh allora
          // salviamo la faccia corrispondente nel mapping da facce originale
          // a facce triangolate
          if (vertex_start < mesh.original_positions &&
              vertex < mesh.original_positions) {
            triangulated_faces[face] = {face};
          }

          // Aggiungiamo l'edge ai vincoli
          edges.push_back({local_vertex_start, local_vertex});
        }
      }
    }

    auto get_triangle_edge = [](int k) -> vec2i {
      if (k == 0) return {0, 1};
      if (k == 1) return {1, 2};
      if (k == 2) return {2, 0};
      return {-1, -1};
    };

    // Aggiungiamo gli edge di vincolo sia per i lati del triangolo
    for (int k = 0; k < 3; k++) {
      auto  tri_edge = get_triangle_edge(k);
      auto& points   = edgemap[k];

      // Se sul lato non ci sono altri punti allora aggiungiamo il lato stesso
      // ai vincoli
      if (points.size() == 0) {
        edges.push_back(tri_edge);
        continue;
      }

      // Se ci sono punti allora li ordiniamo per lerp crescente e creiamo i
      // vari vincoli
      if (points.size() > 1) {
        sort(points.begin(), points.end(), [](auto& a, auto& b) {
          auto& [node_a, l_a] = a;
          auto& [node_b, l_b] = b;
          return l_a < l_b;
        });
      }

      auto& [first, l] = points.front();
      auto& [last, l1] = points.back();
      edges.push_back({tri_edge.x, first});
      edges.push_back({last, tri_edge.y});

      for (auto i = 0; i < points.size() - 1; i++) {
        auto& [start, l] = points[i];
        auto& [end, l1]  = points[i + 1];
        edges.push_back({start, end});
      }
    }

    // Se nel triangolo non ho più di tre nodi allora non serve la
    // triangolazione
    if (nodes.size() == 3) continue;
    auto triangles = constrained_triangulation(nodes, edges);

#ifdef MY_DEBUG
    debug_nodes[face]     = nodes;
    debug_indices[face]   = indices;
    debug_triangles[face] = triangles;
#endif

    // Calcoliamo l'adiacenza locale e la trasformiamo in globale
    auto adjacency = face_adjacencies(triangles);
    for (auto& adj : adjacency) {
      for (auto& x : adj) {
        if (x == -1) {
          x = -2;
        } else {
          x += (int)mesh.triangles.size();
        }
      }
    }
    mesh.adjacencies += adjacency;

    // Aggiungiamo i nuovi triangoli alla mesh e aggiorniamo la face_edgemap
    // corrispondente
    triangulated_faces[face].clear();
    for (auto i = 0; i < triangles.size(); i++) {
      auto& [x, y, z] = triangles[i];
      auto v0         = indices[x];
      auto v1         = indices[y];
      auto v2         = indices[z];

      auto triangle_idx = (int)mesh.triangles.size();
      mesh.triangles.push_back({v0, v1, v2});

      update_face_edgemap(face_edgemap, {v0, v1}, triangle_idx);
      update_face_edgemap(face_edgemap, {v1, v2}, triangle_idx);
      update_face_edgemap(face_edgemap, {v2, v0}, triangle_idx);

      triangulated_faces[face].push_back(triangle_idx);
    }
  }
}

void compute_cells(app_state* app) {
  auto& polygons = app->state.polygons;
  auto& mesh     = app->mesh;

  auto vertices = add_vertices(mesh, polygons);

  auto hashgrid = compute_hashgrid(polygons, vertices);
  compute_intersections(hashgrid, mesh);

  // Mappa a ogni edge generato le due facce generate adiacenti.
  auto face_edgemap       = unordered_map<vec2i, vec2i>{};
  auto triangulated_faces = unordered_map<int, vector<int>>{};

  // Triangolazione e aggiornamento dell'adiacenza
  triangulate(mesh, face_edgemap, triangulated_faces, hashgrid);
  update_face_adjacencies(mesh, triangulated_faces);

  // Calcola i tags per ogni faccia
  app->mesh.tags = face_tags(
      app->mesh, hashgrid, face_edgemap, triangulated_faces);

  // Annulliamo le facce che sono già state triangolate
  for (auto& [face, triangles] : triangulated_faces) {
    if (triangles.size() <= 1) continue;
    mesh.triangles[face]   = {0, 0, 0};
    mesh.adjacencies[face] = {-3, -3, -3};
  }

  check_tags(app->mesh);

  // Trova l'adiacenza fra celle tramite il flood-fill
  app->cells = make_mesh_cells(mesh, mesh.tags);

  save_tree_png(app, "0");

  auto cycles = compute_graph_cycles(app->cells);

  auto skip_polygons = vector<int>();
  for (auto& cycle : cycles) {
    for (auto& [node, polygon] : cycle) {
      skip_polygons.push_back(polygon);
    }
  }

  // Calcoliamo il labelling definitivo per effettuare le booleane
  auto label_size = polygons.size();
  if (polygons.back().points.empty()) label_size -= 1;

  for (auto& cell : app->cells) {
    cell.labels = vector<int>(label_size, 0);
  }

  for (auto& cycle : cycles) {
    for (auto& c : cycle) {
      app->cells[c.x].labels[c.y] = 1;
    }
  }

  // Trova le celle ambiente nel grafo dell'adiacenza delle celle
  auto ambient_cells = find_ambient_cells(app->cells, skip_polygons);

  printf("Ambient cells: ");
  for (auto ambient_cell : ambient_cells) {
    auto cells = app->cells;
    compute_cell_labels(cells, {ambient_cell}, skip_polygons);

    auto found = false;
    for (int i = 0; i < cells.size(); i++) {
      auto& cell = cells[i];
      auto  it   = find_xxx(
          cell.labels, [](const int& label) { return label < 0; });
      if (it != -1) {
        found = true;
        break;
      }
    }

    if (!found) {
      app->cells        = cells;
      app->ambient_cell = ambient_cell;
      break;
    }
  }

  // assert(ambient_cells.size());

  save_tree_png(app, "1");

#if DRAW_BORDER_FACES
  // Draw inner and outer faces
  for (auto i = 0; i < polygons.size(); i++) {
    auto& polygon               = polygons[i];
    auto  a                     = app->materials.red;
    auto  b                     = app->materials.green;
    polygon.inner_shape         = add_patch_shape(app, polygon.inner_faces, a);
    polygon.outer_shape         = add_patch_shape(app, polygon.outer_faces, b);
    polygon.inner_shape->hidden = true;
    polygon.outer_shape->hidden = true;
  }

  auto face_polygons = unordered_map<int, vector<int>>();

  auto cells      = vector<vector<int>>();
  auto cell_faces = unordered_map<int, vector<int>>();

  for (auto p = 1; p < polygons.size(); p++) {
    if (!polygons[p].points.size()) continue;
    auto check = [&](int face, int polygon) {
      return find_in_vec(mesh.tags[face], polygon) == -1;
    };

    auto start_out   = polygons[p].outer_faces;
    auto visited_out = flood_fill(mesh, start_out, -p, check);
    for (auto o : visited_out) face_polygons[o].push_back(-p);

    auto start_in   = polygons[p].inner_faces;
    auto visited_in = flood_fill(mesh, start_in, p, check);
    for (auto i : visited_in) face_polygons[i].push_back(p);
  }
#endif
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

        compute_cells(app);

        init_shapes(app);
        set_default_shapes(app);
        for (auto& op : app->test.operations) {
          compute_bool_operation(app->state.shapes, op);
        }

        app->cell_shapes.resize(app->cells.size());
        for (int i = 0; i < app->cells.size(); i++) {
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
        app->mesh.normals = compute_normals(
            app->mesh.triangles, app->mesh.positions);
        set_normals(app->mesh_instance->shape, app->mesh.normals);
        init_edges_and_vertices_shapes_and_points(app);
        app->mesh_instance->hidden = true;

      } break;

      case (int)gui_key('N'): {
        debug_cells(app);
      } break;

      case (int)gui_key('B'): {
        debug_borders(app);
      } break;

      case (int)gui_key('F'): {
        auto add = [&](int face, int neighbor) -> bool {
          for (int k = 0; k < 3; k++) {
            if (app->mesh.tags[face][k] == 0) continue;
            if (find_in_vec(
                    app->mesh.tags[neighbor], -app->mesh.tags[face][k]) != -1)
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
          auto tag = app->mesh.tags[visited[i]];
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
            if (app->mesh.tags[face][k] == 0) continue;
            if (find_in_vec(
                    app->mesh.tags[neighbor], -app->mesh.tags[face][k]) != -1)
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

      case (int)gui_key('C'): {
        auto old_camera = app->glcamera;
        app->state.points.clear();
        app->state.polygons.clear();
        app->state.polygons.push_back(mesh_polygon{});
        load_shape(app, app->model_filename);
        clear_scene(app->glscene);
        init_glscene(app, app->glscene, app->mesh, {});
        app->glcamera = old_camera;
      } break;

      case (int)gui_key::enter: {
        commit_state(app);
        app->state.polygons.push_back({});
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
  add_option(cli, "input", input,
      "Input filename. Either a model or a json test file", true);
  add_option(cli, "--msaa", window->msaa, "Multisample anti-aliasing.");
  add_option(cli, "--test", app->test_filename, "Test filename.");
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

  init_glscene(app, app->glscene, app->mesh, {});
  if (window->msaa > 1) set_ogl_msaa();
  set_ogl_blending(true);

  app->widgets = create_imgui(window);
  app->window  = window;

  if (app->test_filename != "") {
    init_from_test(app);
  }

  run_ui(window, update_app);

  // clear
  clear_scene(app->glscene);

  // done
  return 0;
}
