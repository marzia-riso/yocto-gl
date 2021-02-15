#include "app.h"
using namespace yocto;

#include <deque>

#ifdef _WIN32
#undef near
#undef far
#endif

void save_test(app_state* app, const string& filename) {
  app->test        = {};
  app->test.model  = app->filename;
  app->test.points = app->state.points;
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

// Rappresentazione di un segmento all'interno di una faccia. Serivra' per la
// triangolazione. Teniamo sia la rapprezentazione discreta (come coppia di
// vertici della mesh) che la rapprezentazione in coordiate baricentriche.
struct triangle_segment {
  int polygon      = -1;
  int start_vertex = -1;
  int end_vertex   = -1;

  vec2f start = {};
  vec2f end   = {};
};

void debug_draw(app_state* app, int face,
    const vector<triangle_segment>& segments, const string& header = "") {
  static int count = 0;
  auto&      n     = debug_nodes[face];
  auto&      is    = debug_indices[face];
  //  auto       e     = vec2i{find_idx(is, edge_key.x), find_idx(is,
  //  edge_key.y)};

  auto base = app->test_filename;
  if (base == "") base = "data/tests/no-name.json";
  auto ext0 = ".triangulation" + to_string(count) + ".png";
  if (header.size()) {
    ext0 = "-" + header + ext0;
  }
  auto edges = vector<vec2i>{};
  for (auto& s : segments) {
    auto e = vec2i{find_idx(is, s.start_vertex), find_idx(is, s.end_vertex)};
    edges.push_back({e.x, e.y});
  }
  debug_edges[face] = edges;

  save_triangulation(replace_extension(base, ext0), face);

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

void debug_visits(app_state* app) {
  printf("Debugging polygon visit: %d\n", app->current_polygon);
  for (auto i = 0; i < app->state.polygons.size(); i++) {
    if (app->state.polygons[i].points.size()) {
      app->state.polygons[i].inner_visit->hidden = (i != app->current_polygon);
      app->state.polygons[i].outer_visit->hidden = (i != app->current_polygon);
    }
  }

  app->current_polygon = (app->current_polygon + 1) %
                         app->state.polygons.size();
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
          app->test_filename, false, "data/", "test.json", "*.json")) {
    load_test(app->test, app->test_filename);
    init_from_test(app);
  }
  continue_line(widgets);

  static auto filename = ""s;
  if (draw_filedialog_button(widgets, "save test", true, "save file", filename,
          true, "data/", "test.json", "*.json")) {
    save_test(app, filename);
  }

  static auto view_triangulation = false;
  draw_checkbox(widgets, "view triangulation", view_triangulation);
  if (view_triangulation) {
    static ogl_texture* texture = new ogl_texture{};
    ImGui::Begin("Triangulation viewer");
    auto [x, y] = ImGui::GetWindowSize();
    // auto size   = yocto::min(yocto::min(x, y), 1024);

    // ImGui::Text("pointer = %p", texture);
    auto face = app->last_clicked_point_original.face;
    draw_triangulation(texture, face);
    ImGui::Image((void*)texture->texture_id, {800, 800}, {0, 1}, {1, 0});
    ImGui::End();
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
    draw_label(widgets, "filename", app->filename);
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

  auto [isec_original, isec] = intersect_shapes(app, input);
  if (!isec.hit) return;
  auto point_original = mesh_point{isec_original.element, isec_original.uv};
  auto point          = mesh_point{isec.element, isec.uv};

  app->last_clicked_point_original = point_original;
  app->last_clicked_point          = point;

  if (input.modifier_alt) {
    commit_state(app);

    // Add point index to last polygon.
    auto polygon_id = app->state.polygons.size() - 1;
    app->state.polygons[polygon_id].points.push_back(app->state.points.size());

    // Add point to state.
    app->state.points.push_back(point);

    // TODO(giacomo): recomputing all paths of the polygon at every click is bad
    update_polygon(app, polygon_id);
  }
}

void do_the_thing(app_state* app) {
  auto& state             = app->state;
  auto  original_vertices = app->mesh.positions.size();

  // Checking mesh invariants
  auto pre_boundary   = compute_boundary_faces(app->mesh.adjacencies);
  auto pre_check      = check_mesh(app->mesh);
  auto pre_components = compute_mesh_components(app->mesh);

  printf("Pre boundary faces: %d\n", pre_boundary.size());
  printf("Pre adjacency check: %d\n", pre_check.size());
  printf("Pre components: %d\n", pre_components.size());

  // Riempiamo l'hashgrid con i segmenti per triangolo.
  // Hashgrid from triangle idx to <polygon idx, segment idx,
  // segment start uv, segment end uv> to handle intersections and
  // self-intersections
  auto hashgrid = compute_hashgrid(state.polygons);

  // Mappa segmento (polygon_id, segment_id) a lista di intersezioni.
  // Per ogni faccia dell'hashgrid, calcoliamo le intersezioni fra i segmenti
  // contenuti.
  auto intersections = compute_intersections(hashgrid, app->mesh, state.points);

  // Mappa ogni faccia alla lista di triangle_segments di quella faccia.
  auto triangle_segments = unordered_map<int, vector<triangle_segment>>{};
  auto face_edgemap      = unordered_map<vec2i, vec2i>{};

  // Aggiungiamo gli estremi dei segmenti come vertici della mesh.
  // Dobbiamo inserire nei punti giusti anche i punti di intersezione che
  // spezzano i segmenti.
  // Inoltre popoliamo triangle_segments.

  for (auto polygon_id = 0; polygon_id < state.polygons.size(); polygon_id++) {
    auto& segments = state.polygons[polygon_id].segments;

    auto vertices = vector<int>(segments.size());
    for (auto i = 0; i < segments.size(); i++) {
      vertices[i] = add_vertex(
          app->mesh, {segments[i].face, segments[i].start});
    }

    // Per ogni segmento del poligono.
    for (auto segment_id = 0; segment_id < segments.size(); segment_id++) {
      auto& segment = segments[segment_id];

      auto start_uv     = segment.start;
      auto start_vertex = vertices[segment_id];

      // Se questo segmento aveva una o piu' interesezioni con altri segmenti...
      if (intersections.find({polygon_id, segment_id}) != intersections.end()) {
        auto& isecs = intersections[{polygon_id, segment_id}];

        // Popoliamo triangle_segments.
        for (auto& [end_vertex, l] : isecs) {
          auto end_uv = lerp(segment.start, segment.end, l);
          if (start_vertex != end_vertex) {
            triangle_segments[segment.face].push_back(
                {polygon_id, start_vertex, end_vertex, start_uv, end_uv});
          }

          // Accorcio il segmento corrente.
          start_uv     = end_uv;
          start_vertex = end_vertex;
        }
      }

      auto end_uv = segment.end;

      // L'indice del prossimo vertice che aggiungeremo al prossimo giro.
      auto end_vertex = vertices[(segment_id + 1) % vertices.size()];

      if (start_vertex < original_vertices && end_vertex < original_vertices) {
        auto get_edge = [&](const vec3i& triangle,
                            vec2i        edge) -> tuple<vec2i, int> {
          if (edge.x > edge.y) swap(edge.x, edge.y);
          for (auto i = 0; i < 3; i++) {
            auto x = triangle[i];
            auto y = triangle[(i + 1) % 3];

            if ((vec2i{x, y} == edge) || (vec2i{y, x} == edge))
              return {edge, i};
          }
        };

        printf("Are we skipping this?: %d %d - first face: %d\n", start_vertex,
            end_vertex, segment.face);

        auto& triangle = app->mesh.triangles[segment.face];
        auto [edge, k] = get_edge(triangle, {start_vertex, end_vertex});

        auto  neighbor       = app->mesh.adjacencies[segment.face][k];
        auto& triangle_neigh = app->mesh.triangles[neighbor];

        update_face_edgemap(face_edgemap, edge, segment.face);
        update_face_edgemap(face_edgemap, edge, neighbor);
        triangle_segments[segment.face].push_back(
            {polygon_id, start_vertex, end_vertex, zero2f, zero2f});
        continue;
      }

      if (start_vertex == end_vertex) {
        continue;
      }

      triangle_segments[segment.face].push_back(
          {polygon_id, start_vertex, end_vertex, start_uv, end_uv});
    }
  }

  // Adesso possiamo triangolare ogni faccia.

  //(marzia) collapse the triangle_segments iterations
  // Not now, they're useful while debugging

  debug_triangles.clear();
  debug_nodes.clear();
  debug_indices.clear();

  // Mappa a ogni edge generato le due facce generate adiacenti.
  for (auto& [face, segments] : triangle_segments) {
    auto [a, b, c] = app->mesh.triangles[face];
    auto nodes     = vector<vec2f>{{0, 0}, {1, 0}, {0, 1}};

    // Mappa i nodi locali ai vertici della mesh.
    auto indices = vector<int>{a, b, c};

    // Lista di archi-vincolo locali
    auto edges      = vector<vec2i>();
    auto edgemap    = unordered_map<vec2i, vector<tuple<int, float>>>();
    edgemap[{0, 1}] = {};
    edgemap[{1, 2}] = {};
    edgemap[{0, 2}] = {};

    // Per ogni segmento della faccia.
    for (auto s = 0; s < segments.size(); s++) {
      auto& [polygon_id, start_vertex, end_vertex, start_uv, end_uv] =
          segments[s];

      if ((start_uv == zero2f) && (end_uv == zero2f)) continue;

      // Aggiungi senza duplicati. Aggiornando indices insieme a nodes,
      // manteniamo la corrispondenza.
      auto edge_start = find_idx(indices, start_vertex);
      auto edge_end   = find_idx(indices, end_vertex);

      if (edge_start == -1) {
        edge_start = (int)indices.size();
        nodes.push_back(start_uv);
        indices.push_back(start_vertex);

        auto [tri_edge, l] = get_mesh_edge({0, 1, 2}, start_uv);
        if (tri_edge != zero2i) {
          auto tri_edge_key = make_edge_key(tri_edge);
          if (tri_edge_key != tri_edge) l = 1.0f - l;
          edgemap[tri_edge_key].push_back({edge_start, l});
        }
      }

      if (edge_end == -1) {
        edge_end = (int)indices.size();
        nodes.push_back(end_uv);
        indices.push_back(end_vertex);

        auto [tri_edge, l] = get_mesh_edge({0, 1, 2}, end_uv);
        if (tri_edge != zero2i) {
          auto tri_edge_key = make_edge_key(tri_edge);
          if (tri_edge_key != tri_edge) l = 1.0f - l;
          edgemap[tri_edge_key].push_back({edge_end, l});
        }
      }

      // Per adesso, ad ogni nuovo edge associamo due facce adiacenti nulle.
      // Ora serve per debugging.
      auto edge = make_edge_key({start_vertex, end_vertex});
      // face_edgemap[edge] = {-1, -1};

      edges.push_back({edge_start, edge_end});
    }

    for (auto& [tri_edge, points] : edgemap) {
      if (points.size() == 0) {
        edges.push_back(tri_edge);
        continue;
      }

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

    if (nodes.size() == 3) continue;
    auto triangles        = constrained_triangulation(face, nodes, edges);
    debug_nodes[face]     = nodes;
    debug_indices[face]   = indices;
    debug_triangles[face] = triangles;

    for (auto ee : edges) {
      auto found = false;
      for (auto& tr : triangles) {
        int k = 0;
        for (k = 0; k < 3; k++) {
          auto edge = vec2i{tr[k], tr[(k + 1) % 3]};
          if (make_edge_key(edge) == make_edge_key(ee)) {
            found = true;
            break;
          }
        }
      }
      if (!found) {
        debug_draw(app, face, segments);
        assert(0);
      }
    }

    // Aggiungiamo i nuovi triangoli e aggiorniamo la face_edgemap.
    for (auto i = 0; i < triangles.size(); i++) {
      auto& [x, y, z] = triangles[i];
      auto v0         = indices[x];
      auto v1         = indices[y];
      auto v2         = indices[z];

      auto triangle_idx = (int)app->mesh.triangles.size();
      app->mesh.triangles.push_back({v0, v1, v2});

      update_face_edgemap(face_edgemap, {v0, v1}, triangle_idx);
      update_face_edgemap(face_edgemap, {v1, v2}, triangle_idx);
      update_face_edgemap(face_edgemap, {v2, v0}, triangle_idx);
    }

    // Rendi triangolo originale degenere per farlo sparire.
    app->mesh.triangles[face] = {0, 0, 0};
  }  // end for triangle_segments

  auto temp = vector<pair<vec2i, vec2i>>{};
  for (auto& [key, value] : face_edgemap) {
    if (key == vec2i{17128, 17180}) {
      printf("Key: %d %d - %d %d\n", key.x, key.y, value.x, value.y);
    }

    if (key == vec2i{16784, 16786}) {
      printf("Key: %d %d - %d %d\n", key.x, key.y, value.x, value.y);
    }

    if (key == vec2i{39944, 40049}) {
      printf("Key: %d %d - %d %d\n", key.x, key.y, value.x, value.y);
    }

    if (value.x == -1 || value.y == -1) {
      temp.push_back({key, value});
      // printf("Key: %d %d - %d %d", key.x, key.y, value.x, value.y);
    }
  }
  // assert(temp.size() == 0);

  // Creiamo inner_faces e outer_faces di ogni poligono.
  for (auto& [face, segments] : triangle_segments) {
    for (auto i = 0; i < segments.size(); i++) {
      auto& [polygon, start_vertex, end_vertex, start_vu, end_uv] = segments[i];
      auto edge     = vec2i{start_vertex, end_vertex};
      auto edge_key = make_edge_key(edge);

      auto faces = face_edgemap.at(edge_key);

      if (faces.x == -1 || faces.y == -1) {
        auto qualcosa = hashgrid[face];

        debug_draw(app, face, segments);

        auto ff = app->mesh.adjacencies[face][1];
        debug_draw(app, ff, segments, "other");

        assert(0);
      }

      // Il triangolo di sinistra ha lo stesso orientamento del poligono.
      auto& [a, b, c] = app->mesh.triangles[faces.x];

      // Controlliamo che l'edge si nello stesso verso del poligono. Se non
      // e' cosi, invertiamo.
      if ((edge == vec2i{b, a}) || (edge == vec2i{c, b}) ||
          (edge == vec2i{a, c})) {
        swap(faces.x, faces.y);
      }

      if (faces.x != -1) state.polygons[polygon].inner_faces.push_back(faces.x);
      if (faces.y != -1) state.polygons[polygon].outer_faces.push_back(faces.y);
    }
  }

  // Removing face duplicates
  // TODO(giacomo): check if we can avoid this
  for (auto i = 1; i < state.polygons.size(); i++) {
    if (!state.polygons[i].points.size()) continue;
    auto& inner = state.polygons[i].inner_faces;
    auto& outer = state.polygons[i].outer_faces;

    sort(inner.begin(), inner.end());
    inner.erase(unique(inner.begin(), inner.end()), inner.end());

    sort(outer.begin(), outer.end());
    outer.erase(unique(outer.begin(), outer.end()), outer.end());
  }

  // Draw inner and outer faces
  for (auto i = 0; i < state.polygons.size(); i++) {
    auto& polygon               = state.polygons[i];
    auto  a                     = app->materials.red;
    auto  b                     = app->materials.green;
    polygon.inner_shape         = add_patch_shape(app, polygon.inner_faces, a);
    polygon.outer_shape         = add_patch_shape(app, polygon.outer_faces, b);
    polygon.inner_shape->hidden = true;
    polygon.outer_shape->hidden = true;
  }

  app->mesh.normals = compute_normals(app->mesh.triangles, app->mesh.positions);
  app->mesh.adjacencies = face_adjacencies(app->mesh.triangles);
  set_positions(app->mesh_instance->shape, app->mesh.positions);
  set_triangles(app->mesh_instance->shape, app->mesh.triangles);
  set_normals(app->mesh_instance->shape, app->mesh.normals);
  init_edges_and_vertices_shapes_and_points(app);
  app->bvh = make_triangles_bvh(app->mesh.triangles, app->mesh.positions, {});
  app->mesh_instance->hidden = true;

  // Checking mesh invariants
  auto post_boundary   = compute_boundary_faces(app->mesh.adjacencies);
  auto post_check      = check_mesh(app->mesh);
  auto post_components = compute_mesh_components(app->mesh);
  printf("Post boundary faces: %d\n", post_boundary.size());
  printf("Post adjacency check: %d\n", post_check.size());
  printf("Post components: %d\n", post_components.size());

  for (auto f : post_check) {
    auto& tri = app->mesh.triangles[f];
    auto& adj = app->mesh.adjacencies[f];
    printf("Face: %d - Vert: (%d %d %d) - Adj: (%d %d %d)\n", f, tri[0], tri[1],
        tri[2], adj[0], adj[1], adj[2]);
  }

  assert(pre_components.size() == post_components.size());

  app->mesh.tags      = compute_face_tags(app->mesh, state.polygons);
  auto& tags          = app->mesh.tags;
  auto  face_polygons = unordered_map<int, vector<int>>();

  auto cells      = vector<vector<int>>();
  auto cell_faces = unordered_map<int, vector<int>>();

  for (auto p = 1; p < state.polygons.size(); p++) {
    auto& polygon = state.polygons[p];

    if (!polygon.points.size()) continue;
    auto check = [&](int face, int polygon) {
      return find_in_vec(tags[face], polygon) == -1;
    };

    auto start_out   = polygon.outer_faces;
    auto visited_out = flood_fill(app->mesh, start_out, -p, check);
    for (auto o : visited_out) face_polygons[o].push_back(-p);

    auto start_in   = polygon.inner_faces;
    auto visited_in = flood_fill(app->mesh, start_in, p, check);
    for (auto i : visited_in) face_polygons[i].push_back(p);

    auto a                      = app->materials.blue;
    auto b                      = app->materials.light_blue;
    polygon.inner_visit         = add_patch_shape(app, visited_in, a);
    polygon.outer_visit         = add_patch_shape(app, visited_out, b);
    polygon.inner_visit->hidden = true;
    polygon.outer_visit->hidden = true;
  }

  // Inverting face_polygons map
  for (auto& [face, polygons] : face_polygons) {
    auto idx = find_idx(cells, polygons);
    if (idx == -1) {
      idx = (int)cells.size();
      cells.push_back(polygons);
    }

    cell_faces[idx].push_back(face);
  }

  for (auto i = 0; i < cells.size(); i++) {
    printf("Cell: %d -> ", i);
    for (auto c : cells[i]) printf("%d ", c);
    printf("\n\t Faces: %d \n", cell_faces[i].size());

    auto color =
        app->cell_materials[(i + 1) % app->cell_materials.size()]->color;

    app->cell_patches.push_back((int)app->glscene->instances.size());
    add_patch_shape(app, cell_faces[i], color);
  }
}

void key_input(app_state* app, const gui_input& input) {
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
        do_the_thing(app);
      } break;

      case (int)gui_key('N'): {
        debug_cells(app);
      } break;

      case (int)gui_key('B'): {
        debug_borders(app);
      } break;

      case (int)gui_key('V'): {
        debug_visits(app);
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
        auto start   = app->last_clicked_point.face;
        auto visited = flood_fill(app->mesh, {start}, add);

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

      case (int)gui_key('C'): {
        auto old_camera = app->glcamera;
        app->state.points.clear();
        app->state.polygons.clear();
        app->state.polygons.push_back(mesh_polygon{});
        load_shape(app, app->filename);
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
    app->filename = app->test.model;
  } else {
    app->filename = input;
  }

  load_shape(app, app->filename);

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
