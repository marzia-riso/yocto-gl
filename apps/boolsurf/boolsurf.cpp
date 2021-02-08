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
  app->test.points = app->points;
  for (auto& mesh_polygon : app->polygons) {
    app->test.polygons.push_back(mesh_polygon.points);
  }
  save_test(app->test, filename);
}

void init_from_test(app_state* app) {
  auto& points = app->points;
  points       = app->test.points;
  for (auto& polygon : app->test.polygons) {
    // Add new polygon to state.
    auto& mesh_polygon  = app->polygons.emplace_back();
    mesh_polygon.points = polygon;
    if (polygon.empty()) continue;

    // Draw polygon.
    for (int i = 0; i < polygon.size(); i++) {
      auto start = polygon[i];
      auto end   = polygon[(i + 1) % polygon.size()];
      auto path  = compute_geodesic_path(app->mesh, points[start], points[end]);
      draw_path(app->glscene, app->mesh, app->paths_material, path, 0.0005f);
      auto segments = mesh_segments(
          app->mesh.triangles, path.strip, path.lerps, path.start, path.end);
      update_mesh_polygon(mesh_polygon, segments);
    }
  }
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

void debug_draw(app_state* app, const vector<vec3i>& triangles,
    const vector<vec2f>& nodes, const vector<int>& indices,
    const vector<triangle_segment>& segments) {
  static int count = 0;
  auto&      t     = triangles;
  auto       n     = nodes;
  auto       is    = indices;
  //  auto       e     = vec2i{find_idx(is, edge_key.x), find_idx(is,
  //  edge_key.y)};

  auto base = app->test_filename;
  if (base == "") base = "data/tests/no-name.json";
  auto ext0 = ".triangulation" + to_string(count) + ".png";
  draw_triangulation(replace_extension(base, ext0), t, n);
  auto edges = vector<vec3i>{};
  for (auto& s : segments) {
    auto e = vec2i{find_idx(is, s.start_vertex), find_idx(is, s.end_vertex)};
    edges.push_back({e.x, e.y, e.y});
  }
  auto ext1 = ".edges" + to_string(count) + ".png";
  draw_triangulation(replace_extension(base, ext1), edges, n);

  save_test(app, "data/tests/crash.json");
  count += 1;
  assert(0);
}

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

  if (begin_header(widgets, "view")) {
    auto  glmaterial = app->mesh_material;
    auto& params     = app->drawgl_prms;
    draw_checkbox(widgets, "faceted", params.faceted);
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

  if (input.modifier_alt &&
      input.mouse_left.state == gui_button::state::releasing) {
    auto isec = intersect_shape(app, input);
    if (isec.hit) {
      if (app->polygons.size() == 1) app->polygons.push_back(mesh_polygon{});
      if (is_closed(app->polygons.back()))
        app->polygons.push_back(mesh_polygon{});

      auto& polygon = app->polygons.back();
      auto  point   = mesh_point{isec.element, isec.uv};

      if (polygon.points.size()) {
        auto& last_point = app->points[polygon.points.back()];
        if ((point.face == last_point.face) && (point.uv == last_point.uv))
          return;
      }

      app->points.push_back(point);
      polygon.points.push_back(app->points.size() - 1);

      draw_mesh_point(
          app->glscene, app->mesh, app->paths_material, point, 0.0008f);

      if (polygon.points.size() > 1) {
        auto geo_path = compute_path(polygon, app->points, app->mesh);
        auto instance = draw_path(
            app->glscene, app->mesh, app->paths_material, geo_path, 0.0005f);
        app->instances.push_back(instance);

        auto segments = mesh_segments(app->mesh.triangles, geo_path.strip,
            geo_path.lerps, geo_path.start, geo_path.end);

        update_mesh_polygon(polygon, segments);
      }
    }
  }
}

void do_the_thing(app_state* app) {
  // Hashgrid from triangle idx to <polygon idx, edge_idx, segment idx,
  // segment start uv, segment end uv> to handle intersections and
  // self-intersections

  struct hashgrid_segment {
    int polygon = -1;
    int segment = -1;

    vec2f start = {};
    vec2f end   = {};
  };

  struct intersection {
    int   vertex_id = -1;
    float lerp      = -1.0f;
  };

  auto hashgrid = unordered_map<int, vector<hashgrid_segment>>{};

  // Mappa segmento (polygon_id, segment_id) a lista di intersezioni.
  auto intersections = unordered_map<vec2i, vector<intersection>>{};

  // for (auto p = 0; p < app->polygons.size(); p++) {
  //   if (app->polygons[p].segments.size())
  //   app->polygons[p].segments.pop_back();
  // }

  // Riempiamo l'hashgrid con i segmenti per triangolo.
  for (auto p = 0; p < app->polygons.size(); p++) {
    auto& polygon = app->polygons[p];
    for (auto s = 0; s < polygon.segments.size(); s++) {
      auto& segment = polygon.segments[s];
      hashgrid[segment.face].push_back({p, s, segment.start, segment.end});
    }
  }

  // Per ogni faccia dell'hashgrid, calcoliamo le intersezioni fra i segmenti
  // contenuti.
  for (auto& [face, entries] : hashgrid) {
    for (auto i = 0; i < entries.size() - 1; i++) {
      auto& AB = entries[i];
      for (auto j = i + 1; j < entries.size(); j++) {
        auto& CD = entries[j];
        if (AB.polygon == CD.polygon &&
            yocto::abs(AB.segment - CD.segment) <= 1) {
          continue;
        }

        auto l = intersect_segments(AB.start, AB.end, CD.start, CD.end);
        if (l.x <= 0.0f || l.x >= 1.0f || l.y <= 0.0f || l.y >= 1.0f) continue;

        //        C
        //        |
        // A -- point -- B
        //        |
        //        D

        auto uv       = lerp(CD.start, CD.end, l.y);
        auto point    = mesh_point{face, uv};
        auto point_id = (int)app->points.size();
        app->points.push_back(point);

        auto vertex_id = (int)app->mesh.positions.size();
        auto pos       = eval_position(
            app->mesh.triangles, app->mesh.positions, point);
        app->mesh.positions.push_back(pos);

        intersections[{AB.polygon, AB.segment}].push_back({vertex_id, l.x});
        intersections[{CD.polygon, CD.segment}].push_back({vertex_id, l.y});
      }
    }
  }

  for (auto& [key, isecs] : intersections) {
    // Ordiniamo le intersezioni sulla lunghezza del segmento.
    sort(isecs.begin(), isecs.end(),
        [](auto& a, auto& b) { return a.lerp < b.lerp; });
  }

  // Mappa ogni faccia alla lista di triangle_segments di quella faccia.
  auto triangle_segments = unordered_map<int, vector<triangle_segment>>{};

  // Aggiungiamo gli estremi dei segmenti come vertici della mesh.
  // Dobbiamo inserire nei punti giusti anche i punti di intersezione che
  // spezzano i segmenti.
  // Inoltre popoliamo triangle_segments.
  for (auto polygon_id = 0; polygon_id < app->polygons.size(); polygon_id++) {
    auto& segments = app->polygons[polygon_id].segments;
    auto  first_id = (int)app->mesh.positions.size();

    // Per ogni segmento del poligono.
    for (auto segment_id = 0; segment_id < segments.size(); segment_id++) {
      auto& segment  = segments[segment_id];
      auto  start_uv = segment.start;
      auto  pos      = eval_position(
          app->mesh.triangles, app->mesh.positions, {segment.face, start_uv});

      // Aggiungiamo nuovo vertice alla mesh.
      auto start_vertex = (int)app->mesh.positions.size();
      app->mesh.positions.push_back(pos);

      // Se questo segmento aveva una o piu' interesezioni con altri segmenti...
      if (intersections.find({polygon_id, segment_id}) != intersections.end()) {
        auto& isecs = intersections[{polygon_id, segment_id}];

        // Popoliamo triangle_segments.
        for (auto& [end_vertex, l] : isecs) {
          auto end_uv = lerp(segment.start, segment.end, l);
          triangle_segments[segment.face].push_back(
              {polygon_id, start_vertex, end_vertex, start_uv, end_uv});

          // Accorcio il segmento corrente.
          start_uv     = end_uv;
          start_vertex = end_vertex;
        }
      }

      auto end_vertex = -1;

      if (segment_id == segments.size() - 1) {
        // Se e' l'ultimo vertice del poligono, gia' ce l'ho: e' il primo.
        end_vertex = first_id;
      } else {
        // L'indice del prossimo vertice che aggiungeremo al prossimo giro.
        end_vertex = (int)app->mesh.positions.size();
      }

      triangle_segments[segment.face].push_back(
          {polygon_id, start_vertex, end_vertex, start_uv, segment.end});
    }
  }

  // Adesso possiamo triangolare ogni faccia.

  //(marzia) collapse the triangle_segments iterations
  // Not now, they're useful while debugging

  auto debug_triangles = unordered_map<int, vector<vec3i>>{};
  auto debug_nodes     = unordered_map<int, vector<vec2f>>{};
  auto debug_indices   = unordered_map<int, vector<int>>{};

  // Mappa a ogni edge generato le due facce generate adiacenti.
  auto face_edgemap = unordered_map<vec2i, vec2i>{};
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
      auto edge          = make_edge_key({start_vertex, end_vertex});
      face_edgemap[edge] = {-1, -1};

      edges.push_back({edge_start, edge_end});
    }

    // printf("Segments: %d\n", segments.size());
    // printf("Edges: %d\n", edges.size());

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

    debug_nodes[face]     = nodes;
    debug_indices[face]   = indices;
    debug_triangles[face] = constrained_triangulation(nodes, edges);
    auto& triangles       = debug_triangles[face];

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
        // draw_triangulation("data/tests/debugging.png", triangles, nodes);
        debug_draw(app, triangles, nodes, indices, segments);
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
  }

  // Creaiamo inner_faces e outer_faces di ogni poligono.
  for (auto& [face, segments] : triangle_segments) {
    for (auto i = 0; i < segments.size(); i++) {
      auto& [polygon, start_vertex, end_vertex, start_vu, end_uv] = segments[i];
      auto edge     = vec2i{start_vertex, end_vertex};
      auto edge_key = make_edge_key(edge);

      auto faces = face_edgemap.at(edge_key);

      if (faces.x == -1 || faces.y == -1) {
        auto t  = debug_triangles[face];
        auto n  = debug_nodes[face];
        auto is = debug_indices[face];
        debug_draw(app, t, n, is, segments);
      }

      // Il triangolo di sinistra ha lo stesso orientamento del poligono.
      auto& [a, b, c] = app->mesh.triangles[faces.x];

      // Controlliamo che l'edge si nello stesso verso del poligono. Se non
      // e' cosi, invertiamo.
      if ((edge == vec2i{b, a}) || (edge == vec2i{c, b}) ||
          (edge == vec2i{a, c})) {
        swap(faces.x, faces.y);
      }

      if (faces.x != -1) app->polygons[polygon].inner_faces.push_back(faces.x);
      if (faces.y != -1) app->polygons[polygon].outer_faces.push_back(faces.y);
    }
  }

  // Removing face duplicates
  for (auto i = 1; i < app->polygons.size(); i++) {
    auto& inner = app->polygons[i].inner_faces;
    auto& outer = app->polygons[i].outer_faces;

    sort(inner.begin(), inner.end());
    inner.erase(unique(inner.begin(), inner.end()), inner.end());

    sort(outer.begin(), outer.end());
    outer.erase(unique(outer.begin(), outer.end()), outer.end());
  }

  app->mesh.normals = compute_normals(app->mesh.triangles, app->mesh.positions);
  app->mesh.adjacencies = face_adjacencies(app->mesh.triangles);
  set_positions(app->mesh_instance->shape, app->mesh.positions);
  set_triangles(app->mesh_instance->shape, app->mesh.triangles);
  set_normals(app->mesh_instance->shape, app->mesh.normals);
  init_edges_and_vertices_shapes_and_points(app);
  app->mesh_instance->hidden = true;

  auto tags          = compute_face_tags(app->mesh, app->polygons);
  auto face_polygons = unordered_map<int, vector<int>>();

  auto cells      = vector<vector<int>>();
  auto cell_faces = unordered_map<int, vector<int>>();

  for (auto p = 1; p < app->polygons.size(); p++) {
    auto check = [&](int face, int polygon) {
      return find_in_vec(tags[face], polygon) == -1;
    };

    auto start_out   = app->polygons[p].outer_faces;
    auto visited_out = flood_fill(app->mesh, start_out, -p, check);
    for (auto o : visited_out) face_polygons[o].push_back(-p);

    auto start_in   = app->polygons[p].inner_faces;
    auto visited_in = flood_fill(app->mesh, start_in, p, check);
    for (auto i : visited_in) face_polygons[i].push_back(p);
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

    auto color = app->cell_materials[i + 1]->color;
    add_patch_shape(app, cell_faces[i], color, 0.00025f * (i + 1));
  }

  // Previous Implementation
  // auto graph = compute_graph(
  //     app->points.size(), edge_map, counterclockwise);
  // // print_graph(graph);

  // auto edge_info  = compute_edge_info(edge_map, app->polygons);
  // auto components = compute_connected_components(graph);

  // for (auto& component : components) {
  //   auto cells = compute_cells(
  //       component, app->points, app->mesh, app->polygons.size());
  //   // draw_arrangement(app->glscene, app->mesh, app->cell_materials,
  //   //     app->points, arrangement);

  //   print_graph(component);
  //   print_cells(cells);

  //   auto dual_graph = compute_dual_graph(cells, edge_info);
  //   // print_dual_graph(dual_graph);
  //   auto outer_face = compute_outer_face(dual_graph);

  //   visit_dual_graph(dual_graph, cells, outer_face);
  // }

  // Boolean operation example
  // auto ids = vector<int>(arrangement.size(), 0);
  // for (auto i = 0; i < arrangement.size(); i++)
  //   if (!arrangement[i].embedding[1]) ids[i] = 1;

  // polygon_and(arrangement, ids, 0);
  // // polygon_or(arrangement, ids, 2);

  // auto result = vector<cell_polygon>();
  // for (auto i = 0; i < ids.size(); i++) {
  //   if (ids[i]) {
  //     result.push_back(arrangement[i]);
  //     for (auto a : arrangement[i].points) printf("%d ", a);
  //   }
  //   printf("\n");
  // }

  // draw_arrangement(
  //     app->glscene, app->mesh, app->cell_materials, app->points,
  //     result);
}

void key_input(app_state* app, const gui_input& input) {
  for (auto idx = 0; idx < input.key_buttons.size(); idx++) {
    auto button = input.key_buttons[idx];
    if (button.state != gui_button::state::pressing) continue;

    switch (idx) {
      case (int)gui_key('I'): {
        do_the_thing(app);
      } break;
        // case (int)gui_key('L'): {
        //   for (auto& [_, patches] : app->patch_in)
        //     for (auto p : patches)
        //       app->glscene->instances[p]->hidden =
        //           !app->glscene->instances[p]->hidden;
        // } break;
        // case (int)gui_key('R'): {
        //   for (auto& [_, patches] : app->patch_out)
        //     for (auto p : patches)
        //       app->glscene->instances[p]->hidden =
        //           !app->glscene->instances[p]->hidden;
        // } break;
        // case (int)gui_key('N'): {
        //   for (auto& [key, patches] : app->patch_in)
        //     for (auto p : patches)
        //       app->glscene->instances[p]->hidden =
        //           (key == app->current_polygon) ? false : true;

        //   for (auto& [key, patches] : app->patch_out)
        //     for (auto p : patches)
        //       app->glscene->instances[p]->hidden =
        //           (key == app->current_polygon) ? false : true;

        //   app->current_polygon = (app->current_polygon + 1) %
        //                          app->polygons.size();
        // } break;

      case (int)gui_key('C'): {
        auto old_camera = app->glcamera;
        app->points.clear();
        app->polygons.clear();
        app->polygons.push_back(mesh_polygon{});
        load_shape(app, app->filename);
        clear_scene(app->glscene);
        init_glscene(app, app->glscene, app->mesh, {});
        app->glcamera = old_camera;
      } break;

      case (int)gui_key::enter: {
        auto& polygon = app->polygons.back();
        if (polygon.points.size() < 3 || is_closed(polygon)) return;

        auto point = polygon.points.front();
        polygon.points.push_back(point);

        auto geo_path = compute_path(polygon, app->points, app->mesh);
        draw_path(
            app->glscene, app->mesh, app->paths_material, geo_path, 0.0005f);

        auto segments = mesh_segments(app->mesh.triangles, geo_path.strip,
            geo_path.lerps, geo_path.start, geo_path.end);

        update_mesh_polygon(polygon, segments);

        break;
      }
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
  window->msaa     = 8;

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

  if (app->test_filename != "") {
    init_from_test(app);
  }

  run_ui(window, update_app);

  // clear
  clear_scene(app->glscene);

  // done
  return 0;
}
