#include "boolsurf.h"

#include <cassert>
#include <deque>

#include "ext/CDT/CDT/include/CDT.h"

constexpr auto adjacent_to_nothing = -2;

// Build adjacencies between faces (sorted counter-clockwise)
static vector<vec3i> face_adjacencies_fast(const vector<vec3i>& triangles) {
  auto get_edge = [](const vec3i& triangle, int i) -> vec2i {
    auto x = triangle[i], y = triangle[i < 2 ? i + 1 : 0];
    return x < y ? vec2i{x, y} : vec2i{y, x};
  };

  auto adjacencies = vector<vec3i>{triangles.size(), vec3i{-1, -1, -1}};
  auto edge_map    = hash_map<vec2i, int>();
  edge_map.reserve((size_t)(triangles.size() * 1.5));
  for (int i = 0; i < triangles.size(); ++i) {
    for (int k = 0; k < 3; ++k) {
      auto edge = get_edge(triangles[i], k);
      auto it   = edge_map.find(edge);
      if (it == edge_map.end()) {
        edge_map[edge] = i;
      } else {
        auto neighbor     = it->second;
        adjacencies[i][k] = neighbor;
        for (int kk = 0; kk < 3; ++kk) {
          auto edge2 = get_edge(triangles[neighbor], kk);
          if (edge2 == edge) {
            adjacencies[neighbor][kk] = i;
            break;
          }
        }
      }
    }
  }
  return adjacencies;
}

void init_mesh(bool_mesh& mesh) {
  if (mesh.quads.size()) {
    mesh.triangles = quads_to_triangles(mesh.quads);
    mesh.quads.clear();
  }

  mesh.normals       = compute_normals(mesh);
  mesh.adjacencies   = face_adjacencies_fast(mesh.triangles);
  mesh.num_triangles = (int)mesh.triangles.size();
  mesh.num_positions = (int)mesh.positions.size();

  // Fit shape in [-1, +1]^3
  auto bbox = invalidb3f;
  for (auto& pos : mesh.positions) bbox = merge(bbox, pos);
  for (auto& pos : mesh.positions) pos = (pos - center(bbox)) / max(size(bbox));

  mesh.bbox     = bbox;
  mesh.bbox.min = (mesh.bbox.min - center(bbox)) / max(size(bbox));
  mesh.bbox.max = (mesh.bbox.max - center(bbox)) / max(size(bbox));

  mesh.bvh = make_triangles_bvh(mesh.triangles, mesh.positions, {});

  mesh.dual_solver = make_dual_geodesic_solver(
      mesh.triangles, mesh.positions, mesh.adjacencies);
}

void reset_mesh(bool_mesh& mesh) {
  mesh.triangles.resize(mesh.num_triangles);
  mesh.positions.resize(mesh.num_positions);
  mesh.adjacencies.resize(mesh.num_triangles);
  mesh.dual_solver.graph.resize(mesh.num_triangles);
  mesh.triangulated_faces.clear();

  auto get_triangle_center = [](const vector<vec3i>&  triangles,
                                 const vector<vec3f>& positions,
                                 int                  face) -> vec3f {
    vec3f pos[3] = {positions[triangles[face].x], positions[triangles[face].y],
        positions[triangles[face].z]};
    auto  l0     = length(pos[0] - pos[1]);
    auto  p0     = (pos[0] + pos[1]) / 2;
    auto  l1     = length(pos[1] - pos[2]);
    auto  p1     = (pos[1] + pos[2]) / 2;
    auto  l2     = length(pos[2] - pos[0]);
    auto  p2     = (pos[2] + pos[0]) / 2;
    return (l0 * p0 + l1 * p1 + l2 * p2) / (l0 + l1 + l2);
  };

  for (auto& [face, _] : mesh.triangulated_faces) {
    for (int k = 0; k < 3; k++) {
      auto neighbor = mesh.adjacencies[face][k];
      if (neighbor == -1) continue;
      auto kk = find_adjacent_triangle(
          mesh.triangles[neighbor], mesh.triangles[face]);

      // Fix adjacencies and dual_solver.
      mesh.adjacencies[neighbor][kk]              = face;
      mesh.dual_solver.graph[neighbor][kk].node   = face;
      mesh.dual_solver.graph[neighbor][kk].length = length(
          get_triangle_center(mesh.triangles, mesh.positions, neighbor) -
          get_triangle_center(mesh.triangles, mesh.positions, face));
    }
  }
}

geodesic_path compute_geodesic_path(
    const bool_mesh& mesh, const mesh_point& start, const mesh_point& end) {
  auto path = geodesic_path{};
  if (start.face == end.face) {
    path.start = start;
    path.end   = end;
    path.strip = {start.face};
    return path;
  }

  auto strip = compute_strip(
      mesh.dual_solver, mesh.triangles, mesh.positions, end, start);
  path = shortest_path(
      mesh.triangles, mesh.positions, mesh.adjacencies, start, end, strip);
  return path;
}

vector<mesh_segment> mesh_segments(const vector<vec3i>& triangles,
    const vector<int>& strip, const vector<float>& lerps,
    const mesh_point& start, const mesh_point& end) {
  auto result = vector<mesh_segment>{};
  result.reserve(strip.size());

  for (int i = 0; i < strip.size(); ++i) {
    vec2f start_uv;
    if (i == 0) {
      start_uv = start.uv;
    } else {
      vec2f uvw[3] = {{0, 0}, {1, 0}, {0, 1}};
      auto  k      = find_adjacent_triangle(
          triangles[strip[i]], triangles[strip[i - 1]]);
      auto a   = uvw[mod3(k)];
      auto b   = uvw[mod3(k + 1)];
      start_uv = lerp(a, b, 1 - lerps[i - 1]);
    }

    vec2f end_uv;
    if (i == strip.size() - 1) {
      end_uv = end.uv;
    } else {
      vec2f uvw[3] = {{0, 0}, {1, 0}, {0, 1}};
      auto  k      = find_adjacent_triangle(
          triangles[strip[i]], triangles[strip[i + 1]]);
      auto a = uvw[k];
      auto b = uvw[mod3(k + 1)];
      end_uv = lerp(a, b, lerps[i]);
    }
    if (start_uv == end_uv) continue;
    result.push_back({start_uv, end_uv, strip[i]});
  }
  return result;
}

void recompute_polygon_segments(const bool_mesh& mesh, const bool_state& state,
    mesh_polygon& polygon, int index) {
  if (index > 0) {
    auto& last_segment = polygon.edges.back();
    polygon.length -= last_segment.size();
    polygon.edges.pop_back();
  }

  auto faces = hash_set<int>();
  for (int i = index; i < polygon.points.size(); i++) {
    auto start = polygon.points[i];
    faces.insert(state.points[start].face);
    auto end  = polygon.points[(i + 1) % polygon.points.size()];
    auto path = compute_geodesic_path(
        mesh, state.points[start], state.points[end]);
    auto threshold = 0.001f;
    for (auto& l : path.lerps) {
      l = yocto::clamp(l, 0 + threshold, 1 - threshold);
    }
    auto segments = mesh_segments(
        mesh.triangles, path.strip, path.lerps, path.start, path.end);

    polygon.edges.push_back(segments);
    polygon.length += segments.size();
  }

  polygon.is_contained_in_single_face = (faces.size() == 1);
}

struct hashgrid_polyline {
  int           polygon  = -1;
  vector<vec2f> points   = {};
  vector<int>   vertices = {};

  bool is_closed = false;
};

inline int num_segments(const hashgrid_polyline& polyline) {
  if (polyline.is_closed) return (int)polyline.points.size();
  return (int)polyline.points.size() - 1;
}

inline pair<vec2f, vec2f> get_segment(
    const hashgrid_polyline& polyline, int i) {
  if (polyline.is_closed) {
    return {
        polyline.points[i], polyline.points[(i + 1) % polyline.points.size()]};
  } else {
    return {polyline.points[i], polyline.points[i + 1]};
  }
}

inline vec2i get_segment_vertices(const hashgrid_polyline& polyline, int i) {
  if (polyline.is_closed) {
    return {polyline.vertices[i],
        polyline.vertices[(i + 1) % polyline.vertices.size()]};
  } else {
    return {polyline.vertices[i], polyline.vertices[i + 1]};
  }
}

using mesh_hashgrid = hash_map<int, vector<hashgrid_polyline>>;

inline int add_vertex(bool_mesh& mesh, mesh_hashgrid& hashgrid,
    const mesh_point& point, int polyline_id, int vertex = -1) {
  float eps             = 0.00001;
  auto  update_polyline = [&](int v) {
    if (polyline_id < 0) return;
    auto& polyline = hashgrid[point.face][polyline_id];
    polyline.vertices.push_back(v);
    polyline.points.push_back(point.uv);
  };

  {  // Maybe collapse with original mesh vertices.
    auto uv = point.uv;
    auto tr = mesh.triangles[point.face];
    if (uv.x < eps && uv.y < eps) {
      update_polyline(tr.x);
      return tr.x;
    }
    if (uv.x > 1 - eps && uv.y < eps) {
      update_polyline(tr.y);
      return tr.y;
    }
    if (uv.y > 1 - eps && uv.x < eps) {
      update_polyline(tr.z);
      return tr.z;
    }
  }

  {  // Maybe collapse with already added vertices.
    auto& polylines = hashgrid[point.face];
    for (auto& polyline : polylines) {
      for (int i = 0; i < polyline.vertices.size(); i++) {
        if (length(point.uv - polyline.points[i]) < eps) {
          update_polyline(polyline.vertices[i]);
          return polyline.vertices[i];
        }
      }
    }
  }

  // No collapse. Add new vertex to mesh.
  if (vertex == -1) {
    vertex   = (int)mesh.positions.size();
    auto pos = eval_position(mesh.triangles, mesh.positions, point);
    mesh.positions.push_back(pos);
  }

  update_polyline(vertex);
  return vertex;
}

static mesh_hashgrid compute_hashgrid(bool_mesh& mesh,
    const vector<mesh_polygon>& polygons, hash_map<int, int>& control_points) {
  // La hashgrid associa ad ogni faccia una lista di polilinee.
  // Ogni polilinea è definita da una sequenza punti in coordinate
  // baricentriche, ognuno di essi assiociato al corrispondente vertice della
  // mesh.
  auto hashgrid = mesh_hashgrid{};

  for (auto polygon_id = 0; polygon_id < polygons.size(); polygon_id++) {
    auto& polygon = polygons[polygon_id];
    if (polygon.length == 0) continue;

    // La polilinea della prima faccia del poligono viene processata alla fine
    // (perché si trova tra il primo e l'ultimo edge)
    int  first_face   = polygon.edges[0][0].face;
    int  first_vertex = -1;
    auto indices      = vec2i{-1, -1};  // edge_id, segment_id

    int last_face   = -1;
    int last_vertex = -1;

    for (auto e = 0; e < polygon.edges.size(); e++) {
      auto& edge = polygon.edges[e];

      for (auto s = 0; s < edge.size(); s++) {
        auto& segment = edge[s];

        // Iniziamo a riempire l'hashgrid a partire da quando troviamo una
        // faccia diversa da quella iniziale del poligono (il primo tratto
        // verrà aggiunto a posteriori per evitare inconsistenza)
        if (segment.face == first_face && indices == vec2i{-1, -1}) continue;
        if (indices == vec2i{-1, -1}) indices = {e, s};

        auto& entry = hashgrid[segment.face];
        auto  ids   = vec2i{e, s};
        ids.y       = (s + 1) % edge.size();
        ids.x       = ids.y > s ? e : (e + 1) % polygon.edges.size();

        // Se la faccia del segmento che stiamo processando è diversa
        // dall'ultima salvata allora creiamo una nuova polilinea, altrimenti
        // accodiamo le nuove informazioni.
        if (segment.face != last_face) {
          auto  polyline_id = (int)entry.size();
          auto& polyline    = entry.emplace_back();
          polyline.polygon  = polygon_id;

          last_vertex = add_vertex(mesh, hashgrid,
              {segment.face, segment.start}, polyline_id, last_vertex);
          if (first_vertex == -1) first_vertex = last_vertex;

          last_vertex = add_vertex(
              mesh, hashgrid, {segment.face, segment.end}, polyline_id);

        } else {
          auto  polyline_id = (int)entry.size() - 1;
          auto& polyline    = entry.back();
          assert(segment.end != polyline.points.back());

          last_vertex = add_vertex(
              mesh, hashgrid, {segment.face, segment.end}, polyline_id);
        }

        last_face = segment.face;
      }

      if (last_vertex != -1)
        control_points[last_vertex] =
            polygon.points[(e + 1) % polygon.edges.size()];
    }

    if (indices == vec2i{-1, -1}) {
      auto& entry        = hashgrid[first_face];
      auto  polyline_id  = (int)entry.size();
      auto& polyline     = entry.emplace_back();
      polyline.polygon   = polygon_id;
      polyline.is_closed = true;

      for (auto e = 0; e < polygon.edges.size(); e++) {
        auto& edge = polygon.edges[e];
        for (int s = 0; s < edge.size(); s++) {
          auto& segment = edge[s];

          last_vertex = add_vertex(
              mesh, hashgrid, {segment.face, segment.start}, polyline_id);
        }

        if (last_vertex != -1)
          control_points[last_vertex] =
              polygon.points[(e + 1) % polygon.edges.size()];
      }
    };

    // Ripetiamo parte del ciclo (fino a indices) perché il primo tratto di
    // polilinea non è stato inserito nell'hashgrid
    auto vertex = -1;
    for (auto e = 0; e <= indices.x; e++) {
      auto end_idx = (e < indices.x) ? polygon.edges[e].size() : indices.y;
      for (auto s = 0; s < end_idx; s++) {
        auto ids = vec2i{e, s};
        ids.y    = (s + 1) % polygon.edges[e].size();
        ids.x    = ids.y > s ? e : e + 1;

        auto& segment     = polygon.edges[e][s];
        auto& entry       = hashgrid[segment.face];
        auto  polyline_id = (int)entry.size() - 1;

        if (e == indices.x && s == indices.y - 1) vertex = first_vertex;

        // auto& polyline    = entry.back();
        assert(segment.face == last_face);
        last_vertex = add_vertex(
            mesh, hashgrid, {segment.face, segment.end}, polyline_id, vertex);
      }

      if (e > 0 && last_vertex != -1)
        control_points[last_vertex] =
            polygon.points[(e + 1) % polygon.edges.size()];
    }
  }
  return hashgrid;
}

static hash_map<int, int> compute_control_points(vector<mesh_polygon>& polygons,
    const vector<vector<vector<int>>> vertices) {
  auto control_points = hash_map<int, int>();
  for (auto p = 0; p < vertices.size(); p++) {
    for (auto e = 0; e < vertices[p].size(); e++) {
      auto control_point_idx            = vertices[p][e][0];
      auto mesh_point_idx               = polygons[p].points[e];
      control_points[control_point_idx] = mesh_point_idx;
    }
  }
  return control_points;
}

// TODO(giacomo): CAMBIAMI NOME
static vector<mesh_cell> flood_fill_new(vector<int>& starts,
    const vector<vec3i>& adjacencies, const vector<vec3i>& border_tags,
    int num_polygons) {
  auto result    = vector<mesh_cell>{};
  auto cell_tags = vector<int>(adjacencies.size(), -1);

  // consume task stack
  while (starts.size()) {
    // pop element from task stack
    auto first_face = starts.back();
    starts.pop_back();

    if (cell_tags[first_face] >= 0) {
      continue;
    }

    auto  cell_id = (int)result.size();
    auto& cell    = result.emplace_back();
    cell.faces.reserve(adjacencies.size());
    auto face_stack = vector<int>{first_face};

    while (!face_stack.empty()) {
      auto face = face_stack.back();
      face_stack.pop_back();

      if (cell_tags[face] >= 0) continue;
      cell_tags[face] = cell_id;

      cell.faces.push_back(face);

      for (int k = 0; k < 3; k++) {
        auto neighbor = adjacencies[face][k];
        if (neighbor < 0) continue;
        auto p = border_tags[face][k];

        auto neighbor_cell = cell_tags[neighbor];
        if (neighbor_cell >= 0 && p != 0) {
          // La faccia neighbor e' gia' stata visitata.
          if (neighbor_cell == cell_id) {
            // Sto visitando la stessa cella.
            if (find_in_vec(border_tags[neighbor], -p) != -1) {
              // Sto attraversando il bordo di un poligono, quindi
              // connetto la cella a se stessa.
              cell.adjacency.insert({cell_id, +p});
              cell.adjacency.insert({cell_id, -p});

            } else {
              continue;
            }
          } else {
            // Non sto visitando la stessa cella.
            if (p > 0) {
              // Sto entrando nel poligono p.
              cell.adjacency.insert({neighbor_cell, +p});

              if (p < num_polygons)
                result[neighbor_cell].adjacency.insert({cell_id, -p});
            } else {
              // Sto uscendo dal poligono p.
              result[neighbor_cell].adjacency.insert({cell_id, -p});

              if (-p < num_polygons) {
                cell.adjacency.insert({neighbor_cell, +p});
              }
            }
          }
        } else {
          // La faccia neighbor non e' mai stata visitata.
          if (p == 0) {
            // Non sto attraversando il bordo del poligono p.
            face_stack.push_back(neighbor);
          } else {
            // Sto attraversando il bordo del poligono p.
            starts.push_back(neighbor);
          }
        }
      }
    }  // end of while
    cell.faces.shrink_to_fit();
  }  // end of while

  return result;
}

inline vector<mesh_cell> make_mesh_cells(
    const vector<vec3i>& adjacencies, const bool_borders& borders) {
  // Iniziamo dall'ultima faccia che sicuramente non e' stata distrutta.
  auto starts = vector<int>{(int)adjacencies.size() - 1};
  auto result = flood_fill_new(
      starts, adjacencies, borders.tags, borders.num_polygons);
  return result;
}

static vector<int> find_roots(const vector<mesh_cell>& cells) {
  // Trova le celle non hanno archi entranti con segno di poligono positivo.
  auto adjacency = vector<int>(cells.size(), 0);
  for (auto& cell : cells) {
    for (auto& [adj, p] : cell.adjacency) {
      if (p > 0) adjacency[adj] += 1;
    }
  }

  auto result = vector<int>{};
  for (int i = 0; i < adjacency.size(); i++) {
    if (adjacency[i] == 0) result.push_back(i);
  }
  return result;
}

static void compute_cycles(const vector<mesh_cell>& cells, int node,
    vec2i parent, vector<int>& visited, vector<vec2i>& parents,
    vector<vector<vec2i>>& cycles) {
  // Se il nodo il considerazione è già stato completamente visitato allora
  // terminiamo la visita
  if (visited[node] == 2) return;

  // Se il nodo in considerazione non è stato completamente visitato e lo
  // stiamo rivisitando ora allora abbiamo trovato un ciclo
  if (visited[node] == 1) {
    auto  cycle   = vector<vec2i>();
    auto& current = parent;
    cycle.push_back(current);

    // Risalgo l'albero della visita fino a che non trovo lo stesso nodo e
    // salvo il ciclo individuato
    while (current.x != node) {
      auto prev = parents[current.x];

      // (marzia) check: è vero che ho un ciclo corretto se il verso
      // (entrante/uscente) è lo stesso per tutti gli archi?
      if (sign(prev.y) != sign(current.y)) return;
      current = prev;
      cycle.push_back(current);
    }

    cycles.push_back(cycle);
    return;
  }

  // Settiamo il padre del nodo attuale e iniziamo ad esplorare i suoi vicini
  parents[node] = parent;
  visited[node] = 1;

  for (auto& [neighbor, polygon] : cells[node].adjacency) {
    // Se stiamo percorrendo lo stesso arco ma al contrario allora continuo,
    // altrimenti esploriamo il vicino
    if (polygon > 0) continue;
    // if (neighbor == parent.x && polygon == -parent.y) continue;
    compute_cycles(cells, neighbor, {node, -polygon}, visited, parents, cycles);
  }

  // Settiamo il nodo attuale come completamente visitato
  visited[node] = 2;
}

inline vector<vector<vec2i>> compute_graph_cycles(
    const vector<mesh_cell>& cells) {
  auto visited        = vector<int>(cells.size(), 0);
  auto parents        = vector<vec2i>(cells.size());
  auto cycles         = vector<vector<vec2i>>();
  auto start_node     = 0;
  auto invalid_parent = vec2i{-1, -1};
  compute_cycles(cells, start_node, invalid_parent, visited, parents, cycles);
  return cycles;
}

inline vector<vector<int>> compute_components(
    const bool_state& state, const mesh_shape& shape) {
  // Calcoliamo le componenti tra le celle presenti in una shape
  // (per calcolarne i bordi in maniera più semplice)
  auto cells   = vector<int>(shape.cells.begin(), shape.cells.end());
  auto visited = hash_map<int, bool>();
  for (auto cell : cells) visited[cell] = false;

  auto components = vector<vector<int>>();

  for (auto cell : cells) {
    if (visited[cell]) continue;

    auto& component = components.emplace_back();

    auto stack = vector<int>();
    stack.push_back(cell);

    while (!stack.empty()) {
      auto cell_idx = stack.back();
      stack.pop_back();

      if (visited[cell_idx]) continue;
      visited[cell_idx] = true;
      component.push_back(cell_idx);

      auto& cell = state.cells[cell_idx];
      for (auto& [neighbor, _] : cell.adjacency) {
        if (find_idx(cells, neighbor) == -1) continue;
        if (visited[neighbor]) continue;
        stack.push_back(neighbor);
      }
    }
  }
  return components;
}

void save_tree_png(const bool_state& state, string filename,
    const string& extra, bool color_shapes);

static vector<vector<int>> propagate_cell_labels(const vector<mesh_cell>& cells,
    const vector<int>& start, const vector<vector<vec2i>>& cycles,
    const hash_set<int>& skip_polygons, int num_polygons) {
  // Inizializziamo le label delle celle a 0.
  auto labels = vector<vector<int>>(cells.size(), vector<int>(num_polygons, 0));

  // Inizializza la label dei nodi nei cicli.
  for (auto& cycle : cycles) {
    for (auto& c : cycle) labels[c.x][c.y] = 1;
  }
  // Calcoliamo le label delle celle visitando il grafo di adiacenza a
  // partire dalle celle ambiente e incrementanto/decrementanto l'indice
  // corrispondente al poligono.

  auto queue   = deque<int>(start.begin(), start.end());
  auto visited = vector<bool>(cells.size(), false);
  for (auto& s : start) visited[s] = true;

  while (!queue.empty()) {
    // print("queue", queue);
    auto cell_id = queue.front();
    queue.pop_front();

    auto& cell = cells[cell_id];

    for (auto& [neighbor, polygon] : cell.adjacency) {
      auto polygon_unsigned = uint(yocto::abs(polygon));

      auto is_cycle_edge = contains(skip_polygons, (int)polygon_unsigned);

      if (polygon < 0 && visited[neighbor]) continue;

      // Se il nodo è già stato visitato e la nuova etichetta è diversa da
      // quella già calcolata allora prendo il massimo valore in ogni
      // componente

      auto& neighbor_labels = labels[neighbor];
      auto  cell_labels     = labels[cell_id];
      cell_labels[polygon_unsigned] += sign(polygon);

      auto updated_neighbor_labels = false;
      for (int i = 0; i < neighbor_labels.size(); i++) {
        if (is_cycle_edge) {
          if (contains(skip_polygons, i)) {
            continue;
          }
        }

        if (neighbor_labels[i] == null_label) {
          neighbor_labels[i]      = cell_labels[i];
          updated_neighbor_labels = true;
          continue;
        }

        if (cell_labels[i] > neighbor_labels[i]) {
          neighbor_labels[i]      = cell_labels[i];
          updated_neighbor_labels = true;
        }
      }

      if (updated_neighbor_labels) {
        if (!contains(queue, neighbor)) {
          // printf("add: %d\n", neighbor);
          queue.push_back(neighbor);
        }
      }
      visited[neighbor] = true;
    }
  }
  return labels;
}

static void add_polygon_intersection_points(bool_state& state,
    hash_map<int, vector<hashgrid_polyline>>& hashgrid, bool_mesh& mesh) {
  // Calcoliamo sia le intersezioni che le self-intersections, aggiungendo i
  // vertici nuovi alla mesh.

  for (auto& [face, polylines] : hashgrid) {
    // Check for polyline self interesctions
    for (auto p0 = 0; p0 < polylines.size(); p0++) {
      auto& poly = polylines[p0];

      for (int s0 = 0; s0 < num_segments(poly) - 1; s0++) {
        auto [start0, end0] = get_segment(poly, s0);
        int num_added       = 0;

        for (int s1 = s0 + 1; s1 < num_segments(poly); s1++) {
          // Skip adjacent segments.
          if (poly.is_closed) {
            if (yocto::abs(s0 - s1) % num_segments(poly) <= 1) continue;
          } else {
            if (yocto::abs(s0 - s1) <= 1) continue;
          }

          auto [start1, end1] = get_segment(poly, s1);

          auto l = intersect_segments(start0, end0, start1, end1);
          if (l.x <= 0.0f || l.x >= 1.0f || l.y <= 0.0f || l.y >= 1.0f) {
            continue;
          }

          auto uv                      = lerp(start1, end1, l.y);
          auto point                   = mesh_point{face, uv};
          auto vertex                  = add_vertex(mesh, hashgrid, point, -1);
          state.control_points[vertex] = state.points.size();
          state.isecs_generators[vertex] = {poly.polygon, poly.polygon};

          state.points.push_back(point);

          insert(poly.points, s0 + 1, uv);
          insert(poly.vertices, s0 + 1, vertex);
          insert(poly.points, s1 + 2, uv);
          insert(poly.vertices, s1 + 2, vertex);
          num_added += 1;
          s1 += 2;
        }
        s0 += num_added;
      }
    }

    // Check for intersections between different polylines
    for (auto p0 = 0; p0 < polylines.size() - 1; p0++) {
      for (auto p1 = p0 + 1; p1 < polylines.size(); p1++) {
        auto& poly0 = polylines[p0];
        auto& poly1 = polylines[p1];
        for (int s0 = 0; s0 < num_segments(poly0); s0++) {
          auto [start0, end0] = get_segment(poly0, s0);
          int num_added       = 0;

          for (int s1 = 0; s1 < num_segments(poly1); s1++) {
            auto [start1, end1] = get_segment(poly1, s1);

            auto l = intersect_segments(start0, end0, start1, end1);
            if (l.x <= 0.0f || l.x >= 1.0f || l.y <= 0.0f || l.y >= 1.0f) {
              continue;
            }

            auto uv     = lerp(start1, end1, l.y);
            auto point  = mesh_point{face, uv};
            auto vertex = add_vertex(mesh, hashgrid, point, -1);
            state.control_points[vertex]   = state.points.size();
            state.isecs_generators[vertex] = {poly0.polygon, poly1.polygon};

            state.points.push_back(point);

            insert(poly0.points, s0 + 1, uv);
            insert(poly0.vertices, s0 + 1, vertex);
            insert(poly1.points, s1 + 1, uv);
            insert(poly1.vertices, s1 + 1, vertex);
            num_added += 1;
            s1 += 1;
          }
          s0 += num_added;
        }
      }
    }
  }
}

triangulation_info compute_triangulation_constraints(const bool_mesh& mesh,
    int face, const vector<hashgrid_polyline>& polylines) {
  auto info    = triangulation_info{};
  info.face    = face;
  info.nodes   = vector<vec2f>{{0, 0}, {1, 0}, {0, 1}};
  info.indices = vector<int>(
      &mesh.triangles[face][0], &mesh.triangles[face][3]);

  for (auto& polyline : polylines) {
    // Per ogni segmento della polyline, aggiorniamo triangulation_info,
    // aggiungendo nodi, indici, e edge constraints.
    for (auto i = 0; i < num_segments(polyline); i++) {
      vec2f uvs[2];
      tie(uvs[0], uvs[1]) = get_segment(polyline, i);
      auto vertices       = get_segment_vertices(polyline, i);
      assert(vertices[0] < mesh.positions.size());
      assert(vertices[1] < mesh.positions.size());

      // TODO(giacomo): Set to -1 or 'invalid'.
      auto local_vertices = vec2i{-7, -8};
      for (int k = 0; k < 2; k++) {
        local_vertices[k] = find_idx(info.indices, vertices[k]);
        if (local_vertices[k] == -1) {
          info.indices.push_back(vertices[k]);
          info.nodes.push_back(uvs[k]);
          local_vertices[k] = (int)info.indices.size() - 1;
        }
      }

      // Aggiungiamo l'edge ai constraints della triangolazione.
      info.edges.push_back({local_vertices[0], local_vertices[1]});

      // Extra: Se i nodi sono su un lato k != -1 di un triangolo allora li
      // salviamo nella edgemap.
      for (int j = 0; j < 2; j++) {
        auto [k, lerp] = get_edge_lerp_from_uv(uvs[j]);
        if (k != -1) {
          info.edgemap[k].push_back({local_vertices[j], lerp});
        }
      }
    }
  }
  return info;
}

void add_boundary_edge_constraints(
    array<vector<pair<int, float>>, 3>& edgemap, vector<vec2i>& edges) {
  // Aggiungiamo gli edge di vincolo per i lati del triangolo.
  for (int k = 0; k < 3; k++) {
    auto  edge        = get_triangle_edge_from_index(k);
    auto& edge_points = edgemap[k];

    // Se sul lato non ci sono punti aggiuntivi, allora lo aggiungiamo ai
    // vincoli cosi' come e'.
    if (edge_points.empty()) {
      edges.push_back(edge);
      continue;
    }

    // Ordiniamo i punti che giacciono sul lato per lerp crescente.
    if (edge_points.size() > 1) {
      sort(edge_points.begin(), edge_points.end(),
          [](auto& a, auto& b) { return a.second < b.second; });
    }

    // Creiamo primo e ultimo vincolo di questo lato.
    edges.push_back({edge.x, edge_points[0].first});
    edges.push_back({edge.y, edge_points.back().first});

    // Creiamo i rimanenti vincoli contenuti nel lato.
    for (auto i = 0; i < edge_points.size() - 1; i++) {
      edges.push_back({edge_points[i].first, edge_points[i + 1].first});
    }
  }
}

static pair<vector<vec3i>, vector<vec3i>> single_split_triangulation(
    const vector<vec2f>& nodes, const vec2i& edge) {
  // Calcoliamo la triangolazione con un singolo segmento all'interno del
  // triangolo.
  auto start_edge = get_edge_from_uv(nodes[edge.x]);
  auto end_edge   = get_edge_from_uv(nodes[edge.y]);

  auto triangles   = vector<vec3i>(3);
  auto adjacencies = vector<vec3i>(3);

  // image: libs/boolsurf/notes/sinlge-split-adjacency.jpg
  if (edge.x < 3) {
    // Se il segmento ha come inizio un punto in un lato e come fine il
    // vertice del triangolo opposto
    triangles[0] = {edge.x, end_edge.x, edge.y};
    triangles[1] = {edge.x, edge.y, end_edge.y};
    triangles.resize(2);

    adjacencies[0] = {adjacent_to_nothing, adjacent_to_nothing, 1};
    adjacencies[1] = {0, adjacent_to_nothing, adjacent_to_nothing};
    adjacencies.resize(2);

  } else if (edge.y < 3) {
    // Se il segmento ha come inizio un vertice di un triangolo e come fine un
    // punto punto nel lato opposto
    triangles[0] = {edge.y, start_edge.x, edge.x};
    triangles[1] = {edge.y, edge.x, start_edge.y};
    triangles.resize(2);

    adjacencies[0] = {adjacent_to_nothing, adjacent_to_nothing, 1};
    adjacencies[1] = {0, adjacent_to_nothing, adjacent_to_nothing};
    adjacencies.resize(2);

  } else {
    // Se il segmento ha inizio e fine su due lati del triangolo
    auto [x, y] = start_edge;
    if (start_edge.y == end_edge.x) {
      auto z       = end_edge.y;
      triangles[0] = {x, edge.x, z};
      triangles[1] = {edge.x, edge.y, z};
      triangles[2] = {edge.x, y, edge.y};

      adjacencies[0] = {adjacent_to_nothing, 1, adjacent_to_nothing};
      adjacencies[1] = {2, adjacent_to_nothing, 0};
      adjacencies[2] = {adjacent_to_nothing, adjacent_to_nothing, 1};

    } else if (start_edge.x == end_edge.y) {
      auto z       = end_edge.x;
      triangles[0] = {x, edge.x, edge.y};
      triangles[1] = {edge.x, z, edge.y};
      triangles[2] = {edge.x, y, z};

      adjacencies[0] = {adjacent_to_nothing, 1, adjacent_to_nothing};
      adjacencies[1] = {2, adjacent_to_nothing, 0};
      adjacencies[2] = {adjacent_to_nothing, adjacent_to_nothing, 1};

    } else {
      assert(0);
    }
  }

  return {triangles, adjacencies};
}

// Constrained Delaunay Triangulation
static pair<vector<vec3i>, vector<vec3i>> constrained_triangulation(
    vector<vec2f>& nodes, const vector<vec2i>& edges) {
  // Questo purtroppo serve.
  for (auto& n : nodes) n *= 1e9;

  // (marzia): qui usiamo float, ma si possono usare anche i double
  using Float = float;
  auto cdt = CDT::Triangulation<Float>(CDT::FindingClosestPoint::ClosestRandom);
  cdt.insertVertices(
      nodes.begin(), nodes.end(),
      [](const vec2f& point) -> Float { return point.x; },
      [](const vec2f& point) -> Float { return point.y; });
  cdt.insertEdges(
      edges.begin(), edges.end(),
      [](const vec2i& edge) -> int { return edge.x; },
      [](const vec2i& edge) -> int { return edge.y; });

  cdt.eraseOuterTriangles();
  auto adjacencies = vector<vec3i>();
  adjacencies.reserve(cdt.triangles.size());

  auto triangles = vector<vec3i>();
  triangles.reserve(cdt.triangles.size());

  for (auto& tri : cdt.triangles) {
    auto verts = vec3i{
        (int)tri.vertices[0], (int)tri.vertices[1], (int)tri.vertices[2]};

    auto adjacency = vec3i{};
    for (auto k = 0; k < 3; k++) {
      auto neigh = tri.neighbors[k];
      if (neigh == CDT::noNeighbor)
        adjacency[k] = adjacent_to_nothing;
      else
        adjacency[k] = (int)neigh;
    }

    // TODO: serve? (marzia): Forse no!
    auto& a           = nodes[verts.x];
    auto& b           = nodes[verts.y];
    auto& c           = nodes[verts.z];
    auto  orientation = cross(b - a, c - b);
    if (fabs(orientation) < 0.00001) {
      printf("Collinear (ma serve?)\n");
      continue;
    }

    triangles.push_back(verts);
    adjacencies.push_back(adjacency);
  }
  return {triangles, adjacencies};
}

static void update_face_adjacencies(bool_mesh& mesh) {
  // Aggiorniamo le adiacenze per i triangoli che sono stati processati
  auto border_edgemap = hash_map<vec2i, int>{};
  border_edgemap.reserve(mesh.triangulated_faces.size() * 6);

  // Per ogni triangolo processato elaboro tutti i suoi sottotriangoli
  for (auto& [face, faces] : mesh.triangulated_faces) {
    // Converto il triangolo in triplette di vertici globali.
    auto triangles = vector<vec3i>(faces.size());
    for (int i = 0; i < faces.size(); i++) {
      triangles[i] = mesh.triangles[faces[i]];
    }

    for (int i = 0; i < faces.size(); i++) {
      // Guardo se nell'adiacenza ci sono dei triangoli mancanti
      // (segnati con adjacent_to_nothing per non confonderli i -1 già
      // presenti nell'adiacenza della mesh originale).
      auto& adj = mesh.adjacencies[faces[i]];
      for (int k = 0; k < 3; k++) {
        if (adj[k] != adjacent_to_nothing) continue;

        // Prendo l'edge di bordo corrispondente ad un adjacent_to_nothing
        auto edge = get_mesh_edge_from_index(triangles[i], k);

        // Se è un arco della mesh originale lo processo subito
        if (edge.x < mesh.num_positions && edge.y < mesh.num_positions) {
          // Cerco il triangolo adiacente al triangolo originale su quel lato
          for (int kk = 0; kk < 3; kk++) {
            auto edge0 = get_mesh_edge_from_index(mesh.triangles[face], kk);
            if (make_edge_key(edge) == make_edge_key(edge0)) {
              // Aggiorno direttamente l'adiacenza nel nuovo triangolo e del
              // vicino
              auto neighbor = mesh.adjacencies[face][kk];
              if (neighbor == -1) continue;

              mesh.adjacencies[faces[i]][k] = neighbor;

              auto it = find_in_vec(mesh.adjacencies[neighbor], face);
              mesh.adjacencies[neighbor][it] = faces[i];
            }
          }
          continue;
        }

        // Se non è un arco della mesh originale
        auto edge_key = make_edge_key(edge);
        auto it       = border_edgemap.find(edge_key);

        // Se non l'ho mai incontrato salvo in una mappa l'edge e la
        // faccia corrispondente. Se l'ho già incontrato ricostruisco
        // l'adiacenza tra il triangolo corrente e il neighbor già trovato.
        if (it == border_edgemap.end()) {
          // border_edgemap.insert(it, {edge_key, faces[i]});
          border_edgemap[edge_key] = faces[i];
        } else {
          auto neighbor                 = it->second;
          mesh.adjacencies[faces[i]][k] = neighbor;
          for (int kk = 0; kk < 3; ++kk) {
            auto edge2 = get_mesh_edge_from_index(mesh.triangles[neighbor], kk);
            edge2      = make_edge_key(edge2);
            if (edge2 == edge_key) {
              mesh.adjacencies[neighbor][kk] = faces[i];
              break;
            }
          }
        }
      }
    }
  }
}

inline bool check_tags(
    const bool_mesh& mesh, const vector<vec3i>& border_tags) {
  for (int i = 0; i < mesh.triangles.size(); i++) {
    if (mesh.triangulated_faces.find(i) != mesh.triangulated_faces.end()) {
      continue;
    }
    auto face = i;
    auto tr   = mesh.triangles[face];
    if (tr == vec3i{0, 0, 0}) continue;
    for (int k = 0; k < 3; k++) {
      auto neighbor = mesh.adjacencies[face][k];
      if (neighbor < 0) continue;
      auto n0 = mesh.adjacencies[face];
      auto n1 = mesh.adjacencies[neighbor];
      auto kk = find_in_vec(mesh.adjacencies[neighbor], face);
      assert(kk != -1);

      auto tags0 = border_tags[face];
      auto tags1 = border_tags[neighbor];
      auto tag0  = tags0[k];
      auto tag1  = tags1[kk];
      assert(tag0 == -tag1);
    }
  }
  return true;
}

static void triangulate(bool_mesh& mesh, const mesh_hashgrid& hashgrid) {
  for (auto& [face, polylines] : hashgrid) {
    // Calcola le info per la triangolazione, i.e. (nodi ed edge constraints).
    auto info = compute_triangulation_constraints(mesh, face, polylines);

#ifdef MY_DEBUG
    debug_nodes()[face]   = info.nodes;
    debug_indices()[face] = info.indices;
#endif

    // Se la faccia contiene solo segmenti corrispondenti ad edge del triangolo
    // stesso, non serve nessuna triangolazione.
    if (info.nodes.size() == 3) {
      mesh.triangulated_faces[info.face] = {info.face};
      continue;
    }

    auto triangles = vector<vec3i>();
    auto adjacency = vector<vec3i>();

    // Se il triangolo ha al suo interno un solo segmento allora chiamiamo la
    // funzione di triangolazione più semplice, altrimenti chiamiamo CDT
    if (info.edges.size() == 1) {
      tie(triangles, adjacency) = single_split_triangulation(
          info.nodes, info.edges[0]);
    } else {
      add_boundary_edge_constraints(info.edgemap, info.edges);
      tie(triangles, adjacency) = constrained_triangulation(
          info.nodes, info.edges);
    }

#ifdef MY_DEBUG
    debug_edges()[face]     = info.edges;
    debug_triangles()[face] = triangles;
#endif

    // Calcoliamo l'adiacenza locale e la trasformiamo in globale.
    // auto adjacency = face_adjacencies_fast(triangles);
    for (auto& adj : adjacency) {
      for (auto& x : adj) {
        if (x != adjacent_to_nothing) x += mesh.triangles.size();
      }
    }
    mesh.adjacencies += adjacency;

    // Converti triangli locali in globali.
    for (int i = 0; i < triangles.size(); i++) {
      auto& tr = triangles[i];
      tr       = {info.indices[tr.x], info.indices[tr.y], info.indices[tr.z]};
      mesh.triangulated_faces[face].push_back((int)mesh.triangles.size() + i);
    }
    mesh.triangles += triangles;
  }
}

static bool_borders border_tags(
    const bool_mesh& mesh, const mesh_hashgrid& hashgrid, int num_polygons) {
  auto borders         = bool_borders{};
  borders.tags         = vector<vec3i>(mesh.triangles.size(), zero3i);
  borders.num_polygons = num_polygons;

  // Map each mesh edge to the polygons passing through it.
  auto border_map = hash_map<vec2i, unordered_set<int>>{};
  for (auto& [face, polylines] : hashgrid) {
    for (auto& polyline : polylines) {
      auto polygon_id = polyline.polygon;
      for (auto i = 0; i < num_segments(polyline); i++) {
        auto edge     = get_segment_vertices(polyline, i);
        auto edge_key = make_edge_key(edge);
        if (edge == edge_key)
          border_map[edge_key].insert(polygon_id);
        else
          border_map[edge_key].insert(-polygon_id);
      }
    }
  }

  // Map each set of colinear polygons to a virtual tag.
  auto virtual_tag_map = hash_map<unordered_set<int>, int>();
  for (auto& [key, value] : border_map) {
    if (value.size() > 1 && !contains(virtual_tag_map, value)) {
      virtual_tag_map[value] = num_polygons + virtual_tag_map.size();
    }
  }

  // Fill border tags.
  for (auto& [_, faces] : mesh.triangulated_faces) {
    for (auto& face : faces) {
      for (int k = 0; k < 3; k++) {
        auto edge = get_mesh_edge_from_index(mesh.triangles[face], k);
        auto tag  = 0;
        auto it   = border_map.find(edge);
        if (it != border_map.end()) {
          if (it->second.size() > 1) {
            tag = -virtual_tag_map[it->second];
          } else {
            tag = -*it->second.begin();
          }
        } else if (it = border_map.find({edge.y, edge.x});
                   it != border_map.end()) {
          if (it->second.size() > 1) {
            tag = virtual_tag_map[it->second];
          } else {
            tag = *it->second.begin();
          }
        } else {
          continue;
        }
        borders.tags[face][k] = tag;
      }
    }
  }

  check_tags(mesh, borders.tags);

  borders.virtual_tags = vector<hash_set<int>>(virtual_tag_map.size());
  for (auto& [key, value] : virtual_tag_map)
    borders.virtual_tags[value - num_polygons] = key;

  return borders;
}

static vector<int> find_ambient_cells(
    const bool_state& state, const hash_set<int>& cycle_nodes) {
  auto roots     = find_roots(state.cells);
  auto queue     = deque<int>(roots.begin(), roots.end());
  auto distances = vector<int>(state.cells.size(), -99999);
  auto parents   = vector<vector<int>>(state.cells.size());
  for (auto& s : queue) {
    distances[s] = 0;
    parents[s]   = {s};
  }
  while (queue.size()) {
    auto node = queue.front();
    queue.pop_front();

    for (auto& [neighbor, polygon] : state.cells[node].adjacency) {
      if (polygon < 0) continue;
      if (contains(cycle_nodes, node) && contains(cycle_nodes, neighbor)) {
        if (distances[node] == distances[neighbor]) continue;
        parents[neighbor]   = parents[node];
        distances[neighbor] = distances[node];
        queue.push_back(neighbor);
        continue;
      }

      auto new_depth = distances[node] + 1;
      if (new_depth < distances[neighbor]) {
        continue;
      }

      if (new_depth > distances[neighbor]) {
        parents[neighbor]   = {parents[node]};
        distances[neighbor] = new_depth;
      } else if (new_depth == distances[neighbor]) {
        parents[neighbor] += parents[node];
      }
      queue.push_back(neighbor);
    }
  }

  auto max_depth     = max(distances);
  auto ambient_cells = hash_set<int>{};
  for (int i = 0; i < distances.size(); i++) {
    if (distances[i] == max_depth) {
      for (auto& p : parents[i]) ambient_cells.insert(p);
    }
  }

  return vector<int>(ambient_cells.begin(), ambient_cells.end());
}

static void slice_mesh(bool_mesh& mesh, bool_state& state) {
  auto& polygons = state.polygons;

  // Calcoliamo i vertici nuovi della mesh
  // auto vertices             = add_vertices(mesh, polygons);
  state.num_original_points = (int)state.points.size();

  // Calcoliamo hashgrid e intersezioni tra poligoni,
  // aggiungendo ulteriori vertici nuovi alla mesh
  auto hashgrid = compute_hashgrid(mesh, polygons, state.control_points);
  add_polygon_intersection_points(state, hashgrid, mesh);

  // Triangolazione e aggiornamento dell'adiacenza
  triangulate(mesh, hashgrid);
  update_face_adjacencies(mesh);

  // Calcola i border_tags per le facce triangolata.
  mesh.borders = border_tags(mesh, hashgrid, polygons.size());
}

static void compute_cell_labels(bool_state& state) {
  // Calcoliamo possibili cicli all'interno del grafo delle adiacenze della
  // mesh. In modo da eliminare gli archi corrispondenti.
  auto cycles = compute_graph_cycles(state.cells);

  // (marzia) Sicuro si può fare meglio
  auto skip_polygons = hash_set<int>();
  auto cycle_nodes   = hash_set<int>();
  for (auto& cycle : cycles) {
    for (auto& [node, polygon] : cycle) {
      cycle_nodes.insert(node);
      skip_polygons.insert(polygon);
    }
  }

  // Calcoliamo il labelling definitivo per effettuare le booleane tra
  // poligoni

  state.ambient_cells = find_ambient_cells(state, cycle_nodes);

  state.labels = propagate_cell_labels(state.cells, state.ambient_cells, cycles,
      skip_polygons, state.polygons.size());

  // Applichiamo la even-odd rule nel caso in cui le label > 1 (Nelle self
  // intersections posso entrare in un poligono più volte senza esserne prima
  // uscito)
  for (auto& ll : state.labels) {
    for (auto& label : ll) {
      assert(label >= 0);
      if (label > 1) label = label % 2;
    }
  }
}

void update_virtual_adjacencies(
    vector<mesh_cell>& cells, const bool_borders& borders) {
  // Update with void cells
  for (auto c = 0; c < cells.size(); c++) {
    auto& left                 = cells[c];
    auto  added_left_adjacency = hash_set<vec2i>();

    //    for (const auto it = left.adjacency.begin(); it !=
    //    left.adjacency.end(); it++) {
    for (auto& [neighbor, polygon] : left.adjacency) {
      if (polygon < borders.num_polygons) continue;

      auto  virtual_cell_id = (int)cells.size();
      auto& virtual_cell    = cells.emplace_back();

      auto& right    = cells[neighbor];
      auto& polygons = borders.virtual_tags[polygon - borders.num_polygons];

      for (auto p : polygons) {
        if (p > 0) {
          virtual_cell.adjacency.insert({neighbor, p});
          right.adjacency.insert({virtual_cell_id, -p});
        } else {
          virtual_cell.adjacency.insert({c, -p});
          added_left_adjacency.insert({virtual_cell_id, p});
        }
      }
    }

    for (auto it = cells[c].adjacency.begin();
         it != cells[c].adjacency.end();) {
      auto [neighbor, polygon] = *it;
      if (polygon >= borders.num_polygons)
        it = cells[c].adjacency.erase(it);
      else
        it++;
    }

    cells[c].adjacency.insert(
        added_left_adjacency.begin(), added_left_adjacency.end());
  }
}

void compute_cells(bool_mesh& mesh, bool_state& state) {
  // Triangola mesh in modo da embeddare tutti i poligoni come mesh-edges.
  slice_mesh(mesh, state);

  // Trova celle e loro adiacenza via flood-fill.
  state.cells = make_mesh_cells(mesh.adjacencies, mesh.borders);
  update_virtual_adjacencies(state.cells, mesh.borders);

  // Calcola i label delle celle con una visita sulla loro adiacenza.
  compute_cell_labels(state);
}

void compute_shapes(bool_state& state) {
  // Calcoliamo le informazioni sulla shape, come le celle che ne fanno parte
  auto& shapes  = state.shapes;
  auto& sorting = state.shapes_sorting;
  shapes.resize(state.polygons.size());
  sorting.resize(state.polygons.size());

  // Assign a polygon and a color to each shape.
  for (auto p = 0; p < state.polygons.size(); p++) {
    shapes[p].polygon = p;
    shapes[p].color   = get_color(p);
    sorting[p]        = p;
  }

  // Distribute cells to shapes.
  // La prima shape è relativa alla cella ambiente, che è rotto per
  // definizione
  shapes[0].cells = hash_set<int>(
      state.ambient_cells.begin(), state.ambient_cells.end());
  shapes[0].is_root = false;

  for (auto c = 0; c < state.cells.size(); c++) {
    for (auto p = 0; p < state.labels[c].size(); p++) {
      if (state.labels[c][p] > 0) shapes[p].cells.insert(c);
    }
  }
}

void compute_generator_polygons(
    const bool_state& state, int shape_idx, hash_set<int>& result) {
  // Calcoliamo ricorsivamente i poligoni iniziali coinvolti nelle operazioni
  // che hanno generato la shape corrente
  auto& shape = state.shapes[shape_idx];

  // Se la shape non ha generatori allora corrisponde ad una shape di un
  // poligono
  if (shape.generators == vec2i{-1, -1}) {
    result.insert(shape.polygon);
    return;
  }

  // Calcolo i generatori per le shape che hanno generato quella corrente
  compute_generator_polygons(state, shape.generators.x, result);
  compute_generator_polygons(state, shape.generators.y, result);
}

void compute_shape_borders(const bool_mesh& mesh, bool_state& state) {
  // Calcoliamo tutti i bordi di una shape
  for (auto s = 0; s < state.shapes.size(); s++) {
    auto& shape = state.shapes[s];

    // Calcoliamo il bordo solo per le shape root dell'albero csg
    if (!shape.is_root) continue;

    // Calcoliamo i poligoni iniziali coinvolti nelle operazioni che hanno
    // generato la root (ci serve successivamente per salvare nel bordo
    // solamente i punti corretti)
    auto generator_polygons = hash_set<int>();
    compute_generator_polygons(state, s, generator_polygons);

    auto components = compute_components(state, shape);
    for (auto& component : components) {
      // Step 1: Calcoliamo gli edges che stanno sul bordo
      auto edges = hash_set<vec2i>();

      for (auto c : component) {
        auto& cell = state.cells[c];

        // Per ogni cella che compone la shape calcolo il bordo a partire
        // dalle facce che ne fanno parte
        for (auto face : cell.faces) {
          // Se è una faccia interna allora non costituirà il bordo
          if (mesh.borders.tags[face] == zero3i) continue;

          // Per ogni lato del triangolo considero solamente quelli che sono
          // di bordo (tag != 0)
          auto& tri = mesh.triangles[face];
          for (auto k = 0; k < 3; k++) {
            auto tag = mesh.borders.tags[face][k];
            if (tag == 0) continue;
            auto edge     = get_mesh_edge_from_index(tri, k);
            auto rev_edge = vec2i{edge.y, edge.x};

            // Se 'edge' è già stato incontrato allora esso è un bordo tra due
            // celle che fanno parte dela stessa shape, quindi lo elimino dal
            // set.
            auto it = edges.find(rev_edge);
            if (it == edges.end())
              edges.insert(edge);
            else
              edges.erase(it);
          }
        }
      }

      // Step 2: Riordiniamo i bordi
      // Per ogni vertice salviamo il proprio successivo
      auto next_vert = hash_map<int, int>();
      for (auto& edge : edges) next_vert[edge.x] = edge.y;

      for (auto& [key, value] : next_vert) {
        // Se il valore è -1 abbiamo già processato il punto
        if (value == -1) continue;

        // Aggiungiamo un nuovo bordo
        auto border_points = vector<int>();

        auto current = key;

        while (true) {
          auto next = next_vert.at(current);
          if (next == -1) break;

          next_vert.at(current) = -1;

          // Se il vertice corrente è un punto di controllo lo aggiungo al
          // bordo
          if (contains(state.control_points, current)) {
            // Se è un punto di intersezione controlliamo che i poligoni che
            // lo hanno generato siano entrambi compresi nei poligoni che
            // hanno generato anche la shape.
            if (contains(state.isecs_generators, current)) {
              auto& isec_generators = state.isecs_generators.at(current);

              if (contains(generator_polygons, isec_generators.x) &&
                  contains(generator_polygons, isec_generators.y))
                border_points.push_back(current);
            } else
              border_points.push_back(current);
          }

          // Se un bordo è stato chiuso correttamente lo inseriamo tra i bordi
          // della shape
          if (next == key) {
            shape.border_points.push_back(border_points);
            break;
          } else
            current = next;
        }
      }
    }
  }
}

void compute_bool_operation(bool_state& state, const bool_operation& op) {
  auto& a = state.shapes[op.shape_a];
  auto& b = state.shapes[op.shape_b];

  // Convertiamo il vettore di interi in bool per semplificare le operazioni
  auto aa = vector<bool>(state.cells.size(), false);
  for (auto& c : a.cells) aa[c] = true;

  auto bb = vector<bool>(state.cells.size(), false);
  for (auto& c : b.cells) bb[c] = true;

  if (op.type == bool_operation::Type::op_union) {
    for (auto i = 0; i < aa.size(); i++) aa[i] = aa[i] || bb[i];
  } else if (op.type == bool_operation::Type::op_intersection) {
    for (auto i = 0; i < aa.size(); i++) aa[i] = aa[i] && bb[i];
  } else if (op.type == bool_operation::Type::op_difference) {
    for (auto i = 0; i < aa.size(); i++) aa[i] = aa[i] && !bb[i];
  } else if (op.type == bool_operation::Type::op_symmetrical_difference) {
    for (auto i = 0; i < aa.size(); i++) aa[i] = aa[i] != bb[i];
  }

  // Le shape 'a' e 'b' sono state usate nell'operazione,
  // quindi non sono root del csg tree
  a.is_root = false;
  b.is_root = false;

  // Creiamo una nuova shape risultato, settando come generatori le shape 'a'
  // e 'b' e riconvertendo il vettore di bool a interi
  auto  shape_id = state.shapes.size();
  auto& c        = state.shapes.emplace_back();
  c.generators   = {op.shape_a, op.shape_b};
  c.color        = state.shapes[op.shape_a].color;
  auto sorting   = find_idx(state.shapes_sorting, op.shape_a);

  insert(state.shapes_sorting, sorting, (int)shape_id);

  for (auto i = 0; i < aa.size(); i++)
    if (aa[i]) c.cells.insert(i);
}

mesh_point intersect_mesh(const bool_mesh& mesh, const shape_bvh& bvh,
    const scene_camera& camera, const vec2f& uv) {
  auto ray = camera_ray(
      camera.frame, camera.lens, camera.aspect, camera.film, uv);
  auto isec = intersect_triangles_bvh(bvh, mesh.triangles, mesh.positions, ray);
  return {isec.element, isec.uv};
}

vec3f get_cell_color(const bool_state& state, int cell_id, bool color_shapes) {
  if (color_shapes) {
    for (int s = (int)state.shapes_sorting.size() - 1; s >= 0; s--) {
      auto& shape = state.shapes[state.shapes_sorting[s]];
      if (shape.cells.count(cell_id) && shape.is_root) {
        return shape.color;
      }
    }
    return {1, 1, 1};
  } else {
    auto color = vec3f{0, 0, 0};
    int  count = 0;
    for (int p = 0; p < state.labels[cell_id].size(); p++) {
      auto label = state.labels[cell_id][p];
      if (label > 0) {
        color += get_color(p);
        count += 1;
      }
    }
    if (count > 0) {
      color /= count;
    } else {
      color = {0.9, 0.9, 0.9};
    }
    return color;
  }
}

hash_map<int, vector<vec3i>>& debug_triangles() {
  static hash_map<int, vector<vec3i>> result = {};
  return result;
}
hash_map<int, vector<vec2i>>& debug_edges() {
  static hash_map<int, vector<vec2i>> result = {};
  return result;
}
hash_map<int, vector<vec2f>>& debug_nodes() {
  static hash_map<int, vector<vec2f>> result = {};
  return result;
}
hash_map<int, vector<int>>& debug_indices() {
  static hash_map<int, vector<int>> result = {};
  return result;
}
vector<int>& debug_result() {
  static vector<int> result = {};
  return result;
}
vector<bool>& debug_visited() {
  static vector<bool> result = {};
  return result;
}
vector<int>& debug_stack() {
  static vector<int> result = {};
  return result;
}
bool& debug_restart() {
  static bool result = {};
  return result;
}
