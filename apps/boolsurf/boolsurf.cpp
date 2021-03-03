#include "boolsurf.h"

#include <cassert>

#include "ext/CDT/CDT/include/CDT.h"

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

  mesh.dual_solver = make_dual_geodesic_solver(
      mesh.triangles, mesh.positions, mesh.adjacencies);
}

void reset_mesh(bool_mesh& mesh) {
  mesh.triangles.resize(mesh.num_triangles);
  mesh.adjacencies.resize(mesh.num_triangles);  // TODO(giacomo): Not correct!
  mesh.positions.resize(mesh.num_positions);
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

  auto strip = strip_on_dual_graph(
      mesh.dual_solver, mesh.triangles, mesh.positions, end.face, start.face);
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

struct hashgrid_polyline {
  int           polygon  = -1;
  vector<vec2f> points   = {};
  vector<int>   vertices = {};
};
using mesh_hashgrid = hash_map<int, vector<hashgrid_polyline>>;

static mesh_hashgrid compute_hashgrid(
    const vector<mesh_polygon>& polygons, const vector<vector<int>>& vertices) {
  auto hashgrid = hash_map<int, vector<hashgrid_polyline>>{};

  for (auto polygon_id = 0; polygon_id < polygons.size(); polygon_id++) {
    auto& polygon   = polygons[polygon_id];
    int   last_face = -1;

    if (polygon.segments.empty()) continue;
    int first_face = polygon.segments[0].face;
    int s_first    = -1;

    for (auto s = 0; s < polygon.segments.size(); s++) {
      auto& segment = polygon.segments[s];

      if (segment.face == first_face && s_first == -1) continue;
      if (s_first == -1) s_first = s;

      auto& entry = hashgrid[segment.face];
      if (segment.face != last_face) {
        auto& polyline   = entry.emplace_back();
        polyline.polygon = polygon_id;

        auto ss = s - 1;
        assert(ss >= 0);
        polyline.vertices.push_back(vertices[polygon_id][ss]);
        polyline.points.push_back(segment.start);

        polyline.vertices.push_back(vertices[polygon_id][s]);
        polyline.points.push_back(segment.end);
      } else {
        auto& polyline = entry.back();
        assert(segment.end != polyline.points.back());
        polyline.points.push_back(segment.end);
        polyline.vertices.push_back(vertices[polygon_id][s]);
      }
      auto& polyline = entry.back();
      if (polyline.points.size() >= 2) {
        assert(polyline.points.back() != polyline.points.end()[-2]);
      }
      last_face = segment.face;
    }

    // Ripetiamo perche' la prima polyline non la calcoliamo al primo giro.
    assert(last_face != -1);
    for (auto s = 0; s < s_first; s++) {
      auto& segment  = polygon.segments[s];
      auto& entry    = hashgrid[segment.face];
      auto& polyline = entry.back();
      assert(segment.face == last_face);
      polyline.points.push_back(segment.end);
      polyline.vertices.push_back(vertices[polygon_id][s]);
    }
  }
  return hashgrid;
}

inline int add_vertex(bool_mesh& mesh, const mesh_point& point) {
  float eps = 0.00001;
  auto  uv  = point.uv;
  auto  tr  = mesh.triangles[point.face];
  if (uv.x < eps && uv.y < eps) return tr.x;
  if (uv.x > 1 - eps && uv.y < eps) return tr.y;
  if (uv.y > 1 - eps && uv.x < eps) return tr.z;
  auto vertex = mesh.positions.size();
  auto pos    = eval_position(mesh.triangles, mesh.positions, point);
  mesh.positions.push_back(pos);
  return vertex;
}

static vector<vector<int>> add_vertices(
    bool_mesh& mesh, const vector<mesh_polygon>& polygons) {
  auto vertices = vector<vector<int>>(polygons.size());
  for (int i = 0; i < polygons.size(); i++) {
    auto& segments = polygons[i].segments;
    vertices[i].resize(segments.size());

    for (auto s = 0; s < segments.size(); s++) {
      vertices[i][s] = add_vertex(mesh, {segments[s].face, segments[s].end});
    }
  }
  return vertices;
}

static void flood_fill_new(vector<mesh_cell>& result,
    vector<mesh_cell>& cell_stack, vector<int>& starts, const bool_mesh& mesh) {
  auto cell_tags = vector<int>(mesh.triangles.size(), -1);

  // consume task stack
  while (cell_stack.size()) {
    // pop element from task stack
    auto cell = cell_stack.back();
    cell_stack.pop_back();

    auto face_stack = vector<int>{starts.back()};
    starts.pop_back();

    auto cell_id = (int)result.size();

    while (!face_stack.empty()) {
      auto face = face_stack.back();
      face_stack.pop_back();

      if (cell_tags[face] >= 0) continue;
      cell_tags[face] = cell_id;

      cell.faces.push_back(face);

      for (int k = 0; k < 3; k++) {
        auto neighbor = mesh.adjacencies[face][k];
        if (neighbor < 0) continue;
        auto p = mesh.border_tags[face][k];

        auto neighbor_cell = cell_tags[neighbor];
        if (neighbor_cell >= 0 && p != 0) {
          // La faccia neighbor e' gia' stata visitata.
          if (neighbor_cell == cell_id) {
            // Sto visitando la stessa cella.
            if (find_in_vec(mesh.border_tags[neighbor], -p) != -1) {
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
              result[neighbor_cell].adjacency.insert({cell_id, -p});
            } else {
              // Sto uscendo dal poligono p.
              result[neighbor_cell].adjacency.insert({cell_id, -p});
              cell.adjacency.insert({neighbor_cell, +p});
            }
          }
        } else {
          // La faccia neighbor non e' mai stata visitata.
          if (p == 0) {
            // Non sto attraversando il bordo del poligono p.
            face_stack.push_back(neighbor);
          } else {
            // Sto attraversando il bordo del poligono p.
            cell_stack.push_back({});
            starts.push_back(neighbor);
          }
        }
      }
    }  // End of while

    if (cell.faces.size()) {
      result.push_back(cell);
    }
  }
}

inline vector<mesh_cell> make_mesh_cells(
    const bool_mesh& mesh, const vector<vec3i>& tags) {
  auto cell_stack = vector<mesh_cell>{{}};
  auto starts     = vector<int>{0};
  auto result     = vector<mesh_cell>{};
  flood_fill_new(result, cell_stack, starts, mesh);
  return result;
}

static vector<int> find_ambient_cells(
    const vector<mesh_cell>& cells, const vector<int>& skip_polygons) {
  auto adjacency = vector<int>(cells.size(), 0);
  for (auto& cell : cells) {
    for (auto& [adj, p] : cell.adjacency) {
      if (find_idx(skip_polygons, p) != -1) continue;
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
  // terminiamo
  if (visited[node] == 2) return;

  // Se il nodo in considerazione non è stato completamente visitato e lo stiamo
  // rivisitando ora allora abbiamo trovato un ciclo
  if (visited[node] == 1) {
    auto  cycle   = vector<vec2i>();
    auto& current = parent;
    cycle.push_back(current);

    // Risalgo l'albero della visita fino a che non trovo lo stesso nodo e salvo
    // il ciclo individuato
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

static void compute_cell_labels(vector<mesh_cell>& cells,
    const vector<int>& start, const vector<int>& skip_polygons) {
  auto visited = vector<bool>(cells.size(), false);
  auto stack   = start;

  for (auto& c : start) {
    visited[c] = true;
  }

  while (!stack.empty()) {
    auto cell_id = stack.back();
    stack.pop_back();

    visited[cell_id] = true;

    auto& cell = cells[cell_id];
    for (auto& [neighbor, polygon] : cell.adjacency) {
      if (find_idx(skip_polygons, polygon) != -1) continue;
      if (visited[neighbor]) {
        auto tmp = cell.labels;
        tmp[yocto::abs(polygon)] += polygon > 0 ? 1 : -1;
        if (tmp != cells[neighbor].labels) {
          for (int i = 0; i < cell.labels.size(); i++) {
            cells[neighbor].labels[i] = yocto::max(
                cells[neighbor].labels[i], tmp[i]);
            // cells[neighbor].labels[i] = cells[neighbor].labels[i] + tmp[i];
          }
        }
        continue;
      }
      cells[neighbor].labels = cell.labels;
      cells[neighbor].labels[yocto::abs(polygon)] += polygon > 0 ? 1 : -1;
      stack.push_back(neighbor);
      visited[neighbor] = true;
    }
  }

  for (auto& cell : cells) {
    for (auto& label : cell.labels) {
      label = label % 2;
    }
  }
}

static void compute_intersections(
    hash_map<int, vector<hashgrid_polyline>>& hashgrid, bool_mesh& mesh) {
  for (auto& [face, polylines] : hashgrid) {
    // Check for polyline self interesctions
    for (auto p0 = 0; p0 < polylines.size(); p0++) {
      auto& poly = polylines[p0];

      int num_added = 0;
      for (int s0 = 0; s0 < poly.points.size() - 2; s0++) {
        auto& start0 = poly.points[s0];
        auto& end0   = poly.points[(s0 + 1) % poly.points.size()];
        for (int s1 = s0 + 2; s1 < poly.points.size(); s1++) {
          auto& start1 = poly.points[s1];
          auto& end1   = poly.points[(s1 + 1) % poly.points.size()];

          auto l = intersect_segments(start0, end0, start1, end1);
          if (l.x <= 0.0f || l.x >= 1.0f || l.y <= 0.0f || l.y >= 1.0f) {
            continue;
          }

          auto uv     = lerp(start1, end1, l.y);
          auto vertex = add_vertex(mesh, {face, uv});

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
        auto& poly0     = polylines[p0];
        auto& poly1     = polylines[p1];
        int   num_added = 0;
        for (int s0 = 0; s0 < poly0.points.size() - 1; s0++) {
          auto& start0 = poly0.points[s0];
          auto& end0   = poly0.points[(s0 + 1)];
          for (int s1 = 0; s1 < poly1.points.size() - 1; s1++) {
            auto& start1 = poly1.points[s1];
            auto& end1   = poly1.points[(s1 + 1)];
            auto  l      = intersect_segments(start0, end0, start1, end1);
            if (l.x <= 0.0f || l.x >= 1.0f || l.y <= 0.0f || l.y >= 1.0f) {
              continue;
            }

            auto uv     = lerp(start1, end1, l.y);
            auto vertex = add_vertex(mesh, {face, uv});

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

// Constrained Delaunay Triangulation
static vector<vec3i> constrained_triangulation(
    vector<vec2f> nodes, const vector<vec2i>& edges) {
  // Questo purtroppo serve.
  for (auto& n : nodes) n *= 1e9;

  auto cdt = CDT::Triangulation<double>(
      CDT::FindingClosestPoint::ClosestRandom);
  cdt.insertVertices(
      nodes.begin(), nodes.end(),
      [](const vec2f& point) -> double { return point.x; },
      [](const vec2f& point) -> double { return point.y; });
  cdt.insertEdges(
      edges.begin(), edges.end(), [](const vec2i& edge) { return edge.x; },
      [](const vec2i& edge) { return edge.y; });

  cdt.eraseOuterTriangles();
  auto triangles = vector<vec3i>();
  triangles.reserve(cdt.triangles.size());

  for (auto& tri : cdt.triangles) {
    auto verts = vec3i{
        (int)tri.vertices[0], (int)tri.vertices[1], (int)tri.vertices[2]};

    // TODO: serve?
    auto& a           = nodes[verts.x];
    auto& b           = nodes[verts.y];
    auto& c           = nodes[verts.z];
    auto  orientation = cross(b - a, c - b);
    if (fabs(orientation) < 0.00001) {
      printf("Collinear (ma serve?)\n");
      continue;
    }
    triangles.push_back(verts);
  }
  return triangles;
}

static void update_face_adjacencies(
    bool_mesh& mesh, const hash_map<int, vector<int>>& triangulated_faces) {
  // Aggiorniamo le adiacenze per i triangoli che sono stati processati
  auto border_edgemap = hash_map<vec2i, int>{};
  border_edgemap.reserve(triangulated_faces.size() * 6);

  // Per ogni triangolo processato elaboro tutti i suoi sottotriangoli
  for (auto& [face, triangles] : triangulated_faces) {
    // Converto il triangolo in triplette di vertici
    auto triangles_vec3i = vector<vec3i>(triangles.size());
    for (int i = 0; i < triangles.size(); i++) {
      triangles_vec3i[i] = mesh.triangles[triangles[i]];
    }

    for (int i = 0; i < triangles.size(); i++) {
      // Guardo se nell'adiacenza ci sono dei triangoli mancanti
      // (segnati con -2 per non confonderli con dei -1 già presenti nella
      // mesh originale)
      auto& adj = mesh.adjacencies[triangles[i]];
      for (int k = 0; k < 3; k++) {
        if (adj[k] != -2) continue;

        // Prendo l'edge di bordo corrispondente ad un -2
        auto edge = get_edge(triangles_vec3i[i], k);

        // Se è un arco della mesh originale lo processo subito
        if (edge.x < mesh.num_positions && edge.y < mesh.num_positions) {
          // Cerco il triangolo adiacente al triangolo originale su quel lato
          for (int kk = 0; kk < 3; kk++) {
            auto edge0 = get_edge(mesh.triangles[face], kk);
            if (make_edge_key(edge) == make_edge_key(edge0)) {
              // Aggiorno direttamente l'adiacenza nel nuovo triangolo e del
              // vicino
              auto neighbor                     = mesh.adjacencies[face][kk];
              mesh.adjacencies[triangles[i]][k] = neighbor;

              auto it = find_in_vec(mesh.adjacencies[neighbor], face);
              mesh.adjacencies[neighbor][it] = triangles[i];
            }
          }
          continue;
        }

        // Se non è un arco della mesh originale
        auto edge_key = make_edge_key(edge);
        auto it       = border_edgemap.find(edge_key);

        // Se non l'ho mai incontrato salvo in una mappa l'edge ed il
        // triangolo corrispondente Se l'ho già incontrato ricostruisco
        // l'adiacenza tra il triangolo corrente e il neighbor già trovato
        if (it == border_edgemap.end()) {
          //          border_edgemap.insert(it, {edge_key, triangles[i]});
          border_edgemap[edge_key] = triangles[i];
        } else {
          auto neighbor                     = it->second;
          mesh.adjacencies[triangles[i]][k] = neighbor;
          for (int kk = 0; kk < 3; ++kk) {
            auto edge2 = get_edge(mesh.triangles[neighbor], kk);
            edge2      = make_edge_key(edge2);
            if (edge2 == edge_key) {
              mesh.adjacencies[neighbor][kk] = triangles[i];
              break;
            }
          }
        }
      }
    }
  }
}

inline void update_face_edgemap(
    hash_map<vec2i, vec2i>& face_edgemap, const vec2i& edge, const int face) {
  auto key = make_edge_key(edge);
  auto it  = face_edgemap.find(key);
  if (it == face_edgemap.end()) {
    //    face_edgemap.insert(it, {key, {face, -1}});
    face_edgemap[key] = {face, -1};
  } else {
    it->second.y = face;
  }
}

inline bool check_tags(const bool_mesh& mesh) {
  for (int i = 0; i < mesh.triangles.size(); i++) {
    auto face = i;
    auto tr   = mesh.triangles[face];
    if (tr == vec3i{0, 0, 0}) continue;
    for (int k = 0; k < 3; k++) {
      auto neighbor = mesh.adjacencies[face][k];
      if (neighbor == -1) continue;
      auto n0 = mesh.adjacencies[face];
      auto n1 = mesh.adjacencies[neighbor];
      auto kk = find_in_vec(mesh.adjacencies[neighbor], face);
      assert(kk != -1);

      auto tags0 = mesh.border_tags[face];
      auto tags1 = mesh.border_tags[neighbor];
      auto tag0  = tags0[k];
      auto tag1  = tags1[kk];
      assert(tag0 == -tag1);
    }
  }
  return true;
}

static void triangulate(bool_mesh& mesh, hash_map<vec2i, vec2i>& face_edgemap,
    hash_map<int, vector<int>>& triangulated_faces,
    const mesh_hashgrid&        hashgrid) {
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
          if (vertex_start < mesh.num_positions &&
              vertex < mesh.num_positions) {
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
    auto adjacency = face_adjacencies_fast(triangles);
    for (auto& adj : adjacency) {
      for (auto& x : adj) {
        if (x == -1) {
          x = -2;
        } else {
          x += mesh.triangles.size();
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

      auto triangle_idx = mesh.triangles.size();
      mesh.triangles.push_back({v0, v1, v2});

      update_face_edgemap(face_edgemap, {v0, v1}, triangle_idx);
      update_face_edgemap(face_edgemap, {v1, v2}, triangle_idx);
      update_face_edgemap(face_edgemap, {v2, v0}, triangle_idx);

      triangulated_faces[face].push_back(triangle_idx);
    }
  }
}

static vector<vec3i> face_tags(const bool_mesh& mesh,
    const mesh_hashgrid& hashgrid, const hash_map<vec2i, vec2i>& face_edgemap,
    const hash_map<int, vector<int>>& triangulated_faces) {
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
      }
    }
  }
  return tags;
}

void compute_cells(bool_mesh& mesh, bool_state& state) {
  auto& polygons = state.polygons;

  auto vertices = add_vertices(mesh, polygons);

  auto hashgrid = compute_hashgrid(polygons, vertices);
  compute_intersections(hashgrid, mesh);

  // Mappa a ogni edge generato le due facce generate adiacenti.
  auto face_edgemap       = hash_map<vec2i, vec2i>{};
  auto triangulated_faces = hash_map<int, vector<int>>{};

  // Triangolazione e aggiornamento dell'adiacenza
  triangulate(mesh, face_edgemap, triangulated_faces, hashgrid);
  update_face_adjacencies(mesh, triangulated_faces);

  // Calcola i tags per ogni faccia
  mesh.border_tags = face_tags(
      mesh, hashgrid, face_edgemap, triangulated_faces);

  // Annulliamo le facce che sono già state triangolate
  //  for (auto& [face, triangles] : triangulated_faces) {
  //    if (triangles.size() <= 1) continue;
  // mesh.triangles[face]   = {0, 0, 0};
  // mesh.adjacencies[face] = {-3, -3, -3};
  //  }

  //  check_tags(mesh);

  // Trova l'adiacenza fra celle tramite il flood-fill
  state.cells = make_mesh_cells(mesh, mesh.border_tags);

  //  save_tree_png(app, "0");

  auto cycles = compute_graph_cycles(state.cells);

  auto skip_polygons = vector<int>();
  for (auto& cycle : cycles) {
    for (auto& [node, polygon] : cycle) {
      skip_polygons.push_back(polygon);
    }
  }

  // Calcoliamo il labelling definitivo per effettuare le booleane
  auto label_size = polygons.size();
  if (polygons.back().points.empty()) label_size -= 1;

  for (auto& cell : state.cells) {
    cell.labels = vector<int>(label_size, 0);
  }

  for (auto& cycle : cycles) {
    for (auto& c : cycle) {
      state.cells[c.x].labels[c.y] = 1;
    }
  }

  // Trova le celle ambiente nel grafo dell'adiacenza delle celle
  auto ambient_cells = find_ambient_cells(state.cells, skip_polygons);

  for (auto ambient_cell : ambient_cells) {
    auto cells = state.cells;
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
      state.cells        = cells;
      state.ambient_cell = ambient_cell;
      break;
    }
  }

  // assert(state.ambient_cell != -1);
  //  save_tree_png(app, "1");
}

void compute_shapes(bool_state& state) {
  auto& shapes = state.shapes;
  shapes.resize(state.polygons.size());

  // Assign a polygon and a color to each shape.
  for (auto p = 0; p < state.polygons.size(); p++) {
    if (shapes[p].color == zero3f) shapes[p].color = get_color(p);
  }

  // Distribute cells to shapes
  shapes[0].cells = {state.ambient_cell};
  for (auto c = 0; c < state.cells.size(); c++) {
    auto& cell = state.cells[c];
    for (auto p = 0; p < cell.labels.size(); p++) {
      if (cell.labels[p] > 0) shapes[p].cells.insert(c);
    }
  }
}

void compute_bool_operation(bool_state& state, const bool_operation& op) {
  auto& a = state.shapes[op.shape_a];
  auto& b = state.shapes[op.shape_b];

  // Converting to vector of bools to simplify operations
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
  // Converting back to set of ints
  a.cells.clear();
  for (auto i = 0; i < aa.size(); i++)
    if (aa[i]) a.cells.insert(i);

  b.cells.clear();
}
