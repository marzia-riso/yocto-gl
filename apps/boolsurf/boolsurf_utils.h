#pragma once

#include <yocto/yocto_mesh.h>
#include <yocto/yocto_shape.h>

#include <cassert>
#include <deque>
#include <unordered_set>

#include "ext/CDT/CDT/include/CDT.h"

using namespace yocto;
using namespace std;

struct bool_mesh {
  vector<vec3i>        triangles          = {};
  vector<vec3i>        adjacencies        = {};
  vector<vec3f>        positions          = {};
  vector<vec3f>        normals            = {};
  dual_geodesic_solver dual_solver        = {};
  vector<vec3i>        tags               = {};
  int                  original_positions = -1;
};

struct mesh_segment {
  vec2f start = {};
  vec2f end   = {};
  int   face  = -1;
};

struct mesh_polygon {
  vector<int>          points      = {};
  vector<mesh_segment> segments    = {};
  vector<int>          inner_faces = {};
  vector<int>          outer_faces = {};

  shade_instance* polyline_shape = nullptr;
  shade_instance* inner_shape    = nullptr;
  shade_instance* outer_shape    = nullptr;
};

struct hashgrid_segment {
  int polygon = -1;
  int segment = -1;

  vec2f start = {};
  vec2f end   = {};
};

// Vector append and concatenation
template <typename T>
inline void operator+=(vector<T>& a, const vector<T>& b) {
  a.insert(a.end(), b.begin(), b.end());
}
template <typename T>
inline void operator+=(vector<T>& a, const T& b) {
  a.push_back(b);
}
template <typename T>
inline vector<T> operator+(const vector<T>& a, const vector<T>& b) {
  auto c = a;
  c += b;
  return b;
}

inline vec3f eval_position(const bool_mesh& mesh, const mesh_point& point) {
  return eval_position(mesh.triangles, mesh.positions, point);
}

inline bool_mesh init_mesh(const generic_shape* shape) {
  auto mesh               = bool_mesh{};
  mesh.triangles          = shape->triangles;
  mesh.positions          = shape->positions;
  mesh.normals            = shape->normals;
  mesh.adjacencies        = face_adjacencies(mesh.triangles);
  mesh.original_positions = mesh.positions.size();

  // Fit shape in [-1, 1]^3
  auto bbox = invalidb3f;
  for (auto& pos : mesh.positions) bbox = merge(bbox, pos);
  for (auto& pos : mesh.positions) pos = (pos - center(bbox)) / max(size(bbox));

  mesh.dual_solver = make_dual_geodesic_solver(
      mesh.triangles, mesh.positions, mesh.adjacencies);
  return mesh;
}

inline geodesic_path compute_geodesic_path(
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

// TODO(giacomo): Expose this function in yocto_mesh.h
static int find_in_vec(const vec3i& vec, int x) {
  for (auto i = 0; i < 3; i++)
    if (vec[i] == x) return i;
  return -1;
}

template <class T>
inline int find_idx(const vector<T>& vec, const T& x) {
  for (auto i = 0; i < vec.size(); i++)
    if (vec[i] == x) return i;
  return -1;
}

// TODO(gicomo): rename
template <class T, typename F>
inline int find_xxx(const vector<T>& vec, F&& f) {
  for (auto i = 0; i < vec.size(); i++)
    if (f(vec[i])) return i;
  return -1;
}

// TODO(giacomo): Expose this function in yocto_mesh.h
inline int find_adjacent_triangle(
    const vec3i& triangle, const vec3i& adjacent) {
  for (int i = 0; i < 3; i++) {
    auto k = find_in_vec(adjacent, triangle[i]);
    if (k != -1) {
      if (find_in_vec(adjacent, triangle[mod3(i + 1)]) != -1) {
        return i;
      } else {
        return mod3(i + 2);
      }
    }
  }
  // assert(0 && "input triangles are not adjacent");
  return -1;
}

inline vector<mesh_segment> mesh_segments(const vector<vec3i>& triangles,
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

// From yocto_mesh.h + small update
inline vec2f intersect_segments(const vec2f& start1, const vec2f& end1,
    const vec2f& start2, const vec2f& end2) {
  if (end1 == start2) return zero2f;
  if (end2 == start1) return one2f;
  if (start2 == start1) return zero2f;
  if (end2 == end1) return one2f;

  auto a = end1 - start1;    // direction of line a
  auto b = start2 - end2;    // direction of line b, reversed
  auto d = start2 - start1;  // right-hand side

  auto det = a.x * b.y - a.y * b.x;
  if (det == 0) return {-1, -1};

  auto r = (d.x * b.y - d.y * b.x) / det;
  auto s = (a.x * d.y - a.y * d.x) / det;
  return {r, s};
}

struct hashgrid_polyline {
  int           polygon  = -1;
  vector<vec2f> points   = {};
  vector<int>   vertices = {};
};
using mesh_hashgrid = unordered_map<int, vector<hashgrid_polyline>>;

inline mesh_hashgrid compute_hashgrid(
    const vector<mesh_polygon>& polygons, const vector<vector<int>>& vertices) {
  auto hashgrid = unordered_map<int, vector<hashgrid_polyline>>{};

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
  auto vertex = (int)mesh.positions.size();
  auto pos    = eval_position(mesh.triangles, mesh.positions, point);
  mesh.positions.push_back(pos);
  return vertex;
}

inline vector<vector<int>> add_vertices(
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

struct mesh_cell {
  vector<int>          faces     = {};
  unordered_set<vec2i> adjacency = {};  // {cell_id, crossed_polygon_id}
  vector<int>          labels    = {};
};

void flood_fill_new(vector<mesh_cell>& result, vector<mesh_cell>& cell_stack,
    vector<int>& starts, const bool_mesh& mesh) {
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
        auto p = mesh.tags[face][k];

        auto neighbor_cell = cell_tags[neighbor];
        if (neighbor_cell >= 0 && p != 0) {
          // La faccia neighbor e' gia' stata visitata.
          if (neighbor_cell == cell_id) {
            // Sto visitando la stessa cella.
            if (find_in_vec(mesh.tags[neighbor], -p) != -1) {
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

inline void print_cell_info(const mesh_cell& cell, int idx) {
  printf("[cell %d]\n", idx);
  printf("  faces: %d\n", (int)cell.faces.size());
  printf("  adjacent cells: ");
  for (auto& [cell_id, polygon_id] : cell.adjacency)
    printf("(%d %d) ", cell_id, polygon_id);
  printf("\n");

  printf("  label: ");
  for (auto p = 1; p < cell.labels.size(); p++) printf("%d ", cell.labels[p]);
  printf("\n");

  printf("\n\n");
}

inline vector<int> find_ambient_cells(
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

// inline void fix_self_intersections(
//    vector<mesh_cell>& cells, const vector<int>& start) {
//  // auto visited   = vector<bool>(cells.size(), false);
//  // 0 = not visited, 1 = partially visited, 2 = fully visited
//  auto state     = vector<int>(cells.size(), 0);
//  auto sequences = unordered_map<int, vector<vector<int>>>();
//
//  struct stack_item {
//    int cell;
//    int polygon;
//  };
//
//  // Inizializzazione dello stack
//  auto stack = vector<stack_item>(start.size());
//  for (auto s = 0; s < start.size(); s++) {
//    stack[s] = {start[s], 0};
//  }
//
//  while (!stack.empty()) {
//    auto item = stack.back();
//
//    if (state[item.cell] == 0) {  // If not visited
//      printf("Visiting cell: %d\n", item.cell);
//      state[item.cell] = 1;  // Set to partialy visited
//
//      auto& cell = cells[item.cell];
//      for (auto& [neighbor, polygon] : cell.adjacency) {
//        if (state[neighbor] == 0) {
//          if (item.polygon == polygon) {
//            auto& polygon_sequences = sequences[polygon];
//
//            // Se una catena già esistente termina con il nodo padre di
//            neighbor
//            // allora aggiungo neighbor alla catena
//            auto found = false;
//            for (auto& sequence : polygon_sequences) {
//              if (sequence.back() == item.cell) {
//                sequence.push_back(neighbor);
//                found = true;
//              }
//            }
//
//    auto& cell = cells[item.cell];
//    for (auto& [neighbor, polygon] : cell.adjacency) {
//      // if (visited[neighbor]) {
//      //   // auto tmp = cell.labels;
//      //   // tmp[polygon] += 1;
//      //   // if (tmp != cells[neighbor].labels) {
//      //   //   for (int i = 0; i < cell.labels.size(); i++) {
//      //   //     // cells[neighbor].labels[i] = yocto::max(
//      //   //     // cells[neighbor].labels[i], tmp[i]);
//      //   //     cells[neighbor].labels[i] = cells[neighbor].labels[i] +
//      //   tmp[i];
//      //   //   }
//      //   // }
//      //   continue;
//      // }
//
//      // TODO: non serve piu'?
//      // Individuo diverse catene per ogni poligono
//      if (item.polygon == polygon) {
//        auto& polygon_sequences = sequences[polygon];
//
//        // Se una catena già esistente termina con il nodo padre di neighbor
//        // allora aggiungo neighbor alla catena
//        auto found = false;
//        for (auto& sequence : polygon_sequences) {
//          if (sequence.back() == item.cell) {
//            sequence.push_back(neighbor);
//            found = true;
//          }
//
//          stack.push_back({neighbor, polygon});
//        } else if (state[neighbor] == 1) {
//          printf("Detected cycle\n");
//        }
//      }
//    } else if (state[item.cell] == 1) {
//      stack.pop_back();
//      state[item.cell] = 2;
//    }
//  }
//
//  //   if (visited[item.cell]) continue;
//  //   visited[item.cell] = true;
//
//  //   auto& cell = cells[item.cell];
//  //   for (auto& [neighbor, polygon] : cell.adjacency) {
//  //     // if (visited[neighbor]) {
//  //     //   // auto tmp = cell.labels;
//  //     //   // tmp[polygon] += 1;
//  //     //   // if (tmp != cells[neighbor].labels) {
//  //     //   //   for (int i = 0; i < cell.labels.size(); i++) {
//  //     //   //     // cells[neighbor].labels[i] = yocto::max(
//  //     //   //     // cells[neighbor].labels[i], tmp[i]);
//  //     //   //     cells[neighbor].labels[i] = cells[neighbor].labels[i] +
//  //     //   tmp[i];
//  //     //   //   }
//  //     //   // }
//  //     //   continue;
//  //     // }
//
//  //     // Individuo diverse catene per ogni poligono
//  //     if (item.polygon == polygon) {
//  //       auto& polygon_sequences = sequences[polygon];
//
//  //       // Se una catena già esistente termina con il nodo padre di
//  //       neighbor
//  //       // allora aggiungo neighbor alla catena
//  //       auto found = false;
//  //       for (auto& sequence : polygon_sequences) {
//  //         if (sequence.back() == item.cell) {
//  //           sequence.push_back(neighbor);
//  //           found = true;
//  //         }
//  //       }
//
//  //       // Se non ho trovato una catena già esistente allora la creo nuova
//  //       if (!found) polygon_sequences.push_back({item.cell, neighbor});
//  //     }
//
//  //     stack.push_back({neighbor, polygon});
//  //   }
//  // }
//
//  // Invertiamo gli archi dispari della catena
//  for (auto& [polygon, polygon_sequences] : sequences) {
//    for (auto& sequence : polygon_sequences) {
//      for (auto s = 0; s < sequence.size() - 1; s += 2) {
//        auto s1 = sequence[s];
//        auto s2 = sequence[s + 1];
//
//        // Eliminiamo l'arco in un verso e lo creiamo nell'altro
//        cells[s1].adjacency.erase({s2, polygon});
//        cells[s2].adjacency.insert({s1, polygon});
//      }
//    }
//  }
//}

inline void compute_cycles(const vector<mesh_cell>& cells, int node,
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

inline void compute_cell_labels(vector<mesh_cell>& cells,
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

// inline void visit_dual_graph(const vector<vector<edge>>& dual_graph,
//     vector<cell_polygon>& cells, int start) {
//   auto queue   = std::deque<int>{};
//   auto visited = vector<bool>(dual_graph.size());

//   visited[start] = true;
//   queue.push_back(start);

//   while (!queue.empty()) {
//     auto current = queue.front();

//     queue.pop_front();
//     for (auto adj : dual_graph[current]) {
//       if (visited[adj.point]) continue;
//       auto embedding = cells[current].embedding;

//       if (adj.counterclock)
//         embedding[adj.polygon] -= 1;
//       else
//         embedding[adj.polygon] += 1;
//       cells[adj.point].embedding = embedding;
//       visited[adj.point]         = true;

//       queue.push_back(adj.point);
//     }
//   }

//   printf("Cells: \n");
//   for (auto i = 0; i < cells.size(); i++) {
//     printf("%d: Label: ", i);
//     for (auto& e : cells[i].embedding) printf("%d ", e);
//     printf("\n");
//   }
//   printf("\n");
// }

template <typename T>
inline void insert(vector<T>& vec, size_t i, const T& x) {
  vec.insert(vec.begin() + i, x);
}

inline void compute_intersections(
    unordered_map<int, vector<hashgrid_polyline>>& hashgrid, bool_mesh& mesh) {
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

inline vec2i make_edge_key(const vec2i& edge) {
  if (edge.x > edge.y) return {edge.y, edge.x};
  return edge;
};

// (marzia) Not Used
inline tuple<vec2i, float> get_mesh_edge(
    const vec3i& triangle, const vec2f& uv) {
  if (uv.y == 0)
    return {vec2i{triangle.x, triangle.y}, uv.x};  // point on edge(xy)
  else if (uv.x == 0)
    return {vec2i{triangle.z, triangle.x}, 1.0f - uv.y};  // point on edge (xz)
  else if (fabs(uv.x + uv.y - 1.0f) < 0.0001)
    return {vec2i{triangle.y, triangle.z}, uv.y};  // point on edge (yz)
  else
    return {zero2i, -1};
}

inline pair<int, float> get_mesh_edge(const vec2f& uv) {
  if (uv.y == 0)
    return {0, uv.x};  // point on edge(xy)
  else if (uv.x == 0)
    return {2, 1.0f - uv.y};  // point on edge (xz)
  else if (fabs(uv.x + uv.y - 1.0f) < 0.0001)
    return {1, uv.y};  // point on edge (yz)
  else
    return {-1, -1};
}

inline vec2i get_edge(const vec3i& triangle, int k) {
  if (k == 0)
    return {triangle.x, triangle.y};
  else if (k == 1)
    return {triangle.y, triangle.z};
  else if (k == 2)
    return {triangle.z, triangle.x};
  else {
    assert(0);
    return {-1, -1};
  }
}

// Constrained Delaunay Triangulation
inline vector<vec3i> constrained_triangulation(
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

inline void update_face_adjacencies(bool_mesh& mesh,
    const unordered_map<int, vector<int>>&     triangulated_faces) {
  // Aggiorniamo le adiacenze per i triangoli che sono stati processati
  auto border_edgemap = unordered_map<vec2i, int>{};
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
        if (edge.x < mesh.original_positions &&
            edge.y < mesh.original_positions) {
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
          border_edgemap.insert(it, {edge_key, triangles[i]});
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

inline void update_face_edgemap(unordered_map<vec2i, vec2i>& face_edgemap,
    const vec2i& edge, const int face) {
  auto key = make_edge_key(edge);
  auto it  = face_edgemap.find(key);
  if (it == face_edgemap.end()) {
    face_edgemap.insert(it, {key, {face, -1}});
  } else {
    it->second.y = face;
  }
}

template <typename F>
vector<int> flood_fill(const bool_mesh& mesh, const vector<int>& start,
    const int polygon, F&& check) {
  auto visited = vector<bool>(mesh.adjacencies.size(), false);

  auto result = vector<int>();
  auto stack  = start;

  while (!stack.empty()) {
    auto face = stack.back();
    stack.pop_back();

    if (visited[face]) continue;
    visited[face] = true;

    result.push_back(face);

    for (auto neighbor : mesh.adjacencies[face]) {
      if (neighbor < 0 || visited[neighbor])
        continue;
      else if (check(face, -polygon) && check(neighbor, -polygon))
        // Check if "face" is not inner and "neighbor" is outer
        stack.push_back(neighbor);
      else if (check(neighbor, polygon))
        stack.push_back(neighbor);
    }
  }

  return result;
}

template <typename F>
vector<int> flood_fill(
    const bool_mesh& mesh, const vector<int>& start, F&& check) {
  auto visited = vector<bool>(mesh.adjacencies.size(), false);

  auto result = vector<int>();
  auto stack  = start;

  while (!stack.empty()) {
    auto face = stack.back();
    stack.pop_back();

    if (visited[face]) continue;
    visited[face] = true;

    result.push_back(face);

    for (auto neighbor : mesh.adjacencies[face]) {
      if (neighbor < 0 || visited[neighbor]) continue;
      if (check(face, neighbor)) stack.push_back(neighbor);
    }
  }

  return result;
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

      auto tags0 = mesh.tags[face];
      auto tags1 = mesh.tags[neighbor];
      auto tag0  = tags0[k];
      auto tag1  = tags1[kk];
      assert(tag0 == -tag1);
    }
  }
  return true;
}

static auto debug_result  = vector<int>();
static auto debug_visited = vector<bool>{};
static auto debug_stack   = vector<int>{};
static auto debug_restart = true;

template <typename F>
void flood_fill_debug(
    const bool_mesh& mesh, const vector<int>& start, F&& check) {
  int face = -1;
  if (debug_stack.empty()) {
    debug_restart = true;
    return;
  }
  while (!debug_stack.empty()) {
    auto f = debug_stack.back();
    debug_stack.pop_back();
    if (debug_visited[f]) continue;
    face = f;
    break;
  }
  if (face == -1) return;

  debug_visited[face] = true;

  debug_result.push_back(face);

  auto tag = mesh.tags[face];
  auto adj = mesh.adjacencies[face];
  printf("\nfrom %d: tag(%d %d %d) adj(%d %d %d)\n", face, tag[0], tag[1],
      tag[2], adj[0], adj[1], adj[2]);

  for (auto neighbor : mesh.adjacencies[face]) {
    if (neighbor < 0 || debug_visited[neighbor]) continue;
    auto tag = mesh.tags[neighbor];
    auto adj = mesh.adjacencies[neighbor];
    if (check(face, neighbor)) {
      debug_stack.push_back(neighbor);
      printf("ok   %d: tag(%d %d %d) adj(%d %d %d)\n", neighbor, tag[0], tag[1],
          tag[2], adj[0], adj[1], adj[2]);
    }
    printf("no   %d: tag(%d %d %d) adj(%d %d %d)\n", neighbor, tag[0], tag[1],
        tag[2], adj[0], adj[1], adj[2]);
  }
}
