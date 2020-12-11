#pragma once

#include <yocto/yocto_mesh.h>
#include <yocto/yocto_shape.h>

#include <cassert>
#include <unordered_set>

using namespace yocto;
using std::unordered_set;

struct bool_mesh {
  vector<vec3i>        triangles   = {};
  vector<vec3i>        adjacencies = {};
  vector<vec3f>        positions   = {};
  vector<vec3f>        normals     = {};
  dual_geodesic_solver dual_solver = {};
};

struct hashgrid_entry {
  int polygon_id = -1;
  int edge_id    = -1;
  int segment_id = -1;

  vec2f start;
  vec2f end;
};

struct mesh_segment {
  vec2f start = {};
  vec2f end   = {};
  int   face  = -1;
};

struct mesh_polygon {
  vector<int>                  points = {};
  vector<vector<mesh_segment>> edges  = {};
};

struct cell_polygon {
  vector<int> points    = {};
  vector<int> embedding = {};
};

struct edge {
  int  point        = -1;
  int  polygon      = -1;
  bool counterclock = false;
};

struct intersection_node {
  int   point   = -1;
  vec2i edges   = {};
  int   polygon = -1;
  int   segment = -1;
  float t       = -1;
};

inline bool is_closed(const mesh_polygon& polygon) {
  if (polygon.points.size() < 3) return false;
  return (polygon.points.front() == polygon.points.back());
}

inline vec2i get_edge_points(const vector<mesh_polygon>& polygons,
    const vector<mesh_point>& points, const int polygon_id, const int edge_id) {
  auto& polygon = polygons[polygon_id];
  auto  a       = (int)polygon.points[edge_id];
  auto  b       = (int)polygon.points[(edge_id + 1) % polygon.points.size()];
  return vec2i{a, b};
}

inline void update_mesh_polygon(
    mesh_polygon& polygon, const vector<mesh_segment>& segments) {
  polygon.edges.push_back(segments);
}

// (marzia) Not used
// inline vector<int> strip_intersection(
//     const mesh_polygon& left, const mesh_polygon& right) {
//   auto left_faces  = polygon_strip(left);
//   auto right_faces = polygon_strip(right);

//   std::sort(left_faces.begin(), left_faces.end());
//   std::sort(right_faces.begin(), right_faces.end());

//   auto intersections = vector<int>();

//   auto i = 0;
//   auto j = 0;
//   while (i < left_faces.size() && j < right_faces.size()) {
//     if (left_faces[i] == right_faces[j]) {
//       intersections.push_back(left_faces[i]);
//       i++;
//       j++;
//     } else if (left_faces[i] < right_faces[j])
//       i++;
//     else
//       j++;
//   }

//   std::sort(intersections.begin(), intersections.end());
//   intersections.erase(std::unique(intersections.begin(),
//   intersections.end()),
//       intersections.end());

//   return intersections;
// }

inline bool_mesh init_mesh(const generic_shape* shape) {
  auto mesh        = bool_mesh{};
  mesh.triangles   = shape->triangles;
  mesh.positions   = shape->positions;
  mesh.normals     = shape->normals;
  mesh.adjacencies = face_adjacencies(mesh.triangles);

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

inline vector<mesh_segment> mesh_segments(const vector<vec3i>& triangles,
    const vector<int>& strip, const vector<float>& lerps,
    const mesh_point& start, const mesh_point& end) {
  auto result = vector<mesh_segment>(strip.size());

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
      end_uv = lerp(a, b, lerps[i]);  // i'm sorry
    }
    result[i] = {start_uv, end_uv, strip[i]};
  }
  return result;
}

inline void print_graph(const vector<vector<int>>& graph) {
  printf("Graph:\n");
  for (int i = 0; i < graph.size(); i++) {
    printf("%d: [", i);
    for (int k = 0; k < graph[i].size(); k++) {
      printf("%d ", graph[i][k]);
    }
    printf("]\n");
  }
  printf("\n");
}

inline void print_dual_graph(const vector<vector<edge>>& graph) {
  printf("Dual Graph:\n");
  for (int i = 0; i < graph.size(); i++) {
    printf("%d: [", i);
    for (int k = 0; k < graph[i].size(); k++) {
      printf("(%d P: %d C: %d) ", graph[i][k].point, graph[i][k].polygon,
          graph[i][k].counterclock);
    }
    printf("]\n");
  }
  printf("\n");
}

inline void print_faces(const vector<vector<vec2i>>& faces) {
  printf("Faces:\n");
  for (int i = 0; i < faces.size(); i++) {
    printf("%d: [", i);
    for (int k = 0; k < faces[i].size(); k++) {
      printf("%d ", faces[i][k].x);
    }
    printf("]\n");
  }
  printf("\n");
}

inline vector<vector<int>> compute_graph(const int   nodes,
    unordered_map<vec2i, vector<intersection_node>>& edge_map,
    const unordered_map<int, bool>&                  counterclockwise) {
  for (auto& [key, value] : edge_map) {
    sort(value.begin(), value.end(), [](auto& a, auto& b) {
      if (a.segment == b.segment) return a.t < b.t;
      return a.segment < b.segment;
    });
  }
  auto keys = vector<vec2i>();
  for (auto& [key, value] : edge_map) keys.push_back(key);
  sort(keys.begin(), keys.end(), [](auto& a, auto& b) { return a.x < b.x; });

  //        C
  //        |
  // A -- point --> B   cross() < 0
  //        |
  //        V
  //        D
  // A, D, B, C

  //        D
  //        ^
  //        |
  // A -- point --> B cross() > 0
  //        |
  //        C
  // A, C, B, D

  // [AB] : A-- point(C, D), point(...) -- B

  auto graph = vector<vector<int>>(nodes);
  for (auto& key : keys) {
    auto& value = edge_map.at(key);
    assert(key.x == value[0].point);
    assert(key.y == value.back().point);
    graph[key.x].push_back(value[1].point);
    graph[key.y].push_back(value[value.size() - 2].point);

    for (int i = 1; i < value.size() - 1; i++) {
      auto& isec = value[i];
      auto  node = isec.point;
      if (graph[node].size()) continue;
      graph[node].resize(4);
      graph[node][0] = value[i - 1].point;
      graph[node][2] = value[i + 1].point;

      auto& other = edge_map.at(isec.edges);
      auto  id    = -1;
      for (int i = 0; i < other.size(); i++) {
        if (other[i].point == node) {
          id = i;
          break;
        }
      }
      assert(id != -1);
      assert(id != 0);
      assert(id != other.size() - 1);
      graph[node][1] = other[id - 1].point;
      graph[node][3] = other[id + 1].point;
      if (counterclockwise.at(node)) {
        swap(graph[node][1], graph[node][3]);
      }
    }
  }
  return graph;
}

inline unordered_map<vec2i, std::pair<int, bool>> compute_edge_info(
    unordered_map<vec2i, vector<intersection_node>>& edge_map,
    const vector<mesh_polygon>&                      polygons) {
  auto edge_info    = unordered_map<vec2i, std::pair<int, bool>>();
  auto counterclock = unordered_map<int, bool>();

  for (auto i = 0; i < polygons.size(); i++) {
    auto& polygon = polygons[i];
    auto& first   = polygon.edges[0].back();
    auto& second  = polygon.edges[1].front();

    auto v      = first.end - first.start;
    auto w      = second.end - second.start;
    auto ccwise = cross(v, w) > 0;
    printf("Orientation: %d\n", ccwise);

    for (auto p = 0; p < polygon.points.size() - 1; p++) {
      auto& first  = polygon.points[p];
      auto& second = polygon.points[p + 1];

      auto& value = edge_map.at({first, second});
      for (auto v = 0; v < value.size() - 1; v++) {
        auto& start = value[v];
        auto& end   = value[v + 1];

        // If self-intersecting
        if (start.edges != vec2i{-1, -1}) {
          auto& other = edge_map.at(start.edges);
          auto  id    = -1;
          for (int i = 0; i < other.size(); i++) {
            if (other[i].point == start.point) {
              id = i;
              break;
            }
          }

          if (start.polygon == other[id].polygon) ccwise = !ccwise;
        }

        edge_info[{start.point, end.point}] = {start.polygon, ccwise};
        edge_info[{end.point, start.point}] = {start.polygon, !ccwise};
        // printf("Edge: %d %d -> %d\n", start.point, end.point, ccwise);
        // printf("Edge: %d %d -> %d\n", end.point, start.point, !ccwise);
      }
      // printf("\n");
    }
  }
  return edge_info;
}

inline vector<vector<vec2i>> compute_graph_faces(
    const vector<vector<int>>& graph) {
  auto edges = vector<vec2i>();
  for (auto node = 0; node < graph.size(); node++) {
    auto& adjacents = graph[node];
    for (auto& adj : adjacents) edges.push_back(vec2i{node, adj});
  }

  auto faces = vector<vector<vec2i>>();
  auto path  = vector<vec2i>();
  path.push_back(edges.back());
  edges.pop_back();

  while (edges.size() > 0) {
    auto neighbors = graph[path.back().y];
    auto last_node = path.back().y;

    auto idx = (find_idx(neighbors, path.back().x) + 1) % neighbors.size();
    auto next_node = neighbors[idx];
    auto tup       = vec2i{last_node, next_node};

    if (tup == path.front()) {
      faces.push_back(path);
      path.clear();
      path.push_back(edges.back());
      edges.pop_back();
    } else {
      path.push_back(tup);
      auto rem_idx = find_idx(edges, tup);
      edges.erase(edges.begin() + rem_idx);
    }
  }

  if (path.size()) faces.push_back(path);
  return faces;
}

inline vector<cell_polygon> compute_arrangement(
    const vector<vector<vec2i>>& cells, const int num_polygons) {
  auto arrangement = vector<cell_polygon>(cells.size());
  for (auto i = 0; i < cells.size(); i++) {
    auto c = cell_polygon{};
    c.embedding.reserve(num_polygons);
    for (auto i = 0; i < num_polygons; i++) c.embedding.push_back(0);

    for (auto j = 0; j < cells[i].size(); j++) {
      c.points.push_back(cells[i][j].x);
    }
    c.points.push_back(c.points.front());
    arrangement[i] = c;
  }
  return arrangement;
}

inline vector<vector<edge>> compute_dual_graph(
    const vector<cell_polygon>&                 cells,
    unordered_map<vec2i, std::pair<int, bool>>& edge_polygon) {
  auto edge_cell  = unordered_map<vec2i, int>();
  auto dual_graph = vector<vector<edge>>(cells.size());

  for (auto c = 0; c < cells.size(); c++) {
    auto& cell = cells[c];
    for (auto p = 0; p < cell.points.size() - 1; p++) {
      auto edge     = vec2i{cell.points[p], cell.points[p + 1]};
      auto rev_edge = vec2i{edge.y, edge.x};
      if (edge_cell.find(edge) != edge_cell.end()) {
        auto& [pol, ccwise]         = edge_polygon[edge];
        auto& [rev_pol, rev_ccwise] = edge_polygon[rev_edge];
        dual_graph[c].push_back({edge_cell[edge], pol, ccwise});
        dual_graph[edge_cell[edge]].push_back({c, rev_pol, rev_ccwise});
      } else {
        edge_cell[rev_edge] = c;
      }
    }
  }

  for (auto& adj : dual_graph) {
    sort(adj.begin(), adj.end(),
        [](auto& a, auto& b) { return a.point < b.point; });

    adj.erase(unique(adj.begin(), adj.end(),
                  [](auto& a, auto& b) {
                    return ((a.point == b.point) && (a.polygon == b.polygon) &&
                            (a.counterclock == b.counterclock));
                  }),
        adj.end());
  }
  return dual_graph;
}

inline int compute_outer_face(const vector<vector<edge>>& dual_graph) {
  auto face = -1;
  for (auto f = 0; f < dual_graph.size(); f++) {
    auto ccwise = false;
    for (auto& adj : dual_graph[f]) ccwise = ccwise || adj.counterclock;
    if (!ccwise) {
      face = f;
      break;
    }
  }
  return face;
}

inline void visit_dual_graph(const vector<vector<edge>>& dual_graph,
    vector<cell_polygon>& cells, int start) {
  auto queue   = std::deque<int>{};
  auto visited = vector<bool>(dual_graph.size());

  visited[start] = true;
  queue.push_back(start);

  while (!queue.empty()) {
    auto current = queue.front();

    queue.pop_front();
    for (auto adj : dual_graph[current]) {
      if (visited[adj.point]) continue;
      auto embedding = cells[current].embedding;

      if (adj.counterclock)
        embedding[adj.polygon] -= 1;
      else
        embedding[adj.polygon] += 1;
      cells[adj.point].embedding = embedding;
      visited[adj.point]         = true;

      queue.push_back(adj.point);
    }
  }

  printf("\n");
  printf("Cells: \n");
  for (auto i = 0; i < cells.size(); i++) {
    printf("%d: Label: ", i);
    for (auto& e : cells[i].embedding) printf("%d ", e);
    printf("\n");
  }
}

// Polygon operations
inline void polygon_and(const vector<cell_polygon>& cells,
    vector<int>& cell_ids, const int polygon) {
  for (auto i = 0; i < cells.size(); i++) {
    auto& label = cells[i].embedding[polygon];
    cell_ids[i] = cell_ids[i] && label;
  }
}

inline void polygon_or(const vector<cell_polygon>& cells, vector<int>& cell_ids,
    const int polygon) {
  for (auto i = 0; i < cells.size(); i++) {
    auto& label = cells[i].embedding[polygon];
    cell_ids[i] = cell_ids[i] || label;
  }
}

inline void polygon_not(const vector<cell_polygon>& cells,
    vector<int>& cell_ids, const int polygon) {
  for (auto i = 0; i < cells.size(); i++) {
    auto& label = cells[i].embedding[polygon];
    cell_ids[i] = !label;
  }
}

inline vector<int> polygon_common(
    const vector<cell_polygon>& cells, vector<int>& cell_ids, const int num) {
  auto result = vector<int>();
  if (num < 2) return result;

  for (auto i = 0; i < cells.size(); i++) {
    auto  sum   = 0;
    auto& label = cells[i].embedding;
    for (auto& l : label) sum += l;
    if (sum >= num)
      cell_ids[i] = 1;
    else
      cell_ids[i] = 0;
  }
  return result;
}