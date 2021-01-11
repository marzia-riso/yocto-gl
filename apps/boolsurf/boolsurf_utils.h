#pragma once

#include <yocto/yocto_mesh.h>
#include <yocto/yocto_shape.h>

#include <cassert>
#include <unordered_set>

#include "ext/delaunator.cpp"

using namespace yocto;
using namespace std;

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

  vec2f start = {};
  vec2f end   = {};
};

struct mesh_segment {
  vec2f start = {};
  vec2f end   = {};
  int   face  = -1;
};

struct mesh_polygon {
  vector<int>                  points      = {};
  vector<vector<mesh_segment>> edges       = {};
  vector<int>                  inner_faces = {};
  vector<int>                  outer_faces = {};
};

struct cell_polygon {
  vector<int>                  points    = {};
  vector<vector<mesh_segment>> edges     = {};
  vector<int>                  embedding = {};
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

inline int find_node(const vector<vec2f>& vec, const vec2f& x) {
  for (auto i = 0; i < vec.size(); i++)
    if (distance(vec[i], x) < 0.001) return i;
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

inline vec2i make_edge_key(const vec2i& edge) {
  if (edge.x > edge.y) return {edge.y, edge.x};
  return edge;
};

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

inline vector<int> compute_mapping(const vector<vec2f>& nodes, const int face,
    bool_mesh&                                       mesh,
    unordered_map<vec2i, vector<tuple<int, float>>>& vertex_edgemap) {
  auto mapping = vector<int>(nodes.size());
  auto verts   = mesh.triangles[face];
  mapping[0]   = verts.x;
  mapping[1]   = verts.y;
  mapping[2]   = verts.z;

  for (int i = 3; i < nodes.size(); i++) {
    auto point = mesh_point{face, nodes[i]};
    auto pos   = eval_position(mesh.triangles, mesh.positions, point);

    // Ordered edge where the point lies
    auto [edge, l1] = get_mesh_edge(verts, nodes[i]);
    auto edge_key   = make_edge_key(edge);
    if (edge_key != edge) l1 = 1.0f - l1;
    auto id = -1;

    // Point in triangle
    if (edge_key == zero2i) {
      id = mesh.positions.size();
      mesh.positions.push_back(pos);
    } else if (vertex_edgemap.find(edge_key) == vertex_edgemap.end()) {
      id = mesh.positions.size();
      mesh.positions.push_back(pos);
      vertex_edgemap[edge_key] = {{id, l1}};
    } else {
      auto& endpoints = vertex_edgemap[edge_key];
      for (auto e = 0; e < endpoints.size(); e++) {
        auto& [p, l] = endpoints[e];
        if (fabs(l1 - l) < 0.0001) {
          id = p;
          break;
        } else if (l1 > l) {
          id = mesh.positions.size();
          mesh.positions.push_back(pos);
          endpoints.insert(endpoints.begin() + e, {id, l1});
          break;
        }
      }

      if (id == -1) {
        id = mesh.positions.size();
        mesh.positions.push_back(pos);
        endpoints.insert(endpoints.end(), {id, l1});
      }
    }
    mapping[i] = id;
  }
  return mapping;
}

inline vector<vec3i> triangulate(const vector<vec2f>& nodes) {
  auto coords = vector<double>();
  coords.reserve(nodes.size() * 2);
  for (auto& node : nodes) {
    coords.push_back(node.x);
    coords.push_back(node.y);
  }

  auto dt        = delaunator::Delaunator(coords);
  auto triangles = vector<vec3i>();
  triangles.reserve(dt.triangles.size() / 3);
  for (int i = 0; i < dt.triangles.size(); i += 3) {
    auto verts = vec3i{(int)dt.triangles[i], (int)dt.triangles[i + 2],
        (int)dt.triangles[i + 1]};

    // Check collinearity
    auto& a = nodes[verts.x];
    auto& b = nodes[verts.y];
    auto& c = nodes[verts.z];
    auto or = cross(b - a, c - b);
    if (fabs(or) < 0.001) {
      continue;
    }

    triangles.push_back(verts);
  }
  return triangles;
}

inline void update_face_edgemap(unordered_map<vec2i, vec2i>& face_edgemap,
    const vec2i& edge, const int face) {
  auto key = make_edge_key(edge);
  if (face_edgemap.find(key) == face_edgemap.end())
    face_edgemap[key] = {face, -1};
  else
    face_edgemap[key].y = face;
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

inline void print_graph(const unordered_map<int, vector<int>>& graph) {
  printf("Graph:\n");
  for (auto& [node, adjacents] : graph) {
    printf("%d: [", node);
    for (auto adj : adjacents) printf("%d ", adj);
    printf("]\n");
  }
  printf("\n");
}

inline void print_dual_graph(const vector<vector<edge>>& graph) {
  printf("Dual Graph:\n");
  for (int i = 0; i < graph.size(); i++) {
    printf("%d: [", i);
    for (int k = 0; k < graph[i].size(); k++) {
      printf("(%d C: %d) ", graph[i][k].point, graph[i][k].counterclock);
    }
    printf("]\n");
  }
  printf("\n");
}

inline void print_cells(const vector<cell_polygon>& cells) {
  printf("Cells:\n");
  for (auto& cell : cells) {
    printf("[");
    for (auto point : cell.points) printf("%d ", point);
    printf("] \n");
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

inline vector<unordered_map<int, vector<int>>> compute_connected_components(
    const vector<vector<int>>& graph) {
  auto visited    = vector<bool>(graph.size(), false);
  auto components = vector<unordered_map<int, vector<int>>>();
  auto queue      = std::deque<int>{};

  for (auto node = 0; node < graph.size(); node++) {
    if (visited[node]) continue;
    visited[node] = true;
    queue.push_back(node);
    auto component = unordered_map<int, vector<int>>();

    while (!queue.empty()) {
      auto current_node       = queue.front();
      component[current_node] = graph[current_node];
      queue.pop_front();

      for (auto neigh : graph[current_node]) {
        if (visited[neigh]) continue;
        visited[neigh] = true;
        queue.push_back(neigh);
      }
    }
    components.push_back(component);
  }
  return components;
}

inline unordered_map<vec2i, std::pair<int, bool>> compute_edge_info(
    unordered_map<vec2i, vector<intersection_node>>& edge_map,
    const vector<mesh_polygon>&                      polygons) {
  auto edge_info    = unordered_map<vec2i, std::pair<int, bool>>();
  auto counterclock = unordered_map<int, bool>();

  for (auto i = 0; i < polygons.size(); i++) {
    auto& polygon = polygons[i];

    auto a = polygon.edges.back().back();
    auto b = polygon.edges.front().front();

    auto v = a.end - a.start;
    auto w = b.end - b.start;

    auto ccwise = cross(v, w) > 0;

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
      }
    }
  }
  return edge_info;
}

inline vector<vector<vec2i>> compute_graph_faces(
    const unordered_map<int, vector<int>>& graph) {
  auto edges = vector<vec2i>();
  for (auto& [node, adjacents] : graph) {
    for (auto& adj : adjacents) edges.push_back(vec2i{node, adj});
  }

  auto faces = vector<vector<vec2i>>();
  auto path  = vector<vec2i>();
  path.push_back(edges.back());
  edges.pop_back();

  while (edges.size() > 0) {
    auto neighbors = graph.at(path.back().y);
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

inline vector<cell_polygon> compute_cells(
    const unordered_map<int, vector<int>>& graph,
    const vector<mesh_point>& points, const bool_mesh& mesh,
    const int num_polygons) {
  auto graph_faces = compute_graph_faces(graph);
  auto cells       = vector<cell_polygon>(graph_faces.size());

  for (auto i = 0; i < graph_faces.size(); i++) {
    auto cell = cell_polygon{};
    for (auto j = 0; j < graph_faces[i].size(); j++)
      cell.points.push_back(graph_faces[i][j].x);
    cell.points.push_back(cell.points.front());

    cell.embedding.reserve(num_polygons);
    cell.edges.reserve(cell.points.size() - 1);
    for (auto j = 0; j < num_polygons; j++) cell.embedding.push_back(0);

    for (auto j = 0; j < cell.points.size() - 1; j++) {
      auto& start = points[cell.points[j]];
      auto& end   = points[cell.points[j + 1]];

      auto path     = compute_geodesic_path(mesh, start, end);
      auto segments = mesh_segments(
          mesh.triangles, path.strip, path.lerps, path.start, path.end);
      cell.edges.push_back(segments);
    }
    cells[i] = cell;
  }
  return cells;
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
  for (auto f = 0; f < dual_graph.size(); f++) {
    auto ccwise = false;
    for (auto& adj : dual_graph[f]) ccwise = ccwise || adj.counterclock;
    if (!ccwise) return f;
  }
  return -1;
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

  printf("Cells: \n");
  for (auto i = 0; i < cells.size(); i++) {
    printf("%d: Label: ", i);
    for (auto& e : cells[i].embedding) printf("%d ", e);
    printf("\n");
  }
  printf("\n");
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

inline void polygon_common(
    const vector<cell_polygon>& cells, vector<int>& cell_ids, const int num) {
  if (num < 1) return;

  for (auto i = 0; i < cells.size(); i++) {
    auto  sum   = 0;
    auto& label = cells[i].embedding;
    for (auto& l : label) sum += l;
    cell_ids[i] = sum >= num;
  }
  return;
}