#pragma once

#include <yocto/yocto_mesh.h>
#include <yocto/yocto_shape.h>

#include <cassert>
#include <deque>
#include <unordered_set>

#include "ext/CDT/CDT/include/CDT.h"
#include "ext/delaunator.cpp"

using namespace yocto;
using namespace std;

struct bool_mesh {
  vector<vec3i>        triangles   = {};
  vector<vec3i>        adjacencies = {};
  vector<vec3f>        positions   = {};
  vector<vec3f>        normals     = {};
  dual_geodesic_solver dual_solver = {};
  vector<vec3i>        tags        = {};
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

  shade_instance* inner_shape = nullptr;
  shade_instance* outer_shape = nullptr;

  shade_instance* inner_visit = nullptr;
  shade_instance* outer_visit = nullptr;
};

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

struct cell_polygon {
  vector<int>          points    = {};
  vector<mesh_segment> segments  = {};
  vector<int>          embedding = {};
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

// (marzia) Not Used
inline bool is_closed(const mesh_polygon& polygon) {
  if (polygon.points.size() < 3) return false;
  return (polygon.points.front() == polygon.points.back());
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

inline unordered_map<int, vector<hashgrid_segment>> compute_hashgrid(
    const vector<mesh_polygon>& polygons) {
  auto hashgrid = unordered_map<int, vector<hashgrid_segment>>{};

  for (auto p = 0; p < polygons.size(); p++) {
    auto& polygon = polygons[p];
    for (auto s = 0; s < polygon.segments.size(); s++) {
      auto& segment = polygon.segments[s];
      hashgrid[segment.face].push_back({p, s, segment.start, segment.end});
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

inline unordered_map<vec2i, vector<intersection>> compute_intersections(
    const unordered_map<int, vector<hashgrid_segment>>& hashgrid,
    bool_mesh& mesh, vector<mesh_point>& points) {
  auto intersections = unordered_map<vec2i, vector<intersection>>();
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
        auto point_id = (int)points.size();
        points.push_back(point);

        auto vertex_id = add_vertex(mesh, point);

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
  return intersections;
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

// (Previous implementation) Simple Delaunay Triangulation
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
    auto& a    = nodes[verts.x];
    auto& b    = nodes[verts.y];
    auto& c    = nodes[verts.z];
    auto  area = cross(b - a, c - a);
    if (fabs(area) < 0.00001) {
      printf("heyyyy\n");
      continue;
    }
    // if (fabs(orientation) < 0.00001) {
    //   continue;
    // }

    triangles.push_back(verts);
  }

  // // Area of whole triangle must be 1.
  // auto real_area = cross(nodes[1] - nodes[0], nodes[2] - nodes[0]);
  // assert(fabs(real_area - 1) < 0.001);

  // // Check total area.
  // auto area = 0.0f;
  // for (auto& tr : triangles) {
  //   area += cross(nodes[tr.y] - nodes[tr.x], nodes[tr.z] - nodes[tr.x]);
  // }
  // assert(fabs(area - real_area) < 0.001);

  return triangles;
}

// Constrained Delaunay Triangulation
inline vector<vec3i> constrained_triangulation(
    vector<vec2f> nodes, const vector<vec2i>& edges) {
  for (auto& n : nodes) n *= 1e9;

  auto cdt = CDT::Triangulation<float>(CDT::FindingClosestPoint::ClosestRandom);
  cdt.insertVertices(
      nodes.begin(), nodes.end(), [](const vec2f& point) { return point.x; },
      [](const vec2f& point) { return point.y; });
  cdt.insertEdges(
      edges.begin(), edges.end(), [](const vec2i& edge) { return edge.x; },
      [](const vec2i& edge) { return edge.y; });

  cdt.eraseSuperTriangle();
  auto triangles = vector<vec3i>();
  triangles.reserve(cdt.triangles.size());

  for (auto& tri : cdt.triangles) {
    auto verts = vec3i{
        (int)tri.vertices[0], (int)tri.vertices[1], (int)tri.vertices[2]};

    // Check collinearity
    auto& a           = nodes[verts.x];
    auto& b           = nodes[verts.y];
    auto& c           = nodes[verts.z];
    auto  orientation = cross(b - a, c - b);
    if (fabs(orientation) < 0.00001) {
      printf("Detected collinearity\n");
      continue;
    }

    triangles.push_back(verts);
  }

  // Area of whole triangle must be 1.
  //  auto real_area = cross(nodes[1] - nodes[0], nodes[2] - nodes[0]);
  //  assert(fabs(real_area - 1) < 0.001);
  //
  //  // Check total area.
  //  auto area = 0.0f;
  //  for (auto& tr : triangles) {
  //    area += cross(nodes[tr.y] - nodes[tr.x], nodes[tr.z] - nodes[tr.x]);
  //  }
  //  assert(fabs(area - real_area) < 0.001);

  return triangles;
}

inline void update_face_edgemap(unordered_map<vec2i, vec2i>& face_edgemap,
    const vec2i& edge, const int face) {
  auto key = make_edge_key(edge);

  if (face_edgemap.find(key) != face_edgemap.end()) {
    auto& faces = face_edgemap[key];
    if (faces.x == -1) {
      assert(faces.y == -1);
      faces.x = face;
    } else {
      assert(faces.y == -1);
      faces.y = face;
    }
  }
}

inline vector<vec3i> compute_face_tags(
    const bool_mesh& mesh, const vector<mesh_polygon>& polygons) {
  auto tags = vector<vec3i>(mesh.triangles.size(), zero3i);
  for (auto p = 1; p < polygons.size(); p++) {
    for (auto f : polygons[p].inner_faces) {
      for (auto k = 0; k < 3; k++) {
        if (tags[f][k] == 0) {
          tags[f][k] = -p;
          break;
        }
      }
    }

    for (auto f : polygons[p].outer_faces) {
      for (auto k = 0; k < 3; k++) {
        if (tags[f][k] == 0) {
          tags[f][k] = p;
          break;
        }
      }
    }
  }
  return tags;
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
      if (neighbor < 0 || visited[neighbor]) continue;
      // else if (check(face, -polygon) && check(neighbor, -polygon))
      //   // Check if "face" is not inner and "neighbor" is outer
      //   stack.push_back(neighbor);
      // else if (check(neighbor, polygon))
      //   stack.push_back(neighbor);
      if (check(neighbor, polygon)) stack.push_back(neighbor);
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

inline vector<int> compute_boundary_faces(const vector<vec3i>& adjacencies) {
  auto result = vector<int>();
  for (auto face = 0; face < adjacencies.size(); face++) {
    auto& [f0, f1, f2] = adjacencies[face];
    if ((f0 == -1) || (f1 == -1) || (f2 == -1)) result.push_back(face);
  }
  return result;
}

inline vector<int> check_mesh(const bool_mesh& mesh) {
  auto result = vector<int>();
  for (auto face = 0; face < mesh.adjacencies.size(); face++) {
    auto& [v0, v1, v2] = mesh.triangles[face];
    auto& [f0, f1, f2] = mesh.adjacencies[face];

    if ((v0 == 0) && (v1 == 0) && (v2 == 0)) continue;
    if ((f0 == f1) || (f1 == f2) || (f2 == f0)) result.push_back(face);
  }
  return result;
}

inline vector<int> visit_mesh(
    const bool_mesh& mesh, vector<bool>& visited, const int start) {
  auto result = vector<int>();
  auto stack  = vector<int>(start);

  while (!stack.empty()) {
    auto face = stack.back();
    stack.pop_back();

    if (visited[face]) continue;
    visited[face] = true;
    result.push_back(face);

    for (auto neighbor : mesh.adjacencies[face]) {
      if (neighbor < 0 || visited[neighbor]) continue;
      stack.push_back(neighbor);
    }
  }
  return result;
}

inline vector<vector<int>> compute_mesh_components(const bool_mesh& mesh) {
  auto visited    = vector<bool>(mesh.adjacencies.size(), false);
  auto components = vector<vector<int>>();

  for (auto i = 0; i < visited.size(); i++) {
    if (visited[i]) continue;
    auto component = visit_mesh(mesh, visited, i);
    if (!component.size()) continue;
    components.push_back(component);
  }
  return components;
}

// // Polygon operations (from previous implementation)
// inline void polygon_and(const vector<cell_polygon>& cells,
//     vector<int>& cell_ids, const int polygon) {
//   for (auto i = 0; i < cells.size(); i++) {
//     auto& label = cells[i].embedding[polygon];
//     cell_ids[i] = cell_ids[i] && label;
//   }
// }

// inline void polygon_or(const vector<cell_polygon>& cells, vector<int>&
// cell_ids,
//     const int polygon) {
//   for (auto i = 0; i < cells.size(); i++) {
//     auto& label = cells[i].embedding[polygon];
//     cell_ids[i] = cell_ids[i] || label;
//   }
// }

// inline void polygon_not(const vector<cell_polygon>& cells,
//     vector<int>& cell_ids, const int polygon) {
//   for (auto i = 0; i < cells.size(); i++) {
//     auto& label = cells[i].embedding[polygon];
//     cell_ids[i] = !label;
//   }
// }

// inline void polygon_common(
//     const vector<cell_polygon>& cells, vector<int>& cell_ids, const int num)
//     {
//   if (num < 1) return;

//   for (auto i = 0; i < cells.size(); i++) {
//     auto  sum   = 0;
//     auto& label = cells[i].embedding;
//     for (auto& l : label) sum += l;
//     cell_ids[i] = sum >= num;
//   }
//   return;
// }
