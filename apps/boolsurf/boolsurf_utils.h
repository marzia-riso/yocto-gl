#pragma once

#include <yocto/yocto_mesh.h>
#include <yocto/yocto_shape.h>

#include <unordered_set>

using namespace yocto;
using std::sort;
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

struct intersection_node {
  int   point   = -1;
  vec2i edges   = {};
  int   segment = 0;
  float t       = 0;
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
// inline void update_intersection_segments(
//     const vector<isec_polygon>& intersections, const vector<mesh_point>&
//     points, vector<mesh_polygon>& polygons) {
//   auto isecs = vector<vec3i>(intersections.size() * 2);
//   for (auto i = 0; i < intersections.size(); i++) {
//     auto isec        = intersections[i];
//     isecs[2 * i]     = {isec.first.x, isec.first.y, isec.point};
//     isecs[2 * i + 1] = {isec.second.x, isec.second.y, isec.point};
//   }

//   sort(isecs.begin(), isecs.end(), [](auto& a, auto& b) { return a.y >
//   b.y;
//   }); for (auto& isec : isecs) {
//     auto& segments      = polygons[isec.x].segments;
//     auto& point         = points[isec.z];
//     auto  segment_start = mesh_segment{
//         segments[isec.y].start, point.uv, point.face};
//     auto segment_end = mesh_segment{point.uv, segments[isec.y].end,
//     point.face};

//     segments[isec.y] = segment_start;
//     segments.insert(segments.begin() + isec.y + 1, segment_end);
//   }
// }

// (marzia) Not used
// inline vector<int> polygon_strip(const mesh_polygon& polygon) {
//   auto strip = vector<int>(polygon.segments.size());
//   for (auto i = 0; i < polygon.segments.size(); i++)
//     strip[i] = polygon.segments[i].face;
//   return strip;
// }

// (marzia) Not used
// inline vector<mesh_segment> segments_from_face(
//     const mesh_polygon& polygon, int face) {
//   auto segments = vector<mesh_segment>();
//   for (auto i = 0; i < polygon.segments.size(); i++)
//     if (polygon.segments[i].face == face)
//       segments.push_back(polygon.segments[i]);
//   return segments;
// }

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
inline int find_idx(const vector<T>& vec, T x) {
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

inline vector<vector<int>> compute_graph(const int   nodes,
    unordered_map<vec2i, vector<intersection_node>>& edge_map) {
  for (auto& [key, value] : edge_map) {
    sort(value.begin(), value.end(), [](auto& a, auto& b) {
      if (a.segment == b.segment) return a.t < b.t;
      return a.segment < b.segment;
    });
  }

  auto graph = vector<vector<int>>(nodes);
  for (auto& [key, value] : edge_map) {
    for (int i = 0; i < value.size() - 1; i++) {
      auto& first  = value[i];
      auto& second = value[i + 1];

      graph[first.point].push_back(second.point);
      graph[second.point].push_back(first.point);
    }
  }
  return graph;
}

inline vector<vector<vec2i>> compute_graph_faces(vector<vector<int>> graph) {
  auto edges = vector<vec2i>();
  for (auto node = 0; node < graph.size(); node++) {
    auto& adjacents = graph[node];
    for (auto& adj : adjacents) edges.push_back(vec2i{node, adj});
  }

  auto faces = vector<vector<vec2i>>();
  auto path  = vector<vec2i>();
  path.push_back(edges[0]);
  edges.erase(edges.begin());

  while (edges.size() > 0) {
    auto neighbors = graph[path.back().y];
    auto last_node = path.back().y;

    auto idx = (find_idx(neighbors, path.back().x) + 1) % neighbors.size();
    auto next_node = neighbors[idx];
    auto tup       = vec2i{last_node, next_node};

    if (tup == path.front()) {
      faces.push_back(path);
      path.clear();
      path.push_back(edges[0]);
      edges.erase(edges.begin());
    } else {
      path.push_back(tup);
      auto rem_idx = find_idx(edges, tup);
      edges.erase(edges.begin() + rem_idx);
    }
  }

  if (path.size()) faces.push_back(path);
  return faces;
}
