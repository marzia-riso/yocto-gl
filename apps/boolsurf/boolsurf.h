#pragma once

#include "boolsurf_utils.h"

using namespace yocto;
using namespace std;

struct bool_mesh : shape_data {
  vector<vec3i>        adjacencies = {};
  dual_geodesic_solver dual_solver = {};
  vector<vec3i>        border_tags = {};

  int num_triangles = 0;
  int num_positions = 0;
};

struct mesh_segment {
  vec2f start = {};
  vec2f end   = {};
  int   face  = -1;
};

namespace yocto {
struct shade_instance;
}

struct mesh_polygon {
  vector<int>          points      = {};
  vector<mesh_segment> segments    = {};
  vector<int>          inner_faces = {};
  vector<int>          outer_faces = {};

  // TODO(giacomo): Put them in app.
  shade_instance* polyline_shape = nullptr;
  shade_instance* inner_shape    = nullptr;
  shade_instance* outer_shape    = nullptr;
};

struct mesh_cell {
  vector<int>     faces     = {};
  hash_set<vec2i> adjacency = {};  // {cell_id, crossed_polygon_id}
  vector<int>     labels    = {};
};

struct mesh_shape {
  vec3f         color = {0, 0, 0};
  hash_set<int> cells = {};
};

struct bool_state {
  vector<mesh_polygon> polygons = {{}};
  vector<mesh_point>   points   = {};

  int                ambient_cell = -1;
  vector<mesh_cell>  cells        = {};
  vector<mesh_shape> shapes       = {};
};

namespace yocto {  // TODO(giacomo): Fix this.
struct bool_operation {
  enum struct Type {
    op_union,
    op_difference,
    op_intersection,
    op_symmetrical_difference
  };
  int  shape_a = -1;
  int  shape_b = -1;
  Type type    = Type::op_union;

  inline static const auto type_names = vector<string>{"op_union",
      "op_difference", "op_intersection", "op_symmetrical_difference"};
};
}  // namespace yocto

void init_mesh(bool_mesh& mesh);
void reset_mesh(bool_mesh& mesh);
void compute_cells(bool_mesh& mesh, bool_state& state);
void compute_shapes(bool_state& state);
void compute_bool_operation(bool_state& state, const bool_operation& op);

vector<mesh_segment> mesh_segments(const vector<vec3i>& triangles,
    const vector<int>& strip, const vector<float>& lerps,
    const mesh_point& start, const mesh_point& end);

geodesic_path compute_geodesic_path(
    const bool_mesh& mesh, const mesh_point& start, const mesh_point& end);

inline geodesic_path straightest_path(const bool_mesh& mesh,
    const mesh_point& start, const vec2f& direction, float length) {
  return straightest_path(mesh.triangles, mesh.positions, mesh.adjacencies,
      start, direction, length);
}

inline geodesic_path straightest_path(
    const bool_mesh& mesh, const mesh_point& start, const vec2f& coord) {
  auto len = length(coord);
  return straightest_path(mesh.triangles, mesh.positions, mesh.adjacencies,
      start, coord / len, len);
}

inline vec3f eval_position(const bool_mesh& mesh, const mesh_point& point) {
  return eval_position(mesh.triangles, mesh.positions, point);
}

inline vec3f eval_normal(const bool_mesh& mesh, const mesh_point& point) {
  return eval_normal(mesh.triangles, mesh.positions, point);
}

/*
 *
 *
 *
 *
 *
 *
 *
 *
 *     DEBUGGING STUFF
 *
 */

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

template <typename F>
static vector<int> flood_fill(const bool_mesh& mesh, const vector<int>& start,
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
static vector<int> flood_fill(
    const bool_mesh& mesh, const vector<int>& start, F&& check) {
#ifdef MY_DEBUG
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
#endif
}

template <typename F>
static void flood_fill_debug(
    const bool_mesh& mesh, const vector<int>& start, F&& check) {
#ifdef MY_DEBUG
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

  auto tag = mesh.border_tags[face];
  auto adj = mesh.adjacencies[face];
  printf("\nfrom %d: tag(%d %d %d) adj(%d %d %d)\n", face, tag[0], tag[1],
      tag[2], adj[0], adj[1], adj[2]);

  for (auto neighbor : mesh.adjacencies[face]) {
    if (neighbor < 0 || debug_visited[neighbor]) continue;
    auto tag = mesh.border_tags[neighbor];
    auto adj = mesh.adjacencies[neighbor];
    if (check(face, neighbor)) {
      debug_stack.push_back(neighbor);
      printf("ok   %d: tag(%d %d %d) adj(%d %d %d)\n", neighbor, tag[0], tag[1],
          tag[2], adj[0], adj[1], adj[2]);
    }
    printf("no   %d: tag(%d %d %d) adj(%d %d %d)\n", neighbor, tag[0], tag[1],
        tag[2], adj[0], adj[1], adj[2]);
  }
#endif
}
