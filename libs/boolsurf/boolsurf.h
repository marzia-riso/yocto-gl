#pragma once

#include "boolsurf_utils.h"

using namespace yocto;
using namespace std;

const static int null_label = -999;

struct bool_borders {
  vector<vec3i>         tags         = {};
  vector<hash_set<int>> virtual_tags = {};
  int                   num_polygons = 0;
};

struct bool_mesh : scene_shape {
  vector<vec3i>        adjacencies = {};
  dual_geodesic_solver dual_solver = {};
  bool_borders         borders     = {};

  shape_bvh                  bvh                = {};
  bbox3f                     bbox               = {};
  int                        num_triangles      = 0;
  int                        num_positions      = 0;
  hash_map<int, vector<int>> triangulated_faces = {};
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
  vector<int>                  points = {};
  vector<vector<mesh_segment>> edges  = {};
  int                          length = 0;

  bool is_contained_in_single_face = false;
};

// Informazioni per la triangolazione di una faccia della mesh
// Contiene: UV coords dei nodi locali di un triangolo.
// Indici globali della mesh corrispondenti ai nodi locali
// Edges con indici locali per vincolare la triangolazione
// Mappa che va da lato del triangolo k = 1, 2, 3 e a lista di nodi e lerp
// corrispondenti su quel lato (serve per creare ulteriori vincoli)
struct triangulation_info {
  int face = -1;

  vector<vec2f>                      nodes   = {};
  vector<int>                        indices = {};
  vector<vec2i>                      edges   = {};
  array<vector<pair<int, float>>, 3> edgemap = {};
};

struct mesh_cell {
  vector<int>     faces     = {};
  hash_set<vec2i> adjacency = {};  // {cell_id, crossed_polygon_id}
  // vector<int>     labels    = {};
};

struct mesh_shape {
  int   polygon    = 0;
  vec2i generators = {-1, -1};
  bool  is_root    = true;

  vec3f         color = {0, 0, 0};
  hash_set<int> cells = {};

  vector<vector<int>> border_points = {};
  shade_instance*     borders_shape = nullptr;
};

struct bool_state {
  vector<mesh_polygon> polygons = {{}};
  vector<mesh_point>   points   = {};

  int                  num_original_points = 0;
  hash_map<int, int>   control_points      = {};
  hash_map<int, vec2i> isecs_generators    = {};

  vector<mesh_cell>   cells         = {};
  vector<int>         ambient_cells = {};
  vector<vector<int>> labels        = {};

  vector<mesh_shape> shapes         = {};
  vector<int>        shapes_sorting = {};
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

void update_polygon(bool_state& state, const bool_mesh& mesh, int polygon_id);

void compute_cells(bool_mesh& mesh, bool_state& state);
void compute_shapes(bool_state& state);
void compute_shape_borders(const bool_mesh& mesh, bool_state& state);
void compute_bool_operation(bool_state& state, const bool_operation& op);

vector<mesh_segment> mesh_segments(const vector<vec3i>& triangles,
    const vector<int>& strip, const vector<float>& lerps,
    const mesh_point& start, const mesh_point& end);

geodesic_path compute_geodesic_path(
    const bool_mesh& mesh, const mesh_point& start, const mesh_point& end);

void recompute_polygon_segments(const bool_mesh& mesh, const bool_state& state,
    mesh_polygon& polygon, int index = 0);

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

mesh_point intersect_mesh(const bool_mesh& mesh, const shape_bvh& bvh,
    const scene_camera& camera, const vec2f& uv);

inline mesh_point intersect_mesh(
    const bool_mesh& mesh, const scene_camera& camera, const vec2f& uv) {
  return intersect_mesh(mesh, mesh.bvh, camera, uv);
}

vec3f get_cell_color(const bool_state& state, int cell_id, bool color_shapes);

inline int genus(const bool_mesh& mesh) {
  auto num_faces    = (int)mesh.triangles.size();
  auto num_vertices = (int)mesh.positions.size();
  auto num_edges    = 0;
  for (int i = 0; i < mesh.adjacencies.size(); i++) {
    for (int k = 0; k < 3; k++) {
      if (mesh.adjacencies[i][k] != -1)
        num_edges += 1;
      else {
        return -1;
      }
    }
  }
  // V + F - E = 2 - 2g -> g = -(V + F - E - 2) / 2
  return -(num_vertices + num_faces - num_edges / 2 - 2) / 2;
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
  if (debug_stack().empty()) {
    debug_restart() = true;
    return;
  }
  while (!debug_stack().empty()) {
    auto f = debug_stack().back();
    debug_stack().pop_back();
    if (debug_visited()[f]) continue;
    face = f;
    break;
  }
  if (face == -1) return;

  debug_visited()[face] = true;

  debug_result().push_back(face);

  auto tag = mesh.borders.tags[face];
  auto adj = mesh.adjacencies[face];
  printf("\nfrom %d: tag(%d %d %d) adj(%d %d %d)\n", face, tag[0], tag[1],
      tag[2], adj[0], adj[1], adj[2]);

  for (auto neighbor : mesh.adjacencies[face]) {
    if (neighbor < 0 || debug_visited()[neighbor]) continue;
    auto tag = mesh.borders.tags[neighbor];
    auto adj = mesh.adjacencies[neighbor];
    if (check(face, neighbor)) {
      debug_stack().push_back(neighbor);
      printf("ok   %d: tag(%d %d %d) adj(%d %d %d)\n", neighbor, tag[0], tag[1],
          tag[2], adj[0], adj[1], adj[2]);
    }
    printf("no   %d: tag(%d %d %d) adj(%d %d %d)\n", neighbor, tag[0], tag[1],
        tag[2], adj[0], adj[1], adj[2]);
  }
#endif
}
