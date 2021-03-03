#pragma once

#include <yocto/yocto_mesh.h>
#include <yocto/yocto_scene.h>
#include <yocto/yocto_shape.h>  // hashing vec2i

#include <cassert>
using namespace yocto;

inline int mod3(int i) { return (i > 2) ? i - 3 : i; }

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
  return c;
}

template <typename T>
inline void insert(vector<T>& vec, size_t i, const T& x) {
  vec.insert(vec.begin() + i, x);
}

// TODO(giacomo): Expose this function in yocto_mesh.h
inline int find_in_vec(const vec3i& vec, int x) {
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

inline vec2i make_edge_key(const vec2i& edge) {
  if (edge.x > edge.y) return {edge.y, edge.x};
  return edge;
};

// (marzia) Not Used
inline std::tuple<vec2i, float> get_mesh_edge(
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

inline vec3f get_color(int i) {
  static auto colors = vector<vec3f>{
      {0.5, 0.5, 0.5},
      {1, 0, 0},
      {0, 0.5, 0},
      {0, 0, 1},
      {0, 0.5, 0.5},
      {1, 0.5, 0},
      {0.5, 0, 1},
      {0.5, 0, 0},
      {0, 0.5, 0},
      {0, 0, 0.5},
      {0, 0.5, 0.5},
      {0.5, 0.5, 0},
      {0.5, 0, 0.5},
  };

  return colors[i % colors.size()];
}

inline vec3f get_cell_color(const vector<int>& labels, int cell_id) {
  auto color = vec3f{0, 0, 0};
  int  count = 0;
  for (int p = 0; p < labels.size(); p++) {
    auto label = labels[p];
    if (label > 0) {
      color += get_color(p);
      count += 1;
    }
  }
  if (count > 0) {
    color /= count;
    color += vec3f{1, 1, 1} * 0.1f * yocto::sin(cell_id);
  } else {
    color = {0.9, 0.9, 0.9};
  }
  return color;
}

#if 0
#include "ext/robin_hood.h"
template <typename Key, typename Value>
using hash_map = robin_hood::unordered_flat_map<Key, Value>;

template <typename Key>
using hash_set = robin_hood::unordered_flat_set<Key>;
#else

#include <unordered_set>
template <typename Key, typename Value>
using hash_map = std::unordered_map<Key, Value>;

template <typename Key>
using hash_set = std::unordered_set<Key>;
#endif

#ifdef MY_DEBUG
static auto debug_triangles = hash_map<int, vector<vec3i>>{};
static auto debug_edges     = hash_map<int, vector<vec2i>>{};
static auto debug_nodes     = hash_map<int, vector<vec2f>>{};
static auto debug_indices   = hash_map<int, vector<int>>{};

static auto debug_result  = vector<int>();
static auto debug_visited = vector<bool>{};
static auto debug_stack   = vector<int>{};
static auto debug_restart = true;
#endif
