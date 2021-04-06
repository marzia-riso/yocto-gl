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

inline bool operator==(const mesh_point& a, const mesh_point& b) {
  return (a.face == b.face) && (a.uv == b.uv);
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
// (marzia): check name
template <class T, typename F>
inline int find_where(const vector<T>& vec, F&& f) {
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

inline vec2i get_mesh_edge_from_index(const vec3i& triangle, int k) {
  if (k == 0) return {triangle.x, triangle.y};
  if (k == 1) return {triangle.y, triangle.z};
  if (k == 2) return {triangle.z, triangle.x};

  assert(0);
  return {-1, -1};
}

inline vec2i get_triangle_edge_from_index(int k) {
  if (k == 0) return {0, 1};
  if (k == 1) return {1, 2};
  if (k == 2) return {2, 0};

  assert(0);
  return {-1, -1};
}

inline vec2i get_edge_from_uv(const vec2f& uv) {
  if (uv.y == 0) return {0, 1};
  if (fabs(uv.x + uv.y - 1.0f) < 0.00001)
    return {1, 2};  // (marzia): cambiata epsilon, occhio!
  if (uv.x == 0) return {2, 0};

  assert(0);
  return {-1, -1};
};

inline pair<int, float> get_edge_lerp_from_uv(const vec2f& uv) {
  if (uv.y == 0) return {0, uv.x};
  if (uv.x == 0) return {2, 1.0f - uv.y};
  if (fabs(uv.x + uv.y - 1.0f) < 0.00001) return {1, uv.y};

  return {-1, -1};
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

namespace std {
inline string to_string(const vec3i& v) {
  return "{" + std::to_string(v.x) + ", " + std::to_string(v.y) + ", " +
         std::to_string(v.z) + "}";
}

inline string to_string(const vec3f& v) {
  return "{" + std::to_string(v.x) + ", " + std::to_string(v.y) + ", " +
         std::to_string(v.z) + "}";
}

inline string to_string(const vec2i& v) {
  return "{" + std::to_string(v.x) + ", " + std::to_string(v.y) + "}";
}

inline string to_string(const vec2f& v) {
  return "{" + std::to_string(v.x) + ", " + std::to_string(v.y) + "}";
}

inline string to_string(const mesh_point& p) {
  return "{" + std::to_string(p.face) + ", " + std::to_string(p.uv) + "}";
}
}  // namespace std

template <typename T>
void print(const string& name, const T& v) {
  printf("%s: %s\n", name.c_str(), std::to_string(v).c_str());
}

template <typename T>
void print(const string& name, const vector<T>& vec, int max_elements = 100) {
  printf("[size: %lu] ", vec.size());
  printf("%s: [", name.c_str());
  if (vec.empty()) {
    printf("]\n");
    return;
  }
  for (int i = 0; i < min((int)vec.size() - 1, max_elements); i++) {
    printf("%s, ", std::to_string(vec[i]).c_str());
  }
  if (vec.size() > max_elements) {
    printf("...]");
  } else {
    printf("%s]", std::to_string(vec.back()).c_str());
  }
  printf("\n");
}

namespace yocto {
struct ogl_texture;
}

void draw_triangulation(
    ogl_texture* texture, int face, vec2i size = {2048, 2048});

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

template <class K, class V>
inline bool contains(const hash_map<K, V>& map, const K& x) {
  return map.find(x) != map.end();
}

template <class T>
inline bool contains(const hash_set<T>& set, const T& x) {
  return set.find(x) != set.end();
}

template <class T>
inline bool contains(const vector<T>& vec, const T& x) {
  return find(vec.begin(), vec.end(), x) != vec.end();
}

#ifdef MY_DEBUG
hash_map<int, vector<vec3i>>& debug_triangles();
hash_map<int, vector<vec2i>>& debug_edges();
hash_map<int, vector<vec2f>>& debug_nodes();
hash_map<int, vector<int>>&   debug_indices();

vector<int>&  debug_result();
vector<bool>& debug_visited();
vector<int>&  debug_stack();
bool&         debug_restart();
#endif
