#pragma once

#include <yocto/yocto_mesh.h>

using namespace yocto;

struct bezier_mesh {
  vector<vec3i>        triangles   = {};
  vector<vec3i>        adjacencies = {};
  vector<vec3f>        positions   = {};
  vector<vec3f>        normals     = {};
  dual_geodesic_solver dual_solver = {};
};

inline geodesic_path compute_geodesic_path(
    const bezier_mesh& mesh, const mesh_point& start, const mesh_point& end) {
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