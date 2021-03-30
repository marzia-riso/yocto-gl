#include <yocto/yocto_cli.h>
#include <yocto/yocto_trace.h>

#include "boolsurf.h"
#include "ext/json.hpp"

namespace yocto {

using json = nlohmann::json;
using std::array;

// support for json conversions
inline void to_json(json& j, const vec2f& value) {
  nlohmann::to_json(j, (const array<float, 2>&)value);
}

inline void from_json(const json& j, vec2f& value) {
  nlohmann::from_json(j, (array<float, 2>&)value);
}

inline void to_json(json& j, const frame3f& value) {
  nlohmann::to_json(j, (const array<float, 12>&)value);
}

inline void from_json(const json& j, frame3f& value) {
  nlohmann::from_json(j, (array<float, 12>&)value);
}

inline void to_json(json& js, const mesh_point& value) {
  js["face"] = value.face;
  js["uv"]   = value.uv;
}

inline void from_json(const json& js, mesh_point& value) {
  js.at("face").get_to(value.face);
  js.at("uv").get_to(value.uv);
}

inline void to_json(json& js, const bool_operation& op) {
  js["a"] = op.shape_a;
  js["b"] = op.shape_b;
  // js["type"] = bool_operation::type_names[(int)op_shape];
  js["type"] = (int)op.type;
}

inline void from_json(const json& js, bool_operation& op) {
  js.at("a").get_to(op.shape_a);
  js.at("b").get_to(op.shape_b);
  js.at("type").get_to(op.type);
}

inline void to_json(json& js, const scene_camera& camera) {
  js["frame"]        = camera.frame;
  js["orthographic"] = camera.orthographic;
  js["lens"]         = camera.lens;
  js["film"]         = camera.film;
  js["aspect"]       = camera.aspect;
  js["focus"]        = camera.focus;
  js["aperture"]     = camera.aperture;
}

inline void from_json(const json& js, scene_camera& camera) {
  js.at("frame").get_to(camera.frame);
  js.at("orthographic").get_to(camera.orthographic);
  js.at("lens").get_to(camera.lens);
  js.at("film").get_to(camera.film);
  js.at("aspect").get_to(camera.aspect);
  js.at("focus").get_to(camera.focus);
  js.at("aperture").get_to(camera.aperture);
}
}  // namespace yocto

bool load_json(const string& filename, json& js);

struct bool_test {
  string                model;
  vector<mesh_point>    points;
  vector<vector<int>>   polygons;
  vector<vector<vec2f>> polygons_screenspace;

  vector<bool_operation> operations  = {};
  scene_camera           camera      = {};
  bool                   has_camera  = false;
  bool                   screenspace = false;
};

bool save_test(const bool_test& test, const string& filename);
bool load_test(bool_test& test, const string& filename);

bool_state state_from_test(const bool_mesh& mesh, const bool_test& test);

string tree_to_string(const bool_state& state, bool color_shapes);

void save_tree_png(const bool_state& state, string filename,
    const string& extra, bool color_shapes);

using Svg_Path = vector<array<vec2f, 4>>;
struct Svg_Shape {
  vec3f            color = {};
  vector<Svg_Path> paths = {};
};

vector<Svg_Shape> load_svg(const string& filename);

void init_from_svg(bool_state& state, const bool_mesh& mesh,
    const mesh_point& center, const vector<Svg_Shape>& svg, float svg_size);

inline bool_state make_test_state(const bool_test& test, const bool_mesh& mesh,
    const shape_bvh& bvh, const scene_camera& camera, float svg_size) {
  auto state    = bool_state{};
  auto polygons = test.polygons_screenspace;

  for (auto& polygon : polygons) {
    auto area = 0.0f;
    for (int p = 0; p < polygon.size(); p++) {
      auto& point = polygon[p];
      auto& next  = polygon[(p + 1) % polygon.size()];
      area += cross(next, point);
    }

    if (area < 0) {
      std::reverse(polygon.begin(), polygon.end());
    }
  }

  auto bbox = bbox2f{};
  for (auto& polygon : polygons) {
    for (auto& p : polygon) {
      bbox = merge(bbox, p);
    }
  }

  for (auto& polygon : polygons) {
    for (auto& p : polygon) {
      p = (p - center(bbox)) / max(size(bbox));
    }
  }

  auto rng    = make_rng(0);
  auto ss     = vec2f{0.5, 0.5};
  auto size   = 0.1f;
  auto center = intersect_mesh(mesh, camera, ss);

  // print("center", center);
  while (center.face == -1) {
    // || center.uv == zero2f || center.uv == vec2f{1, 0} ||
    //       center.uv == vec2f{0, 1}) {
    ss = vec2f{0.5, 0.5} + (rand2f(rng) - vec2f{0.5, 0.5}) * size;
    // print("ss", ss);
    center = intersect_mesh(mesh, camera, ss);
    // print("center", center);
    size += 0.001;
  }
  assert(center.face != -1);
  center.uv = clamp(center.uv, 0.01, 0.99);

  for (auto& polygon : polygons) {
    state.polygons.push_back({});
    auto polygon_id = (int)state.polygons.size() - 1;

    for (auto uv : polygon) {
      uv.x /= camera.film;                    // input.window_size.x;
      uv.y /= (camera.film / camera.aspect);  // input.window_size.y;
      uv *= svg_size;
      uv.x = -uv.x;

      auto path     = straightest_path(mesh, center, uv);
      path.end.uv.x = clamp(path.end.uv.x, 0.0f, 1.0f);
      path.end.uv.y = clamp(path.end.uv.y, 0.0f, 1.0f);
      // check_point(path.end);
      auto point = path.end;

      // Add point to state.
      state.polygons[polygon_id].points.push_back((int)state.points.size());

      for (auto& p : state.points) {
        assert(!(p.face == point.face && p.uv == point.uv));
      }
      state.points.push_back(point);
    }

    if (state.polygons[polygon_id].points.size() <= 2) {
      assert(0);
      state.polygons[polygon_id].points.clear();
      continue;
    }

    recompute_polygon_segments(mesh, state, state.polygons[polygon_id]);
  }

  return state;
}

inline scene_camera make_camera(const bool_mesh& mesh, int seed = 0) {
  auto bbox_size = size(mesh.bbox);
  auto z         = zero3f;
  if (bbox_size.x <= bbox_size.y && bbox_size.x <= bbox_size.z) {
    z = {1, 0, 0};
  }
  if (bbox_size.y <= bbox_size.x && bbox_size.y <= bbox_size.z) {
    z = {0, 1, 0};
  }
  if (bbox_size.z <= bbox_size.x && bbox_size.z <= bbox_size.y) {
    z = {0, 0, 1};
  }

  // auto x = vec3f{1, 0, 0};
  auto x = zero3f;
  if (bbox_size.x >= bbox_size.y && bbox_size.x >= bbox_size.z) {
    x = {1, 0, 0};
  }
  if (bbox_size.y >= bbox_size.x && bbox_size.y >= bbox_size.z) {
    x = {0, 1, 0};
  }
  if (bbox_size.z >= bbox_size.x && bbox_size.z >= bbox_size.y) {
    x = {0, 0, 1};
  }

  auto up = -cross(x, z);
  // if (up == z) up = {1, 0, 0};
  auto camera = scene_camera{};
  if (seed == 0) {
    camera.frame = lookat_frame(3 * z + 2 * x, zero3f, up);
  } else {
    auto rng     = make_rng(seed);
    auto dir     = sample_hemisphere(rand2f(rng));
    camera.frame = lookat_frame(3 * dir, zero3f, up);
  }
  return camera;
}
