#include <yocto/yocto_commonio.h>

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

// support for json conversions
inline void to_json(json& js, const mesh_point& value) {
  js["face"] = value.face;
  js["uv"]   = value.uv;
}

inline void from_json(const json& js, mesh_point& value) {
  js.at("face").get_to(value.face);
  js.at("uv").get_to(value.uv);
}
}  // namespace yocto

inline bool load_json(const string& filename, json& js) {
  // error helpers
  auto error = ""s;
  auto text  = ""s;
  if (!load_text(filename, text, error)) {
    printf("[%s]: %s\n", __FUNCTION__, error.c_str());
    return false;
  }
  try {
    js = json::parse(text);
    return true;
  } catch (std::exception& e) {
    printf("[%s]: %s\n", __FUNCTION__, e.what());
    return false;
  }
}

inline bool save_polygons(const string& filename,
    const vector<mesh_point>& points, const vector<vector<int>>& polygons) {
  auto js        = json{};
  js["points"]   = points;
  js["polygons"] = polygons;

  auto error = ""s;
  if (!save_text(filename, js.dump(2), error)) {
    printf("[%s]: %s\n", __FUNCTION__, error.c_str());
    return false;
  }
  return true;
}

inline std::pair<vector<mesh_point>, vector<vector<int>>> load_polygons(
    const string& filename) {
  auto js = json{};
  if (!load_json(filename, js)) {
    return {};
  }

  auto points   = vector<mesh_point>{};
  auto polygons = vector<vector<int>>{};
  try {
    points   = js["points"].get<vector<mesh_point>>();
    polygons = js["polygons"].get<vector<vector<int>>>();
  } catch (std::exception& e) {
    printf("[%s]: %s\n", __FUNCTION__, e.what());
    return {};
  }
  return {points, polygons};
}