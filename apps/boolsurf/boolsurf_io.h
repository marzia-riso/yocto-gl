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

struct bool_test {
  string              model;
  vector<mesh_point>  points;
  vector<vector<int>> polygons;

  vector<bool_operation> operations = {};
  scene_camera           camera     = {};
  bool                   has_camera = false;
};

inline bool save_test(const bool_test& test, const string& filename) {
  auto js          = json{};
  js["points"]     = test.points;
  js["polygons"]   = test.polygons;
  js["model"]      = test.model;
  js["operations"] = test.operations;
  js["camera"]     = test.camera;

  auto error = ""s;
  if (!save_text(filename, js.dump(2), error)) {
    printf("[%s]: %s\n", __FUNCTION__, error.c_str());
    return false;
  }
  return true;
}

inline bool load_test(bool_test& test, const string& filename) {
  auto js = json{};
  if (!load_json(filename, js)) {
    return false;
  }

  try {
    test.points   = js["points"].get<vector<mesh_point>>();
    test.polygons = js["polygons"].get<vector<vector<int>>>();
    if (js.find("operations") != js.end()) {
      test.operations = js["operations"].get<vector<bool_operation>>();
    }
    if (js.find("camera") != js.end()) {
      test.camera     = js["camera"].get<scene_camera>();
      test.has_camera = true;
    }
    test.model = js["model"].get<string>();
  } catch (std::exception& e) {
    printf("[%s]: %s\n", __FUNCTION__, e.what());
    return false;
  }
  return true;
}

bool_state state_from_test(const bool_mesh& mesh, const bool_test& test) {
  auto state   = bool_state{};
  state.points = test.points;
  for (auto& polygon : test.polygons) {
    // Add new polygon to state.
    auto& mesh_polygon  = state.polygons.emplace_back();
    mesh_polygon.points = polygon;
  }

  for (auto& mesh_polygon : state.polygons) {
    for (int i = 0; i < mesh_polygon.points.size(); i++) {
      auto start = mesh_polygon.points[i];
      auto end   = mesh_polygon.points[(i + 1) % mesh_polygon.points.size()];
      auto path  = compute_geodesic_path(
          mesh, state.points[start], state.points[end]);
      auto segments = mesh_segments(
          mesh.triangles, path.strip, path.lerps, path.start, path.end);
      mesh_polygon.segments += segments;
    }
  }
  return state;
}

#include <yocto/yocto_color.h>

inline string tree_to_string(const vector<mesh_cell>& cells) {
  string result = "digraph {\n";
  result += "forcelabels=true\n";

  for (int i = 0; i < cells.size(); i++) {
    auto& cell  = cells[i];
    auto  color = rgb_to_hsv(get_cell_color(cell.labels, i));
    char  str[1024];
    sprintf(str, "%d [label=\"%d\" style=filled fillcolor=\"%f %f %f\"]\n", i,
        i, color.x, color.y, color.z);
    result += std::string(str);

    for (auto [neighbor, polygon] : cell.adjacency) {
      if (polygon < 0) continue;
      int  c     = neighbor;
      auto color = rgb_to_hsv(get_color(polygon));
      sprintf(str, "%d -> %d [ label=\"%d\" color=\"%f %f %f\"]\n", i, c,
          polygon, color.x, color.y, color.z);
      result += std::string(str);
    }
  }
  result += "}\n";
  return result;
}

inline void save_tree_png(
    const bool_state& state, string filename, const string& extra = "") {
  if (filename.empty()) filename = "data/tests/test.json";
  auto  graph = replace_extension(filename, extra + ".txt");
  FILE* file  = fopen(graph.c_str(), "w");
  fprintf(file, "%s", tree_to_string(state.cells).c_str());
  fclose(file);

  auto image = replace_extension(filename, extra + ".png");
  auto cmd   = "dot -Tpng "s + graph + " > " + image;
  printf("%s\n", cmd.c_str());
  system(cmd.c_str());
}

#define NANOSVG_ALL_COLOR_KEYWORDS
#define NANOSVG_IMPLEMENTATION
#include "ext/nanosvg/src/nanosvg.h"

using Svg_Path = vector<array<vec2f, 4>>;
struct Svg_Shape {
  vec3f            color = {};
  vector<Svg_Path> paths = {};
};

vector<Svg_Shape> load_svg(const string& filename) {
  struct NSVGimage* image;
  image = nsvgParseFromFile(filename.c_str(), "px", 96);
  printf("svg loaded, size: %f x %f\n", image->width, image->height);
  auto size = vec2f{image->width, image->height};

  auto svg = vector<Svg_Shape>{};
  for (auto shape = image->shapes; shape != NULL; shape = shape->next) {
    auto& svg_shape = svg.emplace_back();
    auto  area      = 0.0f;

    unsigned int c;
    if (shape->fill.type == NSVG_PAINT_COLOR) {
      c = shape->fill.color;
    } else if (shape->fill.type >= NSVG_PAINT_LINEAR_GRADIENT) {
      c = shape->fill.gradient->stops[0].color;
    } else {
      c = 0;
    }
    float r         = ((c >> 16) & 0xFF) / 255.0;  // Extract the RR byte
    float g         = ((c >> 8) & 0xFF) / 255.0;   // Extract the GG byte
    float b         = ((c)&0xFF) / 255.0;
    svg_shape.color = yocto::pow(vec3f{b, g, r}, 2.2f);

    for (auto path = shape->paths; path != NULL; path = path->next) {
      auto& svg_path = svg_shape.paths.emplace_back();
      for (int i = 0; i < path->npts - 1; i += 3) {
        float* p     = &path->pts[i * 2];
        auto&  curve = svg_path.emplace_back();
        curve[0]     = vec2f{p[0], size.y - p[1]} / size.y;
        curve[1]     = vec2f{p[2], size.y - p[3]} / size.y;
        curve[2]     = vec2f{p[4], size.y - p[5]} / size.y;
        curve[3]     = vec2f{p[6], size.y - p[7]} / size.y;

        area += cross(curve[0], curve[1]);
        area += cross(curve[1], curve[2]);
        area += cross(curve[2], curve[3]);
        // printf("(%f %f) (%f %f) (%f %f) (%f %f)\n", curve[0].x, curve[0].y,
        //     curve[1].x, curve[1].y, curve[2].x, curve[2].y, curve[3].x,
        //     curve[3].y);
      }
    }
    if (area < 0) {
      std::reverse(svg_shape.paths.begin(), svg_shape.paths.end());
      for (auto& path : svg_shape.paths) {
        std::reverse(path.begin(), path.end());
        for (auto& curve : path) {
          curve = {curve[3], curve[2], curve[1], curve[0]};
        }
      }
    }
    printf("area: %f\n", area);
  }

  nsvgDelete(image);
  return svg;
}

void init_from_svg(bool_state& state, const bool_mesh& mesh,
    const mesh_point& center, const vector<Svg_Shape>& svg, float svg_size) {
  auto p0    = eval_position(mesh, {center.face, {0, 0}});
  auto p1    = eval_position(mesh, {center.face, {0, 1}});
  auto v     = normalize(p1 - p0);
  auto frame = basis_fromz(eval_normal(mesh, {center.face, {0, 0}}));
  // auto rot   = vec2f{dot(v, frame.x), dot(v, frame.y)};
  //
  // app.commit_state();
  // app.splines() = {};

  for (auto& shape : svg) {
    for (auto& path : shape.paths) {
      auto& polygon = state.polygons.emplace_back();
      // polygon.center = center;
      // polygon.frame  = mat2f{rot, vec2f{-rot.y, rot.x}};
      // polygon.color  = shape.color;
      for (auto& segment : path) {
        // polygon.curves.push_back({});
        for (int i = 0; i < 3; i++) {
          // vec2f uv = clamp(segment[i], 0.0f, 1.0f);
          vec2f uv = segment[i];
          uv -= vec2f{0.5, 0.5};
          uv *= svg_size;
          auto line = straightest_path(mesh, center, uv);

          polygon.points.push_back((int)state.points.size());
          auto& point = state.points.emplace_back();
          point       = line.end;
        }
      }
      auto& segment = path.back();
      // vec2f uv      = clamp(segment[3], 0.0f, 1.0f);
      vec2f uv = segment[3];
      uv -= vec2f{0.5, 0.5};
      uv *= svg_size;
      auto line = straightest_path(mesh, center, uv);

      polygon.points.push_back((int)state.points.size());
      auto& point = state.points.emplace_back();
      point       = line.end;
    }
  }
}
