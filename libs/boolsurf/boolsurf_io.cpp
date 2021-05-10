#include <stdio.h>
#define NANOSVG_ALL_COLOR_KEYWORDS
#define NANOSVG_IMPLEMENTATION
#include "ext/nanosvg/src/nanosvg.h"
//
#include <yocto/yocto_sceneio.h>

#include "boolsurf_io.h"

bool load_json(const string& filename, json& js) {
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

bool save_test(const bool_test& test, const string& filename) {
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

bool load_test(bool_test& test, const string& filename) {
  auto js = json{};
  if (!load_json(filename, js)) {
    return false;
  }

  try {
    if (js.find("screenspace") != js.end()) {
      test.screenspace = js["screenspace"].get<bool>();
    }

    if (test.screenspace) {
      test.polygons_screenspace = js["polygons"].get<vector<vector<vec2f>>>();

      for (auto& polygon : test.polygons_screenspace) {
        auto area = 0.0f;
        for (int p = 0; p < polygon.size(); p++) {
          auto& point = polygon[p];
          auto& next  = polygon[(p + 1) % polygon.size()];
          area += cross(next, point);
        }

        if (area < 0) reverse(polygon.begin(), polygon.end());
      }

      auto bbox = bbox2f{};
      for (auto& polygon : test.polygons_screenspace)
        for (auto& p : polygon) bbox = merge(bbox, p);

      for (auto& polygon : test.polygons_screenspace)
        for (auto& p : polygon) p = (p - center(bbox)) / max(size(bbox));

    } else {
      test.points   = js["points"].get<vector<mesh_point>>();
      test.polygons = js["polygons"].get<vector<vector<int>>>();
    }

    if (js.find("operations") != js.end()) {
      test.operations = js["operations"].get<vector<bool_operation>>();
    }

    if (js.find("camera") != js.end()) {
      test.camera     = js["camera"].get<scene_camera>();
      test.has_camera = true;
    }

    if (js.find("model") != js.end()) {
      test.model = js["model"].get<string>();
    }
  } catch (std::exception& e) {
    printf("[%s]: %s\n", __FUNCTION__, e.what());
    return false;
  }
  return true;
}

bool_state state_from_test(const bool_mesh& mesh, const bool_test& test,
    float drawing_size, bool use_projection) {
  auto state   = bool_state{};
  state.points = test.points;
  state.polygons.clear();

  if (test.screenspace) {
    auto camera = make_camera(mesh);
    return make_test_state(
        test, mesh, mesh.bvh, camera, drawing_size, use_projection);
  }

  for (auto& polygon : test.polygons) {
    // Add new polygon to state.
    auto& mesh_polygon  = state.polygons.emplace_back();
    mesh_polygon.points = polygon;

    recompute_polygon_segments(mesh, state, mesh_polygon);
  }

  return state;
}

scene_model make_scene(const bool_mesh& mesh, const bool_state& state,
    const scene_camera& camera, bool color_shapes) {
  auto scene = scene_model{};
  scene.cameras.push_back(camera);

  for (int i = 0; i < state.cells.size(); i++) {
    auto& cell = state.cells[i];

    auto& instance     = scene.instances.emplace_back();
    instance.material  = (int)scene.materials.size();
    auto& material     = scene.materials.emplace_back();
    material.color     = get_cell_color(state, i, color_shapes);
    material.type      = scene_material_type::glossy;
    material.roughness = 0.5;
    instance.shape     = (int)scene.shapes.size();
    auto& shape        = scene.shapes.emplace_back();

    // TODO(giacomo): Too many copies of positions.
    shape.positions = mesh.positions;
    for (auto face : cell.faces) {
      shape.triangles.push_back(mesh.triangles[face]);
    }

    if (0) {
      auto& instance     = scene.instances.emplace_back();
      instance.shape     = (int)scene.shapes.size();
      instance.material  = (int)scene.materials.size();
      auto& material     = scene.materials.emplace_back();
      material.color     = {0, 0, 1};
      material.type      = scene_material_type::glossy;
      material.roughness = 0.5;
      auto& edges        = scene.shapes.emplace_back();
      for (auto& tr : mesh.triangles) {
        for (int k = 0; k < 3; k++) {
          auto a = tr[k];
          auto b = tr[(k + 1) % 3];
          if (a > b) continue;
          auto index = (int)edges.positions.size();
          edges.radius.push_back(0.001);
          edges.radius.push_back(0.001);
          edges.lines.push_back({index, index + 1});
          edges.positions.push_back(mesh.positions[a]);
          edges.positions.push_back(mesh.positions[b]);
        }
      }
    }
  }

  if (scene.shapes.empty()) {
    auto& instance     = scene.instances.emplace_back();
    instance.material  = (int)scene.materials.size();
    auto& material     = scene.materials.emplace_back();
    material.color     = {0.5, 0.5, 0.5};
    material.type      = scene_material_type::glossy;
    material.roughness = 0.5;
    instance.shape     = (int)scene.shapes.size();
    auto& shape        = scene.shapes.emplace_back();

    shape.positions = mesh.positions;
    shape.triangles = mesh.triangles;

    for (int i = 1; i < state.polygons.size(); i++) {
      auto& polygon   = state.polygons[i];
      auto  positions = vector<vec3f>();
      positions.reserve(polygon.length + 1);

      for (auto& edge : polygon.edges) {
        for (auto& segment : edge) {
          positions.push_back(
              eval_position(mesh, {segment.face, segment.start}));
        }
      }

      if (polygon.edges.size() && polygon.edges.back().size()) {
        auto& segment = polygon.edges.back().back();
        positions.push_back(eval_position(mesh, {segment.face, segment.end}));
      }
      if (positions.empty()) continue;

      auto lines = vector<vec2i>(positions.size() - 1);
      for (int i = 0; i < lines.size(); i++) {
        lines[i] = {i, i + 1};
      }

      auto& instance    = scene.instances.emplace_back();
      instance.material = (int)scene.materials.size();
      auto& material    = scene.materials.emplace_back();
      material.emission = {1, 0, 0};
      material.type     = scene_material_type::matte;
      instance.shape    = (int)scene.shapes.size();
      auto& shape       = scene.shapes.emplace_back();
      shape.radius      = vector<float>(positions.size(), 0.001);
      shape.positions   = positions;
      shape.lines       = lines;
    }
  }
  return scene;
}

#include <yocto/yocto_color.h>

string tree_to_string(const bool_state& state, bool color_shapes) {
  auto&  cells  = state.cells;
  string result = "digraph {\n";
  result += "forcelabels=true\n";

  for (int i = 0; i < cells.size(); i++) {
    auto& cell  = cells[i];
    auto  color = get_cell_color(state, i, color_shapes);
    color       = rgb_to_hsv(color);
    char str[1024];
    auto label = string{};
    if (state.labels.empty())
      label = "";
    else {
      for (int k = 1; k < state.labels[i].size(); k++) {
        if (state.labels[i][k] == null_label) {
          label += "0 ";
          continue;
        }
        label += to_string(state.labels[i][k]) + " ";
      }
    }
    sprintf(str, "%d [label=\"%d\n%s\" style=filled fillcolor=\"%f %f %f\"]\n",
        i, i, label.c_str(), color.x, color.y, color.z);
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

void save_tree_png(const bool_state& state, string filename,
    const string& extra, bool color_shapes) {
  if (filename.empty()) filename = "data/tests/test.json";
  auto  graph = replace_extension(filename, extra + ".txt");
  FILE* file  = fopen(graph.c_str(), "w");
  fprintf(file, "%s", tree_to_string(state, color_shapes).c_str());
  fclose(file);

  auto image = replace_extension(filename, extra + ".png");
  auto cmd   = "dot -Tpng "s + graph + " > " + image;
  printf("%s\n", cmd.c_str());
  system(cmd.c_str());
  cmd = "rm "s + graph;
  system(cmd.c_str());
}

vector<Svg_Shape> load_svg(const string& filename) {
  struct NSVGimage* image;
  image = nsvgParseFromFile(filename.c_str(), "px", 96);
  printf("svg loaded, size: %f x %f\n", image->width, image->height);
  auto size = vec2f{image->width, image->height};

  auto svg = vector<Svg_Shape>{};
  for (auto shape = image->shapes; shape != NULL; shape = shape->next) {
    auto& svg_shape = svg.emplace_back();

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
      auto  area     = 0.0f;

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

      if (area < 0.0f) {
        // std::reverse(svg_shape.paths.begin(), svg_shape.paths.end());
        // for (auto& path : svg_shape.paths) {
        std::reverse(svg_path.begin(), svg_path.end());
        for (auto& curve : svg_path) {
          curve = {curve[3], curve[2], curve[1], curve[0]};
        }
        //}
      }
      printf("area: %f\n", area);
    }
  }

  nsvgDelete(image);
  return svg;
}

void init_from_svg(bool_state& state, const bool_mesh& mesh,
    const mesh_point& center, const vector<Svg_Shape>& svg, float svg_size,
    int svg_subdivs) {
  auto p0  = eval_position(mesh, {center.face, {0, 0}});
  auto p1  = eval_position(mesh, {center.face, {1, 0}});
  auto rot = mat2f{};
  {
    auto frame = mat3f{};
    frame.x    = normalize(p1 - p0);
    frame.z    = eval_normal(mesh, center.face);
    frame.y    = normalize(cross(frame.z, frame.x));

    auto up = vec3f{0, 1, 0};
    auto v  = normalize(vec2f{dot(up, frame.x), dot(up, frame.y)});
    rot     = mat2f{{v.x, v.y}, {-v.y, v.x}};
  }

  for (auto& shape : svg) {
    for (auto& path : shape.paths) {
      auto& polygon = state.polygons.emplace_back();

      auto control_points = vector<mesh_point>{};
      // polygon.center = center;
      // polygon.frame  = mat2f{rot, vec2f{-rot.y, rot.x}};
      // polygon.color  = shape.color;
      for (auto& segment : path) {
        // polygon.curves.push_back({});
        for (int i = 0; i < 3; i++) {
          // vec2f uv = clamp(segment[i], 0.0f, 1.0f);
          vec2f uv = segment[i];
          uv -= vec2f{0.5, 0.5};
          uv = rot * uv;
          uv *= svg_size;
          auto line = straightest_path(mesh, center, uv);
          control_points += line.end;
        }
      }
      auto bezier = compute_bezier_path(mesh.dual_solver, mesh.triangles,
          mesh.positions, mesh.adjacencies, control_points, svg_subdivs);

      for (int i = 0; i < bezier.size() - 1; i++) {
        if (i > 0 && bezier[i] == bezier[i - 1]) continue;
        polygon.points += (int)state.points.size();
        state.points += bezier[i];
      }
    }
  }
}
