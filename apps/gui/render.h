#ifndef _WIN32
#include <yocto_gui/yocto_font.h>
#endif

#include <yocto_gui/yocto_shade.h>

inline void set_patch_shape(
    shade_shape* shape, const bool_mesh& mesh, const vector<int>& faces) {
  auto positions = vector<vec3f>(faces.size() * 3);
  for (int i = 0; i < faces.size(); i++) {
    auto [a, b, c]       = mesh.triangles[faces[i]];
    positions[3 * i + 0] = mesh.positions[a];
    positions[3 * i + 1] = mesh.positions[b];
    positions[3 * i + 2] = mesh.positions[c];
  }
  set_positions(shape, positions);
  set_instances(shape, {});
  shape->shape->elements = ogl_element_type::triangles;
}

inline void set_polygon_shape(
    shade_shape* shape, const bool_mesh& mesh, const mesh_polygon& polygon) {
  auto positions = vector<vec3f>();
  positions.reserve(polygon.length + 1);

  for (auto& edge : polygon.edges)
    for (auto& segment : edge)
      positions.push_back(eval_position(mesh, {segment.face, segment.start}));

  if (polygon.edges.size() && polygon.edges.back().size()) {
    auto& segment = polygon.edges.back().back();
    positions.push_back(eval_position(mesh, {segment.face, segment.end}));
  }

  if (0) {
    set_positions(shape, positions);
    set_instances(shape, {}, {});
    shape->shape->elements = ogl_element_type::line_strip;
  } else {
    auto froms = vector<vec3f>();
    auto tos   = vector<vec3f>();
    froms.reserve(positions.size() - 1);
    tos.reserve(positions.size() - 1);
    for (int i = 0; i < positions.size() - 1; i++) {
      auto from = positions[i];
      auto to   = positions[i + 1];
      if (from == to) continue;
      froms.push_back(from);
      tos.push_back(to);
    }

    float radius   = 0.003;
    auto  cylinder = make_uvcylinder({16, 1, 1}, {radius, 1});
    for (auto& p : cylinder.positions) {
      p.z = p.z * 0.5 + 0.5;
    }

    set_quads(shape, cylinder.quads);
    set_positions(shape, cylinder.positions);
    set_normals(shape, cylinder.normals);
    set_texcoords(shape, cylinder.texcoords);
    set_instances(shape, froms, tos);
  }
}

// inline void set_polygon_shape(shade_scene* scene, const bool_mesh& mesh,
//     mesh_polygon& polygon, int index) {
//   if (!polygon.polyline_shape) {
//     polygon.polyline_shape             = add_instance(scene);
//     polygon.polyline_shape->material   = add_material(scene);
//     polygon.polyline_shape->depth_test = ogl_depth_test::always;
//   }

//   if (!polygon.polyline_shape->shape) {
//     polygon.polyline_shape->shape = add_shape(scene);
//   }

//   polygon.polyline_shape->material->color = get_color(index);

//   if (polygon.length == 0) return;
// }

inline void set_border_shape(shade_scene* scene, const bool_state& state,
    const bool_mesh& mesh, mesh_shape& shape, int index) {
  if (!shape.borders_shape) {
    shape.borders_shape           = add_instance(scene);
    shape.borders_shape->material = add_material(scene);
  }

  if (!shape.borders_shape->shape) {
    shape.borders_shape->shape = add_shape(scene);
  }

  shape.borders_shape->material->color = get_color(index);

  if (shape.border_points.empty()) return;

  auto positions = vector<vec3f>();

  for (auto& border : shape.border_points) {
    for (auto p = 0; p < border.size(); p++) {
      auto start = border[p];
      auto end   = border[(p + 1) % border.size()];

      auto path = compute_geodesic_path(
          mesh, state.points[start], state.points[end]);
      auto segments = mesh_segments(
          mesh.triangles, path.strip, path.lerps, path.start, path.end);

      for (auto& segment : segments) {
        auto pos = eval_position(mesh, {segment.face, segment.start});
        positions.push_back(pos);
      }
    }
  }

  set_positions(shape.borders_shape->shape, positions);
  shape.borders_shape->shape->shape->elements = ogl_element_type::line_strip;
  set_instances(shape.borders_shape->shape, {}, {});
}

void draw_triangulation(ogl_texture* texture, int face, vec2i size) {
#ifndef _WIN32
  static int last_drawn_face = -1;
  if (face == last_drawn_face) {
    return;
  }
  last_drawn_face = face;

  auto& triangles = debug_triangles()[face];
  auto& positions = debug_nodes()[face];
  auto& indices   = debug_indices()[face];
  if (positions.empty()) return;

  auto aspect = float(size.x) / size.y;

  static opengl_font* font = nullptr;
  if (!font) {
    font = new opengl_font{};
    init_font(font, "data/Menlo-Regular.ttf", 100);
  }
  auto text_size = 0.04f;

  static ogl_shape* faces = nullptr;
  if (!faces) {
    faces = new ogl_shape{};
  }
  set_vertex_buffer(faces, positions, 0);
  set_index_buffer(faces, triangles);
  faces->elements = ogl_element_type::triangles;

  static ogl_shape* edges = nullptr;
  if (!edges) {
    edges = new ogl_shape{};
  }
  set_vertex_buffer(edges, positions, 0);
  auto lines = vector<vec2i>();
  for (auto& t : triangles) {
    lines.push_back({t.x, t.y});
    lines.push_back({t.y, t.z});
    lines.push_back({t.z, t.x});
  }
  set_index_buffer(edges, lines);
  edges->elements = ogl_element_type::lines;

  static ogl_shape* points = nullptr;
  if (!points) {
    points = new ogl_shape{};
  }
  set_vertex_buffer(points, positions, 0);
  points->elements   = ogl_element_type::points;
  points->point_size = 5;

  static ogl_program* program = nullptr;
  if (!program) {
    program          = new ogl_program{};
    auto vertex_code = R"(
      #version 330
      layout(location = 0) in vec2 positions;
      uniform float  aspect = 1.0;
      void main() {
        vec2 position = (positions - vec2(0.5)) * 2;
        position *= 0.75;
        position.x -= 0.6;
        position.y += 0.1;
        position.x /= aspect;
        gl_Position = vec4(position, 0, 1);
      }
    )";

    auto fragment_code = R"(
      #version 330
      out vec4 frag_color;
      uniform vec3 color = vec3(1,1,1);
      uniform float alpha = 1;
      void main() {
        frag_color = vec4(color, alpha);
    })";

    set_program(program, vertex_code, fragment_code, true);
  }

  if (texture->texture_id == 0 || size != texture->size) {
    set_texture(texture, size, 4, (float*)nullptr, true, true, false, false);
  }

  static ogl_framebuffer* framebuffer = nullptr;
  if (!framebuffer) {
    framebuffer = new ogl_framebuffer{};
    set_framebuffer(framebuffer, size);
    set_framebuffer_texture(framebuffer, texture, 0);
  }

  bind_framebuffer(framebuffer);
  set_ogl_viewport(size);
  clear_ogl_framebuffer({0, 0, 0.1, 1}, true);
  set_ogl_depth_test(ogl_depth_test::always);

  bind_program(program);
  set_uniform(program, "alpha", 1.0f);
  set_uniform(program, "color", vec3f{1, 1, 1});
  set_uniform(program, "aspect", aspect);
  draw_shape(edges);

  set_uniform(program, "color", vec3f{1, 0, 0});
  draw_shape(points);

  set_uniform(program, "alpha", 0.5f);
  set_uniform(program, "color", vec3f{0.5, 0.5, 0.5});
  draw_shape(faces);

  for (int i = 0; i < positions.size(); i++) {
    auto text     = to_string(i);
    auto position = (positions[i] - vec2f{0.5f, 0.5f}) * 2;
    position *= 0.75;
    position.x -= 0.6;
    position.y += 0.1;
    position.x /= aspect;
    draw_text(font, text, position.x, position.y, text_size, {0.8, 0.4, 0.1});
  }

  {
    auto lines = vector<string>{};
    auto color = vec3f{0.8, 0.4, 0.1};

    lines += "face: "s + to_string(face);
    draw_text(font, lines, -0.35, 0.9, text_size, color);

    lines.clear();
    lines += "triangles"s;
    for (int i = 0; i < triangles.size(); i++) {
      auto [a, b, c] = triangles[i];
      auto text      = "(" + to_string(a) + ", " + to_string(b) + ", " +
                  to_string(c) + ")";
      lines += text;
      // if (i > 16) {
      //   lines += "..."s;
      //   break;
      // }
    }
    draw_text(font, lines, 0.7, 0.9, text_size, color);

    lines.clear();
    lines += "indices"s;
    for (int i = 0; i < indices.size(); i++) {
      auto text = to_string(i) + ": "s;
      text += to_string(indices[i]);
      lines += text;
    }
    draw_text(font, lines, -0.1, 0.9, text_size, color);

    lines.clear();
    lines += "positions"s;
    for (int i = 0; i < positions.size(); i++) {
      auto [a, b] = positions[i];
      auto text   = to_string(i) + ": (" + to_string(a) + ", " + to_string(b) +
                  ")";
      lines += text;
    }
    draw_text(font, lines, 0.15, 0.9, text_size, color);
  }

  unbind_framebuffer();
#endif
}

inline void save_triangulation(const string& filename, int face) {
#ifndef _WIN32
  auto texture = new ogl_texture{};
  draw_triangulation(texture, face);
  auto img = get_texture(texture);

  // flip verticllay
  for (int y = 0; y < img.height / 2; y++)
    for (int x = 0; x < img.width; x++)
      std::swap(img[{x, y}], img[{x, img.height - y - 1}]);

  auto error = ""s;
  if (!save_image(filename, img, error)) {
    printf("%s: %s\n", __FUNCTION__, error.c_str());
  }
#endif
}

[[nodiscard]] shade_instance* draw_segment(shade_scene* scene,
    const bool_mesh& mesh, shade_material* material, const vec3f& start,
    const vec3f& end, float radius = 0.0006f) {
  auto cylinder = make_uvcylinder({4, 1, 1}, {radius, 1});
  for (auto& p : cylinder.positions) {
    p.z = p.z * 0.5 + 0.5;
  }

  auto shape = add_shape(scene);
  set_quads(shape, cylinder.quads);
  set_positions(shape, cylinder.positions);
  set_normals(shape, cylinder.normals);
  set_texcoords(shape, cylinder.texcoords);
  set_instances(shape, {start}, {end});
  return add_instance(scene, identity3x4f, shape, material, false);
}

[[nodiscard]] shade_instance* draw_mesh_segment(shade_scene* scene,
    const bool_mesh& mesh, shade_material* material,
    const mesh_segment& segment, float radius = 0.0012f) {
  auto start = mesh_point{segment.face, segment.start};
  auto end   = mesh_point{segment.face, segment.end};

  // draw_mesh_point(scene, mesh, material, start, radius);
  // draw_mesh_point(scene, mesh, material, end, radius);

  auto pos_start = eval_position(mesh.triangles, mesh.positions, start);
  auto pos_end   = eval_position(mesh.triangles, mesh.positions, end);

  return draw_segment(scene, mesh, material, pos_start, pos_end, radius / 2);
}

[[nodiscard]] shade_instance* draw_sphere(shade_scene* scene,
    const bool_mesh& mesh, shade_material* material, const vector<vec3f>& pos,
    float dim) {
  auto sphere = make_sphere(4, dim);

  auto shape = add_shape(scene, {}, {}, {}, sphere.quads, sphere.positions,
      sphere.normals, sphere.texcoords, {});
  set_instances(shape, pos);
  return add_instance(scene, identity3x4f, shape, material, false);
}

[[nodiscard]] shade_instance* draw_mesh_point(shade_scene* scene,
    const bool_mesh& mesh, shade_material* material, const mesh_point& point,
    float dim) {
  auto pos = eval_position(mesh.triangles, mesh.positions, point);
  return draw_sphere(scene, mesh, material, {pos}, dim);
}

#if 0

void update_path_shape(shade_shape* shape, const bool_mesh& mesh,
    const geodesic_path& path, float radius, float offset = 0,
    bool thin = true) {
  auto positions = path_positions(
      path, mesh.triangles, mesh.positions, mesh.adjacencies);

  if (thin) {
    set_positions(shape, positions);
    shape->shape->elements = ogl_element_type::line_strip;
    set_instances(shape, {}, {});
    return;
  }

  auto froms = vector<vec3f>();
  auto tos   = vector<vec3f>();
  froms.reserve(positions.size() - 1);
  tos.reserve(positions.size() - 1);
  for (int i = 0; i < positions.size() - 1; i++) {
    auto from = positions[i];
    auto to   = positions[i + 1];
    if (from == to) continue;
    froms.push_back(from);
    tos.push_back(to);
  }

  auto cylinder = make_uvcylinder({4, 1, 1}, {radius, 1});
  for (auto& p : cylinder.positions) {
    p.z = p.z * 0.5 + 0.5;
  }

  set_quads(shape, cylinder.quads);
  set_positions(shape, cylinder.positions);
  set_normals(shape, cylinder.normals);
  set_texcoords(shape, cylinder.texcoords);
  set_instances(shape, froms, tos);
}

void update_path_shape(shade_shape* shape, const bool_mesh& mesh,
    const mesh_path& path, float radius, float offset = 0, bool thin = false) {
  auto positions = vector<vec3f>(path.points.size());
  for (int i = 0; i < positions.size(); i++) {
    positions[i] = eval_position(
        mesh.triangles, mesh.positions, path.points[i]);
  }

  if (offset > 0) {
    // auto mesh_points = convert_mesh_path(mesh.triangles, mesh.adjacencies,
    // path.strip, path.lerps, path.start, path.end);
    auto pos              = positions;
    int  num_subdivisions = 8;
    for (int i = 0; i < num_subdivisions; i++) {
      auto pos = positions;
      for (int i = 0; i < pos.size(); i++) {
        auto a       = (i - 1 + (int)pos.size()) % pos.size();
        auto c       = (i + 1) % pos.size();
        positions[i] = (positions[a] + positions[c]) / 2;
      }
    }
    for (int i = 0; i < pos.size(); i++) {
      auto a  = (i - 1 + (int)pos.size()) % pos.size();
      auto b  = i;
      auto c  = (i + 1) % pos.size();
      auto n0 = eval_normal(mesh.triangles, mesh.normals, path.points[b]);
      auto v  = pos[b] - pos[a];
      auto w  = pos[c] - pos[b];
      positions[i] += offset * normalize(cross(n0, v)) / 2;
      positions[i] += offset * normalize(cross(n0, w)) / 2;
    }
  }

  if (thin) {
    set_positions(shape, positions);
    shape->shape->elements = ogl_element_type::line_strip;
    set_instances(shape, {}, {});
    return;
  }

  auto froms = vector<vec3f>();
  auto tos   = vector<vec3f>();
  froms.reserve(positions.size() - 1);
  tos.reserve(positions.size() - 1);
  for (int i = 0; i < positions.size() - 1; i++) {
    auto from = positions[i];
    auto to   = positions[i + 1];
    if (from == to) continue;
    froms.push_back(from);
    tos.push_back(to);
  }

  auto cylinder = make_uvcylinder({4, 1, 1}, {radius, 1});
  for (auto& p : cylinder.positions) {
    p.z = p.z * 0.5 + 0.5;
  }

  set_quads(shape, cylinder.quads);
  set_positions(shape, cylinder.positions);
  set_normals(shape, cylinder.normals);
  set_texcoords(shape, cylinder.texcoords);
  set_instances(shape, froms, tos);
}




[[nodiscard]] vector<shade_instance*> draw_arrangement(shade_scene* scene,
    const bool_mesh& mesh, const vector<shade_material*>& material,
    const vector<mesh_point>& points, vector<cell_polygon>& cells) {
  auto instances = vector<shade_instance*>{};
  for (auto p = 0; p < cells.size(); p++) {
    auto& polygon = cells[p];
    auto  mat     = material[p % material.size()];
    auto  path    = mesh_path{};
    for (auto n = 0; n < polygon.points.size() - 1; n++) {
      auto& start = points[polygon.points[n]];
      auto& end   = points[polygon.points[n + 1]];

      auto geo_path = compute_geodesic_path(mesh, start, end);
      append(path.points,
          convert_mesh_path(mesh.triangles, mesh.adjacencies, geo_path.strip,
              geo_path.lerps, geo_path.start, geo_path.end)
              .points);

      // auto segments = mesh_segments(mesh.triangles, geo_path.strip,
      //    geo_path.lerps, geo_path.start, geo_path.end);
      // update_mesh_polygon(polygon, segments);
    }
    auto shape = add_shape(scene);
    // TODO: Make this proportional to avg_edge_length
    float offset = 0.002f;
    update_path_shape(shape, mesh, path, 0.0010f, offset);
    instances.push_back(add_instance(scene, identity3x4f, shape, mat, false));
  }
  return instances;
}
#endif
