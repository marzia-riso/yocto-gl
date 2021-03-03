#include <yocto/yocto_cli.h>
#include <yocto/yocto_sampling.h>
#include <yocto/yocto_sceneio.h>
#include <yocto/yocto_shape.h>
#include <yocto/yocto_trace.h>

#include "boolsurf.h"
#include "boolsurf_io.h"
using namespace yocto;

vector<mesh_point> sample_points(const bool_mesh& mesh, const shape_bvh& bvh,
    const vec3f& camera_from, const bbox3f& bbox, const vec3f& camera_to,
    float camera_lens, float camera_aspect, uint64_t trial, int num_points = 4,
    int ray_trials = 10000) {
  // init data
  auto points  = vector<mesh_point>{};
  auto rng_ray = make_rng(9867198237913, trial * 2 + 1);
  // try to pick in the camera
  auto  ray_trial = 0;
  float aspect    = size(bbox).x / size(bbox).y;
  auto  uvs       = vector<vec2f>{};
  while (points.size() < num_points) {
    if (ray_trial++ >= ray_trials) break;
    auto uv = rand2f(rng_ray);
    if (points.size()) {
      uv = 2 * uv - vec2f{1, 1};
      uv *= min(aspect, 1 / aspect);
      uv += 2 * uvs[0] - vec2f{1, 1};
      uv = uv * 0.5 + vec2f{0.5, 0.5};
    }

    auto ray  = camera_ray(lookat_frame(camera_from, camera_to, {0, 1, 0}),
        camera_lens, camera_aspect, 0.036f, uv);
    auto isec = intersect_triangles_bvh(
        bvh, mesh.triangles, mesh.positions, ray);
    if (!isec.hit) continue;
    if (isec.element < 0 || isec.element > mesh.triangles.size()) continue;
    if (!(isec.uv.x >= 0 && isec.uv.x <= 1)) continue;
    if (!(isec.uv.y >= 0 && isec.uv.y <= 1)) continue;
    points.push_back({isec.element, isec.uv});
    uvs.push_back(uv);
  }
  // pick based on area
  auto rng_area = make_rng(9867198237913);
  auto cdf      = sample_triangles_cdf(mesh.triangles, mesh.positions);
  while (points.size() < num_points) {
    auto [triangle, uv] = sample_triangles(
        cdf, rand1f(rng_area), rand2f(rng_area));
    points.push_back({mesh_point{triangle, uv}});
  }
  return points;
}

int main(int num_args, const char* args[]) {
  auto test_filename   = "data/tests/test.json"s;
  auto output_filename = "data/render.png"s;

  // parse command line
  auto cli = make_cli("test", "test boolsurf algorithms");
  add_argument(cli, "input", test_filename, "Input test filename (.json).");
  add_option(
      cli, "--output/-o", output_filename, "Output image filename (.png).");
  parse_cli(cli, num_args, args);

  auto test = bool_test{};
  if (!load_test(test, test_filename)) {
    print_fatal("Error loading test " + test_filename);
  }

  auto          mesh = bool_mesh{};
  vector<vec2f> texcoords;
  vector<vec3f> colors;
  string        error;

  if (!load_shape(test.model, mesh, error)) {
    printf("%s\n", error.c_str());
    print_fatal("Error loading model " + test_filename);
  }
  init_mesh(mesh);
  printf("triangles: %d\n", (int)mesh.triangles.size());
  printf("positions: %d\n\n", (int)mesh.positions.size());

  auto state = state_from_test(mesh, test);

  {
    auto timer = print_timed("[compute_cells]");
    compute_cells(mesh, state);
    compute_shapes(state);
  }

  for (auto& operation : test.operations) {
    compute_bool_operation(state, operation);
  }

  auto  scene  = scene_scene{};
  auto& camera = scene.cameras.emplace_back();
  camera       = test.camera;

  for (int i = 0; i < state.cells.size(); i++) {
    auto& cell = state.cells[i];

    auto& instance    = scene.instances.emplace_back();
    instance.material = (int)scene.materials.size();
    auto& material    = scene.materials.emplace_back();
    auto  shape_id    = 0;
    for (int s = (int)state.shapes.size() - 1; s >= 0; s--) {
      if (state.shapes[s].cells.count(i)) {
        shape_id = s;
        break;
      }
    }

    material.color     = get_color(shape_id);
      material.type = material_type::plastic;
    material.roughness = 0.2;
    instance.shape     = (int)scene.shapes.size();
    auto& shape        = scene.shapes.emplace_back();

    // TODO(giacomo): Too many copies of positions.
    shape.positions = mesh.positions;
    for (auto face : cell.faces) {
      shape.triangles.push_back(mesh.triangles[face]);
    }
    shape.normals = compute_normals(shape);
  }

#if 0
  // auto light_material      = add_material(scene);
  // light_material.emission = {40, 40, 40};

  // auto light_shape       = add_shape(scene);
  // auto quad_shape        = make_rect({1, 1}, {0.2, 0.2});
  // light_shape->quads     = quad_shape.quads;
  // light_shape->positions = quad_shape.positions;
  // light_shape->normals   = quad_shape.normals;
  // light_shape->texcoords = quad_shape.texcoords;

  // for (auto p : {vec3f{-2, 2, 2}, vec3f{2, 2, 1}, vec3f{0, 2, -2}}) {
  //   auto ist      = add_instance(scene);
  //   ist->frame    = lookat_frame(p, {0, 0.5, 0}, {0, 1, 0}, true);
  //   ist->shape    = light_shape;
  //   ist->material = light_material;
  // }

#endif
  auto params    = trace_params{};
  params.sampler = trace_sampler_type::eyelight;
  params.samples = 16;
  auto image     = trace_image(scene, params);
  save_image(output_filename, image, error);
}
