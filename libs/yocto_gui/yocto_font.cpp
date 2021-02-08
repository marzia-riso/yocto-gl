#ifdef __APPLE__
#define GL_SILENCE_DEPRECATION
#endif
#include "ext/glad/glad.h"
//
#include <ft2build.h>

#include <cassert>
#include FT_FREETYPE_H
#include FT_OUTLINE_H

#include "yocto_font.h"
using namespace yocto;

// From opengl tutorial <3
// https://learnopengl.com/In-Practice/Text-Rendering

auto vertex = R"(
  #version 330 core
  layout(location = 0) in vec2 vertex;
  out vec2     TexCoords;
  uniform vec2 scale  = vec2(1, 1);
  uniform vec2 center = vec2(0.0, 0.0);

  void main() {
    TexCoords = vec2(vertex.x, 1 - vertex.y);
    vec2 p = vertex;
    p *= scale;
    p += center;
    gl_Position = vec4(p.x, p.y, 0, 1);
  }
)";

auto fragment = R"(
  #version 330 core
  in vec2 TexCoords;
  out vec4 out_color;

  uniform sampler2D text;
  uniform vec3 color = vec3(1, 1, 1);
  uniform float alpha = 1;

  void main() {    
      float sampled = texture(text, TexCoords).r;
      out_color = vec4(color, alpha * sampled);
  }
)";

void init_glfont(opengl_font& font, const string& filename, float size) {
  auto error    = string{};
  auto errorlog = string{};
  if (!set_program(font.program, vertex, fragment, error, errorlog)) {
    printf("%s: %s\n", __FUNCTION__, error.c_str());
    printf("%s: %s\n", __FUNCTION__, errorlog.c_str());
  }

  font.size = size;

  // FreeType
  FT_Library ft;
  if (FT_Init_FreeType(&ft))
    printf("ERROR::FREETYPE: Could not init FreeType Library\n");

  // Load font as face
  FT_Face face;
  if (FT_New_Face(ft, filename.c_str(), 0, &face))
    printf("ERROR::FREETYPE: Failed to load font %s\n", filename.c_str());

  // Set size to load glyphs as
  FT_Set_Pixel_Sizes(face, 0, (int)font.size);

  // Disable byte-alignment restriction
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

  // Load first 128 characters of ASCII set
  for (int c = 0; c < 128; c++) {
    // Load character glyph
    if (FT_Load_Char(face, c, FT_LOAD_RENDER)) {
      printf("%s: ERROR::FREETYTPE: Failed to load Glyph\n", __FUNCTION__);
      continue;
    }

    // Add entry to hash map.
    auto& character = font.characters[(char)c];
    character.size  = {
        int(face->glyph->bitmap.width), int(face->glyph->bitmap.rows)};
    character.bearing = {face->glyph->bitmap_left, face->glyph->bitmap_top};
    character.advance = (GLuint)face->glyph->advance.x;

    // Generate texture
    assert(glGetError() == GL_NO_ERROR);
    auto& texture = character.texture_id;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, face->glyph->bitmap.width,
        face->glyph->bitmap.rows, 0, GL_RED, GL_UNSIGNED_BYTE,
        face->glyph->bitmap.buffer);

    // Set texture options
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    assert(glGetError() == GL_NO_ERROR);
  }
  glBindTexture(GL_TEXTURE_2D, 0);

  // Destroy FreeType
  FT_Done_Face(face);
  FT_Done_FreeType(ft);

  // clang-format off
  static const auto positions = vector<vec3f>{
    {0, 0}, {1, 0}, {1, 1}, {0, 1},
  };
  static const auto triangles = vector<vec3i>{
    {0, 1, 3}, {3, 2, 1}
  };
  // clang-format on
  set_vertex_buffer(font.quad, positions, 0);
  set_index_buffer(font.quad, triangles);
}

void draw_glfont(const opengl_font& font, const string& text, float x, float y,
    float scale, const vec3f& color, float alpha) {
  auto shader = font.program;

  set_ogl_blending(true);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  bind_program(shader);
  set_uniform(shader, "color", color);
  set_uniform(shader, "alpha", alpha);
  assert(glGetError() == GL_NO_ERROR);
  glActiveTexture(GL_TEXTURE0);

  float aspect = 1.0f;
  {
    auto size = get_ogl_viewport_size();
    aspect    = float(size.x) / size.y;
  }

  for (int i = 0; i < text.size(); ++i) {
    auto& ch = font.characters.at(text[i]);

    // printf("\ncharacter[%d]: %c\n", i, text[i]);
    // printf("size: %d, %d\n", ch.size.x, ch.size.y);
    // printf("bearing: %d, %d\n", ch.bearing.x, ch.bearing.y);
    // printf("advance: %d\n", (int)ch.advance);

    float xpos = x + ch.bearing.x * scale;
    float ypos = y - (ch.size.y - ch.bearing.y) * scale;

    float width  = ch.size.x * scale;
    float height = ch.size.y * scale;

    auto center  = vec2f{xpos, ypos};
    auto scaling = vec2f{width / aspect, height};

    set_uniform(shader, "center", center);
    set_uniform(shader, "scale", scaling);

    // auto scale = vec2f{width * 0.5f, height * 0.5f} * dxy;
    // auto smat = mat2f{{scale.x, 1}, {1, scale.y}};
    // smat.x.x /= ratio;
    // auto mat = rot * smat;
    // set_uniform(shader, "mat", mat);

    glBindTexture(GL_TEXTURE_2D, ch.texture_id);
    draw_shape(font.quad);
    assert(glGetError() == GL_NO_ERROR);

    // Advance cursors for next glyph (advance is number of 1/64 pixels)
    // line_pos.x += (ch.advance / 64.0);
    x += (ch.advance >> 6) * scale / aspect;
  }
}
