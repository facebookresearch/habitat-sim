----------------------------------------------------------------------
 habitat-sim/data/fonts/README.txt
 This is the Readme dedicated to fonts.Right now just credits
 Tristan Grimmer for 'ProggyClean.ttf'and gives and example
 of how to use it in python with Magnum
----------------------------------------------------------------------

---------------------------------------
 CREDITS/LICENSES FOR FONTS INCLUDED IN THIS FOLDER
---------------------------------------

ProggyClean.ttf

  Copyright (c) 2004, 2005 Tristan Grimmer
  MIT License
  recommended loading setting: Size = 13.0, DisplayOffset.Y = +1
  http://www.proggyfonts.net/

---------------------------------------
FONTS PYTHON EXAMPLE WITH MAGNUM
---------------------------------------
import string
import magnum as mn
from magnum import shaders, text

.
.
.

DISPLAY_FONT_SIZE = 16.0
viewport_size: mn.Vector2i = mn.gl.default_framebuffer.viewport.size()

# how much to displace window text relative to the center of the
# app window
TEXT_DELTA_FROM_CENTER = 0.5

# the maximum number of chars displayable in the app window
# using the magnum text module.
MAX_DISPLAY_TEXT_CHARS = 256

# Load a TrueTypeFont plugin and open the font file
display_font = text.FontManager().load_and_instantiate("TrueTypeFont")
relative_path_to_font = "../data/fonts/ProggyClean.ttf"
display_font.open_file(
    os.path.join(os.path.dirname(__file__), relative_path_to_font),
    13,
)

# Glyphs we need to render everything
glyph_cache = text.GlyphCache(mn.Vector2i(256))
display_font.fill_glyph_cache(
    glyph_cache,
    string.ascii_lowercase
    + string.ascii_uppercase
    + string.digits
    + ":-_+,.! %µ",
)

# magnum text object that displays CPU/GPU usage data in the app window
window_text = text.Renderer2D(
    display_font,
    glyph_cache,
    DISPLAY_FONT_SIZE,
    text.Alignment.TOP_LEFT,
)
window_text.reserve(MAX_DISPLAY_TEXT_CHARS)

# text object transform in window space is Projection matrix times Translation Matrix
window_text_transform = mn.Matrix3.projection(
    mn.Vector2(viewport_size)
) @ mn.Matrix3.translation(
    mn.Vector2(
        viewport_size[0]
        * -TEXT_DELTA_FROM_CENTER,
        viewport_size[1]
        * TEXT_DELTA_FROM_CENTER,
    )
)
shader = shaders.VectorGL2D()

# make magnum text background transparent
mn.gl.Renderer.enable(mn.gl.Renderer.Feature.BLENDING)
mn.gl.Renderer.set_blend_function(
    mn.gl.Renderer.BlendFunction.ONE,
    mn.gl.Renderer.BlendFunction.ONE_MINUS_SOURCE_ALPHA,
)
mn.gl.Renderer.set_blend_equation(
    mn.gl.Renderer.BlendEquation.ADD, mn.gl.Renderer.BlendEquation.ADD
)

# draw text
shader.bind_vector_texture(glyph_cache.texture)
shader.transformation_projection_matrix = window_text_transform
shader.color = [1.0, 1.0, 1.0]
window_text.render(
    f"""
Hello World
    """
)
shader.draw(window_text.mesh)
