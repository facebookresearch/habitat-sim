// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_BATCH_RENDERER_H_
#define ESP_GFX_BATCH_RENDERER_H_

#include <Corrade/Containers/Pointer.h>
#include <Magnum/GL/GL.h>
#include <Magnum/Magnum.h>
#include <cstddef>

namespace esp {
namespace gfx_batch {

/**
@brief Global batch renderer flag

@see @ref RendererFlags, @ref RendererConfiguration::setFlags()
*/
enum class RendererFlag {
  /**
   * Render without textures.
   *
   * Causes textures to not even get loaded, potentially saving significant
   * amount of memory. Only material and vertex colors are used for rendering.
   */
  NoTextures = 1 << 0

  // TODO memory-map
};

/**
@brief Global batch renderer flags

@see @ref RendererConfiguration::setFlags()
*/
typedef Corrade::Containers::EnumSet<RendererFlag> RendererFlags;

CORRADE_ENUMSET_OPERATORS(RendererFlags)

class Renderer;

/**
@brief Global renderer configuration

Passed to @ref Renderer::Renderer(const RendererConfiguration&) and
@ref RendererStandalone::RendererStandalone(const RendererConfiguration&, const RendererStandaloneConfiguration&).
@see @ref Renderer, @ref RendererStandaloneConfiguration
*/
struct RendererConfiguration {
  explicit RendererConfiguration();
  ~RendererConfiguration();

  /**
   * @brief Set renderer flags
   *
   * By default no flags are set.
   * @see @ref Renderer::flags()
   */
  RendererConfiguration& setFlags(RendererFlags flags);

  /**
   * @brief Set tile size and count
   *
   * By default there's a single @cpp {128, 128} @ce tile, corresponding to a
   * single rendered scene. Tiles are organized in the framebuffer in a grid
   * and @p tileCount implies an upper bound on number of scenes rendered. To
   * avoid hitting GPU limits with large tile counts and/or sizes, it's best to
   * aim for the total framebuffer size given by @cpp tileSize*tileCount @ce to
   * be a power-of-two square. On the other hand, having tiles organized in a
   * single column may make it easier for external libraries to consume the
   * data tile-by-tile, as there's no row stride to account for.
   *
   * Scenes that are empty are not rendered at all, so it's fine to have unused
   * tiles, they only occupy space in the output framebuffer. For example, if
   * you want 13 scenes, set @p tileCount to @cpp {4, 4} @ce and ignore the
   * last 3.
   * @see @ref Renderer::tileSize(), @ref Renderer::tileCount()
   */
  RendererConfiguration& setTileSizeCount(const Magnum::Vector2i& tileSize,
                                          const Magnum::Vector2i& tileCount);

  /**
   * @brief Set max light count per draw
   *
   * By default no lights are used, i.e. flat-shaded rendering. At the moment
   * there's no light culling in place, which means that light count per draw
   * is equivalent to the total light count used per scene.
   * @see @ref Renderer::maxLightCount()
   */
  RendererConfiguration& setMaxLightCount(Magnum::UnsignedInt count);

  /**
   * @brief Set ambient factor
   *
   * How much of the diffuse color / texture is used for ambient light in
   * Phong-shaded materials. Default value is @cpp 0.1f @ce, setting it to
   * @cpp 0.0f @ce will make the render completely black in areas without light
   * contribution. In comparison, flat-shaded materials (i.e., imported with
   * @ref Magnum::Trade::MaterialType::Flat) have the ambient factor always
   * @cpp 1.0 @ce and are unaffected by lights.
   *
   * This value can currently only set upfront due to how materials are
   * implemented internally.
   */
  RendererConfiguration& setAmbientFactor(Magnum::Float factor);

 private:
  friend Renderer;
  struct State;
  Corrade::Containers::Pointer<State> state;
};

/**
@brief Batch renderer file treatment flag

@see @ref RendererFileFlags, @ref Renderer::addFile()
*/
enum class RendererFileFlag {
  /**
   * Treat the file as a whole.
   *
   * By default a file is treated as a so-called *composite* --- a collection
   * of *node hierarchy templates* --- which you then add with
   * @ref Renderer::addNodeHierarchy() using a name corresponding to one of the
   * root nodes. With this flag set, the whole file is treated as a single
   * hierarchy instead, with the @p filename (or @p name, if present) argument
   * of @ref Renderer::addFile() used as a name.
   *
   * @m_class{m-note m-warning}
   *
   * @par
   *    This flag is required to be set when importing scene-less files such as
   *    STL or PLY.
   */
  Whole = 1 << 0,

  /**
   * Generate a mipmap out of single-level uncompressed textures.
   *
   * By default, no renderer-side processing of imported textures is done ---
   * if the image contains multiple levels, they're imported as well, otherwise
   * just a single level is uploaded, regardless of whether the texture has a
   * filtering across mip levels set up.
   *
   * With this option enabled, if the input image is uncompressed and there's
   * just a single level, the texture is created with a full mip pyramid and
   * the lower levels get generated on the fly, leading to a smoother look with
   * large textures. On-the-fly mip level generation is impossible for
   * compressed formats, there this option is ignored.
   */
  GenerateMipmap = 1 << 1
};

/**
@brief Batch renderer file treatment flags

@see @ref Renderer::addFile()
*/
typedef Corrade::Containers::EnumSet<RendererFileFlag> RendererFileFlags;

CORRADE_ENUMSET_OPERATORS(RendererFileFlags)

/**
@brief Renderer light type

@see @ref Renderer::addLight()
*/
enum class RendererLightType {
  Directional = 0, /**< Directional */
  Point = 1        /**< Point */
};

struct SceneStats;

// Even though Clang Format is told to skip formatting comments containing
//  @section or @ref which can't be wrapped to prevent Doxygen bugs, it still
//  wants to do that here. So I'm disabling it for this doc block altogether.
// clang-format off
/**
@brief Batch renderer

A renderer optimized for drawing multiple scenes at once, applying multi-draw
optimizations where possible to reduce draw overhead. While all scenes are
rendered at once, each scene contains an independent set of rendered objects.

This class expects an active @ref Magnum::GL::Context and renders into a
provided framebuffer, making it suitable for use in GUI applications. For
standalone operation use the @ref RendererStandalone subclass instead, which
manages the OpenGL context and a framebuffer on its own. Apart from the context
and framebuffer, the high-level usage is same for both.

@section gfx_batch-Renderer-usage Usage

The renderer gets constructed using a @ref RendererConfiguration with desired
tile size and count set via @ref RendererConfiguration::setTileSizeCount() as
described in its documentation. Each tile corresponds to one rendered scene,
organized in a grid, all scenes have the same rendered size.

First, files with meshes, materials and textures that are meant to be rendered
from should get added via @ref addFile(). The file itself isn't directly
rendered, instead it's treated as a *composite file* with named root scene
nodes being *node hierarchy templates* to be selectively added to particular
scenes. Apart from picking particular root nodes, it's also possible to treat
the whole file as a single hierarchy using @ref RendererFileFlag::Whole. See
the flag documentation for more information.

@m_class{m-note m-warning}

@par
  All needed files should be added upfront, before populating the scenes or
  drawing anything. This is currently not enforced in any way, but is subject
  to change.

Then, particular scenes are populated using @ref addNodeHierarchy(),
referencing the node hierarchy templates via their names. The function takes an
initial transformation and returns an ID of the node added into the scene. The
ID can then be used to subsequently update its transformation via
@ref transformations(), for example in respose to a physics simulation or an
animation. Each scene also has an associated camera and its combined projection
and transformation matrix can be updated using @ref updateCamera().

Finally, the @ref draw() function renders the grid of scenes into a provided
framebuffer.

@section gfx_batch-Renderer-workflow Inner workflow of the batch renderer

The goal of the renderer is to do as much as possible using the least amount of
draw calls. For that, it relies heavily on multi-draw and texture array support
implemented in @ref shaders-usage-multidraw "Magnum shaders".

- At the beginning, all data from files added by @ref addFile() get uploaded to
  the GPU. The data consists of meshes, texture image levels and packed
  material uniforms. Such data are uploaded only once and treated as immutable
  for the rest of the renderer lifetime.
- On each @ref addNodeHierarchy() call, a list of mesh views each together with
  material association, texture layer and transform and node assignment is
  added to a *draw list*. The draw list is
  @ref gfx_batch-Renderer-workflow-draw-list "further detailed below".
- For each @ref draw() and each non-empty scene, the following is done:
  - The transformation passed to @ref updateCamera() is uploaded to a uniform buffer.
  - The renderer calculates hierarchical transformations for all nodes based on
    the matrices supplied via @ref transformations(). Each item in the draw
    list is then assigned a corresponding calculated absolute transformation,
    and the list of transformations corresponding to all draws is uploaded to a
    uniform buffer.
  - Then the draw list is processed, resulting in one or more multi-draw calls
    @ref gfx_batch-Renderer-workflow-draw-list "as described below".

@subsection gfx_batch-Renderer-workflow-draw-list Draw list and multi-draw call submission

In the ideal (and often impossible) case, there would be just a single file
added with @ref addFile(), containing exactly one mesh and exactly one texture
array, with scene nodes referring to sub-views of them --- i.e., mesh index
ranges, texture layer indices and texture transformation matrices. Then, no
matter how many times @ref addNodeHierarchy() gets called, the whole draw list
populated by it can be drawn with a single multi-draw call.

@m_class{m-note m-info}

@par
  When @ref sceneStats() would be retrieved for such a case,
  @ref SceneStats::nodeCount and @ref SceneStats::drawCount would be both high
  numbers, while @ref SceneStats::drawBatchCount would be exactly @cpp 1 @ce.

In practice however, the files may contain meshes with different vertex layouts
(such as some having vertex colors and some not), textures of different formats
and sizes, and different materials requiring different shaders (such as
vertex-colored, alpha-masked etc.). Draw lists populated with
@ref addNodeHierarchy() are then partitioned into *draw batches*, where a
particular draw batch contains all draws with a particular shader, from a
particular mesh and with a particular texture. Each draw batch then corresponds
to a single multi-draw call issued on the GPU.

@subsection gfx_batch-Renderer-workflow-tradeoffs Performance tradeoffs

The optimization goal here is to pick a balance between minimizing driver
overhead from submitting many draw calls and paying extra cost for shader
features, texture fetches and vertex attributes present also for draws that
don't need them.

@m_class{m-note m-info}

@par
  In other words, trying to maximize the ratio of @ref SceneStats::drawCount
  and @ref SceneStats::drawBatchCount for as long as CPU usage gets lower but
  GPU usage doesn't get higher.

For example, if only some meshes use vertex colors, it's possible to add white
vertex colors to the remaining meshes, unifying their layout and making it
possible to draw them together in a single draw call. The cost of extra vertex
processing bandwidth would probably be rather minimal. On the other hand, if
some materials need alpha masking and some not, attempting to draw everything
with alpha masking enabled may have a much larger cost than the additional draw
call saved by this change.

Here it's important to also take into account differences on the web --- there
the driver overhead is significantly higher and it might be beneficial to put
extra effort into reducing draw batch count even though it may result in
slightly worse performance natively.

@section gfx_batch-Renderer-files Creating batch-optimized files

While @ref addFile() can consume any count of any regular scene/model files
@ref file-formats "supported by Magnum", in order to properly take advantage of
all performance features it's needed to combine the files into batch-optimized
* *composite files* containing mesh views and texture arrays.

Batch-optimized files can be stored in any file format that has sufficient
flexibility for @ref Magnum::Trade::MeshData layouts, is capable of storing 2D
array textures and supports storing custom @ref Magnum::Trade::SceneData
fields. At the moment, a file format supporting all this is glTF, with files
produced using the @relativeref{Magnum::Trade,GltfSceneConverter} plugin.

@m_class{m-note m-info}

@par
  For an overview of how a fully manual process of creating a batch-optimized
  glTF file looks like, have a look at the `generateTestData()` function in the
  [GfxBatchRendererTest.cpp](https://github.com/facebookresearch/habitat-sim/blob/main/src/tests/GfxBatchRendererTest.cpp)
  file. Practical cases of dataset conversion would be more focused on reusing
  and combining data imported from other files instead of creating them from
  scratch, but the overall process is similar.

@subsection gfx_batch-Renderer-files-meshes Meshes and mesh views

* *Composite meshes*, which concatenate several meshes of the same layout
together, can be added with the general
@ref Magnum::Trade::AbstractSceneConverter::add(const MeshData&, Containers::StringView)
API and referenced from scene nodes via @ref Magnum::Trade::SceneField::Mesh.
* *Mesh views*, referencing sub-ranges of the composite mesh, aren't a builtin
@ref Magnum::Trade::SceneData feature at the moment however, and thus have to
be added as @ref Trade-SceneData-populating-custom "custom scene fields" of the
following names:

- `meshViewIndexOffset` of type @ref Magnum::Trade::SceneFieldType::UnsignedInt,
  containing offset of the mesh view in the index buffer *in bytes*,
- `meshViewIndexCount` of type @ref Magnum::Trade::SceneFieldType::UnsignedInt,
  containing index count, and
- `meshViewMaterial` of type @ref Magnum::Trade::SceneFieldType::Int containing
  the corresponding material ID. The builtin
  @ref Magnum::Trade::SceneField::MeshMaterial is not used, because glTF bakes
  the material reference into the mesh itself, which means referencing the same
  mesh multiple times with different materials would cause it to be
  unnecessarily duplicated in the glTF.

The custom field IDs can be arbitrary, what's important is that they are
associated with corresponding names using
@ref Magnum::Trade::AbstractSceneConverter::setSceneFieldName(). Furthermore,
to prevent the file from being opened by unsuspecting glTF viewers, a private
`MAGNUMX_mesh_views` extension should be added as both
@cb{.ini} extensionUsed @ce and @cb{.ini} extensionRequired @ce in the
@ref Trade-GltfSceneConverter-configuration "plugin configuration". Inspecting
the resulting file with @ref magnum-sceneconverter "magnum-sceneconverter" may
look similarly to this:

@m_class{m-console-figure}

@parblock

@code{.sh}
magnum-sceneconverter --info-scenes -i ignoreRequiredExtensions batch.gltf
@endcode

<b><!-- don't remove, Doxygen workaround --></b>

@m_class{m-nopad}

@include gfx-batch-info-scenes.ansi

@endparblock

<b><!-- don't remove, Doxygen workaround --></b>

@m_class{m-note m-info}

@par
  This implementation of mesh views is a temporary solution. Eventually mesh
  views will become a builtin Magnum feature, being implicitly encoded in glTF
  accessor properties in a way that's compatible with 3rd party glTF tools as
  well.

<b><!-- don't remove, Doxygen workaround --></b>

@m_class{m-note m-success}

@par
  Meshes can be concatenated together for example using
  @ref Magnum::MeshTools::concatenate().

@subsection gfx_batch-Renderer-files-textures Texture arrays

For 2D texture arrays the
[KHR_texture_ktx](https://github.com/KhronosGroup/glTF/pull/1964) extension is
used. Because it's not approved by Khronos yet, it needs the
@cb{.ini} experimentalKhrTextureKtx @ce
@ref Trade-GltfSceneConverter-configuration "configuration option"
enabled in the converter plugin. Materials should reference layers of it via
@ref Magnum::Trade::MaterialAttribute::BaseColorTextureLayer and related
attributes, together with supplying
@relativeref{Magnum::Trade::MaterialAttribute,BaseColorTextureMatrix}
describing a sub-image of a particular layer. Inspecting the resulting file
with @ref magnum-sceneconverter "magnum-sceneconverter" may look similarly to
this:

@m_class{m-console-figure}

@parblock

@m_class{m-console-wrap}

@code{.sh}
magnum-sceneconverter --info-textures --info-images --info-materials \
  -i experimentalKhrTextureKtx,ignoreRequiredExtensions batch.gltf
@endcode

<b></b>

@m_class{m-nopad}

@include gfx-batch-info-textures.ansi

@endparblock

@m_class{m-note m-success}

@par
  2D textures with power-of-two sizes can be atlased into a texture array for
  example using @ref Magnum::TextureTools::atlasArrayPowerOfTwo(). If a certain
  texture needs @ref Magnum::SamplerWrapping::Repeat or
  @relativeref{Magnum::SamplerWrapping,MirroredRepeat}, it has to occupy a
  whole layer, or be a separate texture altogether.

*/
// clang-format on
class Renderer {
 public:
  /**
   * @brief Constructor
   * @param configuration   Renderer configuration
   *
   * Expects an active @ref Magnum::GL::Context to be present.
   */
  explicit Renderer(const RendererConfiguration& configuration)
      : Renderer{Magnum::NoCreate} {
    create(configuration);
  }

  virtual ~Renderer();

  /**
   * @brief Global renderer flags
   *
   * By default, no flags are set.
   * @see @ref RendererConfiguration::setFlags()
   */
  RendererFlags flags() const;

  /**
   * @brief Tile size
   *
   * The default tile size is @cpp {128, 128} @ce.
   * @see @ref RendererConfiguration::setTileSizeCount()
   */
  Magnum::Vector2i tileSize() const;

  /**
   * @brief Tile count
   *
   * By default there's a single tile.
   * @see @ref RendererConfiguration::setTileSizeCount()
   */
  Magnum::Vector2i tileCount() const;

  /**
   * @brief Scene count
   *
   * Same as the @ref Magnum::Math::Vector::product() "product()" of
   * @ref tileCount(). Empty scenes are not rendered, they only occupy space in
   * the output framebuffer.
   */
  std::size_t sceneCount() const;

  /**
   * @brief Max light count
   *
   * By default there's zero lights, i.e. flat-shaded rendering.
   * @see @ref RendererConfiguration::setMaxLightCount()
   */
  Magnum::UnsignedInt maxLightCount() const;

#ifdef DOXYGEN_GENERATING_OUTPUT
  /**
   * @brief Add a file
   * @param filename        File to add
   * @param flags           File flags
   * @param name            Hierarchy name. Ignored if
   *    @ref RendererFileFlag::Whole isn't set.
   *
   * By default a file is treated as a so-called *composite* --- a collection
   * of *node hierarchy templates*, which you then add with
   * @ref Renderer::addNodeHierarchy() using a name corresponding to one of the
   * root nodes. This means all root nodes have to be named and have their
   * names unique.
   *
   * If @ref RendererFileFlag::Whole is set, the whole file is treated as a
   * single mesh hierarchy template instead, named @p name, or if @p name is
   * empty with the @p filename used as a name.
   *
   * Returns @cpp true @ce on success. Prints a message to @ref Magnum::Error
   * and returns @cpp false @ce if the file can't be imported or the names are
   * conflicting.
   */
  // TODO the name could be used as a prefix for hierarchy template names to
  //  make it possible to add several files of the same name
  bool addFile(Corrade::Containers::StringView filename,
               RendererFileFlags flags = {},
               Corrade::Containers::StringView name = {});
#else
  /* To avoid having to include StringView in the header */
  bool addFile(Corrade::Containers::StringView filename,
               RendererFileFlags flags,
               Corrade::Containers::StringView name);
  bool addFile(Corrade::Containers::StringView filename,
               RendererFileFlags flags = {});
#endif

#ifdef DOXYGEN_GENERATING_OUTPUT
  /**
   * @brief Add a file using a custom importer plugin
   * @param filename        File to add
   * @param importerPlugin  Importer plugin name or path
   * @param flags           File flags
   * @param name            Hierarchy name. Ignored if
   *    @ref RendererFileFlag::Whole isn't set.
   *
   * See @ref addFile(Corrade::Containers::StringView, RendererFileFlags, Corrade::Containers::StringView)
   * for more information.
   */
  // TODO take an importer instead? that way the consumer can configure it,
  //  retrieve its own data from it...
  bool addFile(Corrade::Containers::StringView filename,
               Corrade::Containers::StringView importerPlugin,
               RendererFileFlags flags = {},
               Corrade::Containers::StringView name = {});
#else
  /* To avoid having to include StringView in the header */
  bool addFile(Corrade::Containers::StringView filename,
               Corrade::Containers::StringView importerPlugin,
               RendererFileFlags flags,
               Corrade::Containers::StringView name);
  bool addFile(Corrade::Containers::StringView filename,
               Corrade::Containers::StringView importerPlugin,
               RendererFileFlags flags = {});
#endif

  /**
   * @brief If given mesh hierarchy exists
   *
   * Returns @cpp true @ce if @p name is a mesh hierarchy name added by any
   * previous @ref addFile() call, @cpp false @ce otherwise.
   */
  bool hasNodeHierarchy(Corrade::Containers::StringView name) const;

#ifdef DOXYGEN_GENERATING_OUTPUT
  /**
   * @brief Add a mesh hierarchy
   * @param sceneId         Scene ID, expected to be less than
   *    @ref sceneCount()
   * @param name            *Node hierarchy template* name, added with
   *    @ref addFile() earlier
   * @param bakeTransformation Transformation to bake into the hierarchy
   * @return ID of the newly added node
   *
   * The returned ID can be subsequently used to update transformations via
   * @ref transformations(). The returned IDs are *not* contiguous, the gaps
   * correspond to number of child nodes in the hierarchy.
   *
   * The @p bakeTransformation is baked into the added hierarchy, i.e.
   * @ref transformations() at the returned ID is kept as an identity transform
   * and writing to it will not overwrite the baked transformation. This
   * parameter is useful for correcting orientation/scale of the imported mesh.
   * @see @ref hasNodeHierarchy(), @ref clear()
   */
  std::size_t addNodeHierarchy(Magnum::UnsignedInt sceneId,
                               Corrade::Containers::StringView name,
                               const Magnum::Matrix4& bakeTransformation = {});
#else
  /* To avoid having to include Matrix4 in the header */
  std::size_t addNodeHierarchy(Magnum::UnsignedInt sceneId,
                               Corrade::Containers::StringView name,
                               const Magnum::Matrix4& bakeTransformation);
  std::size_t addNodeHierarchy(Magnum::UnsignedInt sceneId,
                               Corrade::Containers::StringView name);
#endif

  std::size_t addEmptyNode(Magnum::UnsignedInt sceneId);

  /**
   * @brief Add a light
   * @param sceneId         Scene ID, expected to be less than
   *    @ref sceneCount()
   * @param nodeId          Node ID to inherit transformation from, returned
   *    from @ref addNodeHierarchy() or @ref addEmptyNode() earlier
   * @param type            Light type. Can't be changed after adding the
   *    light.
   *
   * Expects that @ref maxLightCount() isn't @cpp 0 @ce. If @p type is
   * @ref RendererLightType::Directional, a negative
   * @ref Magnum::Matrix4::backwards() is taken from @p nodeId transformation
   * as the direction, if @p type is @ref RendererLightType::Point,
   * @ref Magnum::Matrix4::translation() is taken from @p nodeId transformation
   * as the position. The ID returned from this function can be subsequently
   * used to update light properties via @ref lightColors() and
   * @ref lightRanges().
   * @see @ref clearLights(), @ref clear()
   */
  std::size_t addLight(Magnum::UnsignedInt sceneId,
                       std::size_t nodeId,
                       RendererLightType type);

  /**
   * @brief Clear a scene
   * @param sceneId   Scene ID, expected to be less than @ref sceneCount()
   *
   * Clears everything added by @ref addNodeHierarchy(),
   * @ref addEmptyNode() and @ref addLight().
   * @see @ref clearLights()
   */
  void clear(Magnum::UnsignedInt sceneId);

  /**
   * @brief Clear lights in a scene
   * @param sceneId   Scene ID, expected to be less than @ref sceneCount()
   *
   * Clears everything added by @ref addLight().
   * @see @ref clear()
   */
  void clearLights(Magnum::UnsignedInt sceneId);

  /**
   * @brief Get the combined projection and view matrices of a camera
   * (read-only)
   * @param sceneId Scene ID, expected to be less than @ref sceneCount()
   */
  Magnum::Matrix4 camera(Magnum::UnsignedInt sceneId) const;

  /**
   * @brief Get the depth unprojection parameters of a camera (read-only)
   * @param sceneId Scene ID, expected to be less than @ref sceneCount()
   */
  Magnum::Vector2 cameraDepthUnprojection(Magnum::UnsignedInt sceneId) const;

  /**
   * @brief Set the camera projection and view matrices
   * @param sceneId     Scene ID, expected to be less than @ref sceneCount()
   * @param view        View matrix of the camera (inverse transform)
   * @param projection  Projection matrix of the camera
   *
   * Also computes the camera unprojection.
   * Modifications to the transformation are taken into account in the next
   * @ref draw().
   */
  void updateCamera(Magnum::UnsignedInt sceneId,
                    const Magnum::Matrix4& projection,
                    const Magnum::Matrix4& view);

  /**
   * @brief Transformations of all nodes in the scene
   * @param sceneId   Scene ID, expected to be less than @ref sceneCount()
   *
   * Returns a view on all node transformations in given scene. Desired usage
   * is to update only transformations at indices returned by
   * @ref addNodeHierarchy() and @ref addEmptyNode(), updating transformations
   * at other indices is possible but could have unintended consequences.
   * Modifications to the transformations are taken into account in the next
   * @ref draw(). By default, transformations of root nodes (those which IDs
   * were returned from @ref addNodeHierarchy() or @ref addEmptyNode()) are
   * identities, transformations of other nodes are unspecified.
   */
  Corrade::Containers::StridedArrayView1D<Magnum::Matrix4> transformations(
      Magnum::UnsignedInt sceneId);

  /**
   * @brief Colors of all lights in the scene
   * @param sceneId   Scene ID, expected to be less than @ref sceneCount()
   *
   * Returns a view on colors of all lights in given scene. Desired usage is to
   * update only colors at indices returned by @ref addLight(), updating colors
   * at other indices is possible but could have unintended consequences.
   * Modifications to the lights are taken into account in the next
   * @ref draw(). By default, colors of root lights (those which IDs were
   * returned from @ref addLight()) are @cpp 0xffffff_rgbf @ce, colors of other
   * lights are unspecified.
   */
  Corrade::Containers::StridedArrayView1D<Magnum::Color3> lightColors(
      Magnum::UnsignedInt sceneId);

  /**
   * @brief Ranges of all lights in the scene
   * @param sceneId   Scene ID, expected to be less than @ref sceneCount()
   *
   * Returns a view on ranges of all lights in given scene. Desired usage is to
   * update only ranges at indices returned by @ref addLight(), updating ranges
   * at other indices is possible but could have unintended consequences.
   * Modifications to the lights are taken into account in the next
   * @ref draw(). The value range is ignored for
   * @ref RendererLightType::Directional. By default, ranges of root lights
   * (those which IDs were returned from @ref addLight()) are
   * @ref Magnum::Constants::inf(), ranges of other lights are unspecified.
   */
  Corrade::Containers::StridedArrayView1D<Magnum::Float> lightRanges(
      Magnum::UnsignedInt sceneId);

  /**
   * @brief Draw all scenes into provided framebuffer
   *
   * The @p framebuffer is expected to have a size at least as larger as the
   * product of @ref tileSize() and @ref tileCount().
   */
  void draw(Magnum::GL::AbstractFramebuffer& framebuffer);

  /**
   * @brief Scene stats
   *
   * Mainly for testing and introspection purposes. The returned info is
   * up-to-date only if @ref draw() has been called before.
   */
  SceneStats sceneStats(Magnum::UnsignedInt sceneId) const;

#ifndef DOXYGEN_GENERATING_OUTPUT
 protected:
  /* used by RendererStandalone */
  explicit Renderer(Magnum::NoCreateT);
  void create(const RendererConfiguration& configuration);
  void destroy();
#endif

 private:
  struct State;
  Corrade::Containers::Pointer<State> state_;
};

/**
@brief Statistics for a single renderer scene

Returned by @ref Renderer::sceneStats().
*/
struct SceneStats {
  /**
   * @brief Count of transformable nodes
   *
   * Same as size of the array returned by @ref Renderer::transformations().
   */
  std::size_t nodeCount;

  /**
   * @brief Count of draws across all draw batches
   *
   * Never larger than @ref nodeCount. Usually much smaller, as certain nodes
   * are only manipulators grouping a set of other actually renderable nodes.
   */
  std::size_t drawCount;

  /**
   * @brief Count of draw batches
   *
   * Ideal case is just one, but if there's more files or more textures, then
   * each such combination needs a dedicated draw batch. Never larger than
   * @ref drawCount.
   */
  std::size_t drawBatchCount;
};

}  // namespace gfx_batch
}  // namespace esp

#endif
