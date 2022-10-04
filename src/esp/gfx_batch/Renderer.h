// Copyright (c) Facebook, Inc. and its affiliates.
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

enum class RendererFlag {
  NoTextures = 1 << 0
  // TODO memory-map
};
typedef Corrade::Containers::EnumSet<RendererFlag> RendererFlags;
CORRADE_ENUMSET_OPERATORS(RendererFlags)

enum class RendererFileFlag {
  Whole = 1 << 0
};
typedef Corrade::Containers::EnumSet<RendererFileFlag> RendererFileFlags;
CORRADE_ENUMSET_OPERATORS(RendererFileFlags)

struct RendererConfiguration {
  explicit RendererConfiguration();
  ~RendererConfiguration();

  RendererConfiguration& setFlags(RendererFlags flags);
  RendererConfiguration& setTileSizeCount(const Magnum::Vector2i& tileSize,
                                          const Magnum::Vector2i& tileCount);

  struct State;
  Corrade::Containers::Pointer<State> state;
};

struct SceneStats;

class Renderer {
 public:
  explicit Renderer(const RendererConfiguration& configuration)
      : Renderer{Magnum::NoCreate} {
    create(configuration);
  }

  ~Renderer();

  Magnum::Vector2i tileSize() const;
  Magnum::Vector2i tileCount() const;
  /* Same as tileCount().product() */
  std::size_t sceneCount() const;

  void addFile(Corrade::Containers::StringView filename, RendererFileFlags flags = {});
  void addFile(Corrade::Containers::StringView filename, RendererFileFlags flags, Corrade::Containers::StringView name);
  // TODO take an importer instead? that way the consumer can configure it
  void addFile(Corrade::Containers::StringView filename,
               Corrade::Containers::StringView importerPlugin, RendererFileFlags flags = {});
  void addFile(Corrade::Containers::StringView filename,
               Corrade::Containers::StringView importerPlugin, RendererFileFlags flags, Corrade::Containers::StringView name);

  // TODO "if there's a scene, name corresponds to a root bject name,
  //  otherwise it's the whole file"
  // TODO or use empty name for the whole scene? but which file??? or use a
  //  scene name for the whole scene??
  // TODO document this may not return consecutive IDs in case the added name
  //  is multiple meshes together
  std::size_t addMeshHierarchy(Magnum::UnsignedInt sceneId,
                               Corrade::Containers::StringView name);
  std::size_t addMeshHierarchy(Magnum::UnsignedInt sceneId,
                               Corrade::Containers::StringView name,
                               const Magnum::Matrix4& transformation);
  void clear(Magnum::UnsignedInt sceneId);

  Magnum::Matrix4& camera(Magnum::UnsignedInt sceneId);
  Corrade::Containers::StridedArrayView1D<Magnum::Matrix4> transformations(
      Magnum::UnsignedInt sceneId);

  void draw(Magnum::GL::AbstractFramebuffer& framebuffer);

  /**
   * @brief Scene stats
   *
   * Mainly for testing and introspection purposes.
   */
  SceneStats sceneStats(Magnum::UnsignedInt sceneId) const;

 protected:
  /* used by RendererStandalone */
  explicit Renderer(Magnum::NoCreateT);
  void create(const RendererConfiguration& configuration);
  void destroy();

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
