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
namespace gfx {

enum class BatchRendererFlag {
  NoTextures = 1 << 0
  // TODO memory-map
};
typedef Corrade::Containers::EnumSet<BatchRendererFlag> BatchRendererFlags;
CORRADE_ENUMSET_OPERATORS(BatchRendererFlags)

struct BatchRendererConfiguration {
  explicit BatchRendererConfiguration();
  ~BatchRendererConfiguration();

  BatchRendererConfiguration& setFlags(BatchRendererFlags flags);
  BatchRendererConfiguration& setTileSizeCount(
      const Magnum::Vector2i& tileSize,
      const Magnum::Vector2i& tileCount);

  struct State;
  Corrade::Containers::Pointer<State> state;
};

class BatchRenderer {
 public:
  explicit BatchRenderer(const BatchRendererConfiguration& configuration)
      : BatchRenderer{Magnum::NoCreate} {
    create(configuration);
  }

  ~BatchRenderer();

  Magnum::Vector2i tileSize() const;
  Magnum::Vector2i tileCount() const;
  /* Same as tileCount().product() */
  std::size_t sceneCount() const;

  // TODO take an importer instead? that way the consumer can configure it
  void addFile(Corrade::Containers::StringView filename);
  void addFile(Corrade::Containers::StringView filename,
               Corrade::Containers::StringView importerPlugin);

  // TODO "if there's a scene, name corresponds to a root bject name,
  //  otherwise it's the whole file"
  // TODO or use empty name for the whole scene? but which file??? or use a
  //  scene name for the whole scene??
  // TODO document this may not return consecutive IDs in case the added name
  //  is multiple meshes together
  std::size_t add(Magnum::UnsignedInt sceneId,
                  Corrade::Containers::StringView name);
  std::size_t add(Magnum::UnsignedInt sceneId,
                  Corrade::Containers::StringView name,
                  const Magnum::Matrix4& transformation);
  void clear(Magnum::UnsignedInt sceneId);

  Magnum::Matrix4& camera(Magnum::UnsignedInt sceneId);
  Corrade::Containers::StridedArrayView1D<Magnum::Matrix4> transformations(
      Magnum::UnsignedInt sceneId);

  void draw(Magnum::GL::AbstractFramebuffer& framebuffer);

 protected:
  /* used by BatchRendererStandalone */
  explicit BatchRenderer(Magnum::NoCreateT);
  void create(const BatchRendererConfiguration& configuration);
  void destroy();

 private:
  struct State;
  Corrade::Containers::Pointer<State> state_;
};

}  // namespace gfx
}  // namespace esp

#endif
