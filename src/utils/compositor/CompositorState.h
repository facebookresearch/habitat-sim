// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_COMPOSITOR_H_
#define ESP_COMPOSITOR_H_

#include <Corrade/Containers/Pointer.h>
#include <Corrade/PluginManager/Manager.h>
#include <Magnum/Math/Matrix4.h>
#include <Magnum/Trade/AbstractImporter.h>
#include <Magnum/Trade/AbstractImageConverter.h>
#include <Magnum/Trade/AbstractSceneConverter.h>
#include <Magnum/Trade/SceneData.h>

namespace esp {

// TODO make these builtin
constexpr Magnum::Trade::SceneField SceneFieldMeshViewIndexOffset = Magnum::Trade::sceneFieldCustom(0);
constexpr Magnum::Trade::SceneField SceneFieldMeshViewIndexCount = Magnum::Trade::sceneFieldCustom(1);
constexpr Magnum::Trade::SceneField SceneFieldMeshViewMaterial = Magnum::Trade::sceneFieldCustom(2);

struct CompositorState {
  explicit CompositorState(Corrade::Containers::StringView output);

  Corrade::PluginManager::Manager<Magnum::Trade::AbstractImporter> importerManager;
  Corrade::PluginManager::Manager<Magnum::Trade::AbstractImageConverter> imageConverterManager;
  Corrade::PluginManager::Manager<Magnum::Trade::AbstractSceneConverter> converterManager;

  Corrade::Containers::Pointer<Magnum::Trade::AbstractSceneConverter> converter;
};

/* Parent, present for all objects */
struct Parent {
  Magnum::UnsignedInt mapping;
  Magnum::Int parent;
};

/* Transformation, present only for nested objects */
struct Transformation {
  Magnum::UnsignedInt mapping;
  Magnum::Matrix4 transformation;
};

/* Mesh assignment, present for all objects except a "template root" that
    contains multiple meshes as children */
struct Mesh {
  Magnum::UnsignedInt mapping;
  Magnum::UnsignedInt mesh; /* always 0 in this case */
  Magnum::UnsignedInt meshIndexOffset;
  Magnum::UnsignedInt meshIndexCount;
  Magnum::Int meshMaterial;
};

struct CompositorSceneState {
  Magnum::Trade::SceneData finalizeScene() const;

  Corrade::Containers::Array<Parent> parents;
  Corrade::Containers::Array<Transformation> transformations;
  Corrade::Containers::Array<Mesh> meshes;
};

/* Meshes, textures & materials get collected, then joined / packed, then added
   to the converter. One CompositorDataState is one mesh/image/texture in the
   output. */
struct CompositorDataState {
  explicit CompositorDataState(const Magnum::Vector2i& textureAtlasSize);

  Magnum::Trade::MeshData finalizeMesh() const;

  Magnum::Trade::ImageData3D finalizeImage(Corrade::Containers::ArrayView<Magnum::Trade::MaterialData> inputMaterials) const;

  Magnum::Trade::TextureData finalizeTexture() const;

  Magnum::Vector2i textureAtlasSize;

  Corrade::Containers::Array<Magnum::Trade::MeshData> inputMeshes;
  /* There's one implicit 1x1 white image for textureless materials */
  // TODO what if nothing needs it?
  Corrade::Containers::Array<Magnum::Trade::ImageData2D> inputImages;

  /* As textures get packed into an atlas, the materials will need to be
     updated with final layer IDs and offsets. Store them temporarily in an
     array, using the imported image index and zero offset as placeholders.

     There's one implicit all-white material for materialless meshes. */
  // TODO what if everything has a material??
  Corrade::Containers::Array<Magnum::Trade::MaterialData> inputMaterials;

  /* Index offset for currently added mesh */
  Magnum::UnsignedInt indexOffset = 0;
};

}

#endif
