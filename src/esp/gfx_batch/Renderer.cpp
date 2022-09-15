// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "Renderer.h"

#include <Corrade/Containers/Array.h>
#include <Corrade/Containers/GrowableArray.h>
#include <Corrade/Containers/Optional.h>
#include <Corrade/Containers/Pair.h>
#include <Corrade/Containers/StringStl.h>
#include <Corrade/Containers/StringStlHash.h>
#include <Corrade/PluginManager/Manager.h>
#include <Corrade/PluginManager/PluginMetadata.h>
#include <Corrade/Utility/Algorithms.h>
#include <Corrade/Utility/ConfigurationGroup.h>
#include <Corrade/Utility/MurmurHash2.h>
#include <Corrade/Utility/Path.h>
#include <Magnum/GL/AbstractFramebuffer.h>
#include <Magnum/GL/Buffer.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/GL/Renderer.h>
#include <Magnum/GL/TextureArray.h>
#include <Magnum/GL/TextureFormat.h>
#include <Magnum/ImageView.h>
#include <Magnum/Math/PackingBatch.h>
#include <Magnum/MeshTools/Compile.h>
#include <Magnum/Shaders/Generic.h>
#include <Magnum/Shaders/Phong.h>
#include <Magnum/Shaders/PhongGL.h>
#include <Magnum/Trade/AbstractImporter.h>
#include <Magnum/Trade/FlatMaterialData.h>
#include <Magnum/Trade/ImageData.h>
#include <Magnum/Trade/MeshData.h>
#include <Magnum/Trade/SceneData.h>
#include <Magnum/Trade/TextureData.h>
#include <unordered_map>

namespace Cr = Corrade;
namespace Mn = Magnum;

namespace esp {
namespace gfx_batch {

// clang-tidy you're NOT HELPING
using namespace Cr::Containers::Literals;  // NOLINT

struct RendererConfiguration::State {
  RendererFlags flags;
  Mn::Vector2i tileSize{128, 128};
  Mn::Vector2i tileCount{16, 12};
};

RendererConfiguration::RendererConfiguration() : state{Cr::InPlaceInit} {}
RendererConfiguration::~RendererConfiguration() = default;

RendererConfiguration& RendererConfiguration::setFlags(RendererFlags flags) {
  state->flags = flags;
  return *this;
}

RendererConfiguration& RendererConfiguration::setTileSizeCount(
    const Mn::Vector2i& tileSize,
    const Mn::Vector2i& tileCount) {
  state->tileSize = tileSize;
  state->tileCount = tileCount;
  return *this;
}

namespace {

struct MeshView {
  // TODO also mesh ID, once we have multiple meshes
  Mn::UnsignedInt indexOffsetInBytes;
  Mn::UnsignedInt indexCount;
  Mn::Int materialId; /* is never -1 tho */
  // TODO also parent, when we are able to fetch the whole hierarchy for a
  //  particular root object name
  Mn::Matrix4 transformation;
};

struct DrawCommand {
#ifndef CORRADE_TARGET_32BIT
  Mn::UnsignedLong
#else
  Mn::UnsignedInt
#endif
      indexOffsetInBytes;
  Mn::UnsignedInt indexCount;
};

struct Scene {
  /* Appended to with add() */
  Cr::Containers::Array<Mn::Int> parents; /* parents[i] < i, always */
  Cr::Containers::Array<Mn::Matrix4> transformations;
  // TODO have this temporary and just once for all scenes, doesn't need to
  //  be stored
  Cr::Containers::Array<Mn::Shaders::TransformationUniform3D>
      absoluteTransformations;
  Cr::Containers::Array<Mn::Shaders::PhongDrawUniform> draws;
  Cr::Containers::Array<Mn::Shaders::TextureTransformationUniform>
      textureTransformations;
  // TODO make the layout match GL to avoid a copy in draw()
  Cr::Containers::Array<DrawCommand> drawCommands;

  /* Updated every frame */
  Mn::GL::Buffer transformationUniform;
  Mn::GL::Buffer drawUniform;
  Mn::GL::Buffer textureTransformationUniform;
};

struct TextureTransformation {
  Mn::UnsignedInt layer;
  Mn::Matrix3 transformation;
};

/* NVidia requires uniform buffer bindings to have an INSANE 256-byte
   alignment, so we give in and pad our stuff */
struct ProjectionPadded : Mn::Shaders::ProjectionUniform3D {
  char padding[256 - sizeof(Mn::Shaders::ProjectionUniform3D)];
};

}  // namespace

struct Renderer::State {
  RendererFlags flags;
  Mn::Vector2i tileSize, tileCount;
  Mn::Shaders::PhongGL shader{Mn::NoCreate};

  /* Filled upon addFile() */
  Mn::GL::Texture2DArray texture{Mn::NoCreate};
  Mn::GL::Mesh mesh{Mn::NoCreate};
  Mn::GL::Buffer materialUniform;
  /* Contains texture transform and layer for each material. Used by add() to
     populate the draw list. */
  Cr::Containers::Array<TextureTransformation> textureTransformations;

  /* Pairs of mesh views (index byte offset and count), material IDs and
     initial transformations for draws. Used by add() to populate the draw
     list. */
  Cr::Containers::Array<MeshView> meshViews;
  /* Range of mesh views and materials corresponding to a particular name */
  std::unordered_map<Cr::Containers::String,
                     Cr::Containers::Pair<Mn::UnsignedInt, Mn::UnsignedInt>>
      meshViewRangeForName;

  /* Updated from camera() */
  Cr::Containers::Array<ProjectionPadded> projections;
  /* Updated every frame */
  Mn::GL::Buffer projectionUniform;

  Cr::Containers::Array<Scene> scenes;
};

Renderer::Renderer(Mn::NoCreateT) {}

void Renderer::create(const RendererConfiguration& configurationWrapper) {
  const RendererConfiguration::State& configuration =
      *configurationWrapper.state;

  CORRADE_INTERNAL_ASSERT(!state_);
  state_.emplace();
  state_->flags = configuration.flags;
  state_->tileSize = configuration.tileSize;
  state_->tileCount = configuration.tileCount;
  const std::size_t sceneCount = configuration.tileCount.product();
  state_->projections = Cr::Containers::Array<ProjectionPadded>{sceneCount};
  state_->scenes = Cr::Containers::Array<Scene>{sceneCount};
  /* Have one extra transformation slot in each scene for easier transform
     calculation in draw() */
  for (Scene& scene : state_->scenes)
    arrayAppend(scene.absoluteTransformations, Cr::InPlaceInit);

  // TODO move this outside
  Mn::GL::Renderer::enable(Mn::GL::Renderer::Feature::FaceCulling);
  Mn::GL::Renderer::enable(Mn::GL::Renderer::Feature::DepthTest);
}

void Renderer::destroy() {
  state_ = {};
}

Renderer::~Renderer() = default;

Mn::Vector2i Renderer::tileCount() const {
  return state_->tileCount;
}

Mn::Vector2i Renderer::tileSize() const {
  return state_->tileSize;
}

std::size_t Renderer::sceneCount() const {
  return state_->scenes.size();
}

void Renderer::addFile(const Cr::Containers::StringView filename) {
  return addFile(filename, "AnySceneImporter");
}

void Renderer::addFile(const Cr::Containers::StringView filename,
                       const Cr::Containers::StringView importerPlugin) {
  CORRADE_ASSERT(!state_->texture.id(),
                 "Renderer::addFile(): sorry, only one file is supported "
                 "at the moment", );

  Cr::PluginManager::Manager<Mn::Trade::AbstractImporter> manager;
  Cr::Containers::Pointer<Mn::Trade::AbstractImporter> importer =
      manager.loadAndInstantiate(importerPlugin);
  CORRADE_INTERNAL_ASSERT(importer);

  /* Set up options for glTF import. We can also import any other files (such
     as serialized magnum blobs or BPS files), assume these don't need any
     custom setup. */
  if (importerPlugin.contains("GltfImporter") ||
      (importerPlugin.contains("AnySceneImporter") &&
       (filename.contains(".gltf") || filename.contains(".glb")))) {
    importer->configuration().setValue("ignoreRequiredExtensions", true);
    importer->configuration().setValue("experimentalKhrTextureKtx", true);

    /* Desired imported types for custom glTF scene fields. If the group
       doesn't exist (which is the case for AnySceneImporter), add it first */
    Cr::Utility::ConfigurationGroup* types =
        importer->configuration().group("customSceneFieldTypes");
    if (!types)
      types = importer->configuration().addGroup("customSceneFieldTypes");
    types->addValue("meshViewIndexOffset", "UnsignedInt");
    types->addValue("meshViewIndexCount", "UnsignedInt");
    types->addValue("meshViewMaterial", "Int");
  }

  /* Basis options. Don't want to bother with all platform variations right
     now, so it's always ASTC, sorry. */
  if (Cr::PluginManager::PluginMetadata* const metadata =
          manager.metadata("BasisImporter")) {
    metadata->configuration().setValue("format", "Astc4x4RGBA");
  }

  // TODO memory-map self-contained files (have a config option? do implicitly
  //  for glb, bps and ply?)
  CORRADE_INTERNAL_ASSERT_OUTPUT(importer->openFile(filename));

  /* One texture for the whole scene */
  if (!(state_->flags & RendererFlag::NoTextures)) {
    CORRADE_ASSERT(importer->textureCount() == 1,
                   "Renderer::addFile(): expected a file with exactly one "
                   "texture, got"
                       << importer->textureCount(), );
    const Cr::Containers::Optional<Mn::Trade::TextureData> texture =
        importer->texture(0);
    CORRADE_INTERNAL_ASSERT(
        texture && texture->type() == Mn::Trade::TextureType::Texture2DArray);

    const Mn::UnsignedInt levelCount =
        importer->image3DLevelCount(texture->image());
    Cr::Containers::Optional<Mn::Trade::ImageData3D> image =
        importer->image3D(texture->image());
    CORRADE_INTERNAL_ASSERT(image);
    state_->texture = Mn::GL::Texture2DArray{};
    state_->texture
        .setMinificationFilter(texture->minificationFilter(),
                               texture->mipmapFilter())
        .setMagnificationFilter(texture->magnificationFilter())
        .setWrapping(texture->wrapping().xy());
    if (image->isCompressed()) {
      state_->texture
          .setStorage(levelCount,
                      Mn::GL::textureFormat(image->compressedFormat()),
                      image->size())
          .setCompressedSubImage(0, {}, *image);
      for (Mn::UnsignedInt level = 1; level != levelCount; ++level) {
        Cr::Containers::Optional<Mn::Trade::ImageData3D> levelImage =
            importer->image3D(texture->image(), level);
        CORRADE_INTERNAL_ASSERT(levelImage && levelImage->isCompressed() &&
                                levelImage->compressedFormat() ==
                                    image->compressedFormat());
        state_->texture.setCompressedSubImage(level, {}, *levelImage);
      }
    } else {
      state_->texture
          .setStorage(levelCount, Mn::GL::textureFormat(image->format()),
                      image->size())
          .setSubImage(0, {}, *image);
    }
  }

  /* One mesh for the whole scene */
  CORRADE_ASSERT(
      importer->meshCount() == 1,
      "Renderer::addFile(): expected a file with exactly one mesh, got"
          << importer->meshCount(), );
  state_->mesh = Mn::MeshTools::compile(
      *CORRADE_INTERNAL_ASSERT_EXPRESSION(importer->mesh(0)));

  /* Immutable material data. Save texture transformations and layers to a
     temporary array to apply them to draws instead */
  state_->textureTransformations = Cr::Containers::Array<TextureTransformation>{
      Cr::DefaultInit, importer->materialCount()};
  {
    Cr::Containers::Array<Mn::Shaders::PhongMaterialUniform> materialData{
        Cr::DefaultInit, importer->materialCount()};
    for (std::size_t i = 0; i != materialData.size(); ++i) {
      const Cr::Containers::Optional<Mn::Trade::MaterialData> material =
          importer->material(i);
      CORRADE_INTERNAL_ASSERT(material);
      const auto& flatMaterial = material->as<Mn::Trade::FlatMaterialData>();
      materialData[i].setAmbientColor(flatMaterial.color());

      CORRADE_ASSERT(
          flatMaterial.hasTexture(),
          "Renderer::addFile(): material" << i << "is not textured", );
      CORRADE_ASSERT(flatMaterial.texture() == 0,
                     "Renderer::addFile(): expected material"
                         << i << "to reference the only texture, got"
                         << flatMaterial.texture(), );

      state_->textureTransformations[i] = {
          flatMaterial.attribute<Mn::UnsignedInt>(
              Mn::Trade::MaterialAttribute::BaseColorTextureLayer),
          flatMaterial.hasTextureTransformation() ? flatMaterial.textureMatrix()
                                                  : Mn::Matrix3{}};
    }

    // TODO immutable buffer storage
    state_->materialUniform.setData(materialData);
  }

  /* Fill initial projection data for each view. Will be uploaded afresh every
     draw. */
  state_->projections = Cr::Containers::Array<ProjectionPadded>{
      Cr::DefaultInit, std::size_t(state_->tileCount.product())};
  // TODO (mutable) buffer storage

  {
    CORRADE_ASSERT(importer->sceneCount() == 1,
                   "Renderer::addFile(): expected exactly one scene, got"
                       << importer->sceneCount(), );
    Cr::Containers::Optional<Mn::Trade::SceneData> scene = importer->scene(0);
    CORRADE_INTERNAL_ASSERT(scene);

    /* Populate the mesh and material list */
    const Cr::Containers::Optional<Mn::UnsignedInt> meshViewIndexOffsetFieldId =
        scene->findFieldId(importer->sceneFieldForName("meshViewIndexOffset"));
    CORRADE_ASSERT(meshViewIndexOffsetFieldId,
                   "Renderer::addFile(): no meshViewIndexOffset field in "
                   "the scene", );
    const Cr::Containers::Optional<Mn::UnsignedInt> meshViewIndexCountFieldId =
        scene->findFieldId(importer->sceneFieldForName("meshViewIndexCount"));
    CORRADE_ASSERT(
        meshViewIndexCountFieldId,
        "Renderer::addFile(): no meshViewIndexCount field in the scene", );
    const Cr::Containers::Optional<Mn::UnsignedInt> meshViewMaterialFieldId =
        scene->findFieldId(importer->sceneFieldForName("meshViewMaterial"));
    CORRADE_ASSERT(
        meshViewMaterialFieldId,
        "Renderer::addFile(): no meshViewMaterial field in the scene", );
    /* SceneData and copy() will assert if the types or sizes don't match, so
       we don't have to */
    state_->meshViews = Cr::Containers::Array<MeshView>{
        Cr::NoInit, scene->fieldSize(*meshViewIndexCountFieldId)};
    Cr::Utility::copy(
        scene->field<Mn::UnsignedInt>(*meshViewIndexOffsetFieldId),
        stridedArrayView(state_->meshViews)
            .slice(&MeshView::indexOffsetInBytes));
    Cr::Utility::copy(
        scene->field<Mn::UnsignedInt>(*meshViewIndexCountFieldId),
        stridedArrayView(state_->meshViews).slice(&MeshView::indexCount));
    Cr::Utility::copy(
        scene->field<Mn::Int>(*meshViewMaterialFieldId),
        stridedArrayView(state_->meshViews).slice(&MeshView::materialId));
    /* Transformations of all objects in the scene. Objects that don't have
       this field default to an indentity transform. */
    Cr::Containers::Array<Mn::Matrix4> transformations{
        std::size_t(scene->mappingBound())};
    for (Cr::Containers::Pair<Mn::UnsignedInt, Mn::Matrix4> transformation :
         scene->transformations3DAsArray()) {
      transformations[transformation.first()] = transformation.second();
    }

    /* Populate transforms of all mesh views. Assuming all three fields have
       the same mapping. */
    const Cr::Containers::StridedArrayView1D<const Mn::UnsignedInt>
        meshViewMapping =
            scene->mapping<Mn::UnsignedInt>(*meshViewIndexCountFieldId);
    for (std::size_t i = 0; i != meshViewMapping.size(); ++i) {
      state_->meshViews[i].transformation = transformations[meshViewMapping[i]];
    }

    /* Templates are the root objects with their names. Their immediate
       children are the actual meshes. Assumes the order matches the order of
       the custom fields. */
    // TODO hacky and brittle! doesn't handle nested children properly, doesn't
    //  account for a different order of the field vs the child lists
    Mn::UnsignedInt offset = 0;
    for (Mn::UnsignedLong root : scene->childrenFor(-1)) {
      Cr::Containers::Array<Mn::UnsignedLong> children =
          scene->childrenFor(root);

      Cr::Containers::String name = importer->objectName(root);
      CORRADE_ASSERT(name,
                     "Renderer::addFile(): node" << root << "has no name", );
      state_->meshViewRangeForName.insert(
          {name, {offset, offset + Mn::UnsignedInt(children.size())}});
      offset += children.size();
    }
    CORRADE_INTERNAL_ASSERT(offset = state_->meshViews.size());
  }

  /* Setup a zero-light (flat) shader, bind buffers that don't change
     per-view */
  Mn::Shaders::PhongGL::Flags flags =
      Mn::Shaders::PhongGL::Flag::MultiDraw |
      Mn::Shaders::PhongGL::Flag::UniformBuffers;
  if (!(state_->flags >= RendererFlag::NoTextures))
    flags |= Mn::Shaders::PhongGL::Flag::AmbientTexture |
             Mn::Shaders::PhongGL::Flag::TextureArrays |
             Mn::Shaders::PhongGL::Flag::TextureTransformation;
  // TODO 1024 is 64K divided by 64 bytes needed for one draw uniform, have
  //  that fetched from actual GL limits instead once I get to actually
  //  splitting draws by this limit
  state_->shader =
      Mn::Shaders::PhongGL{flags, 0, importer->materialCount(), 1024};
  state_->shader.bindMaterialBuffer(state_->materialUniform);
  if (!(state_->flags >= RendererFlag::NoTextures))
    state_->shader.bindAmbientTexture(state_->texture);
}

std::size_t Renderer::addMeshHierarchy(const Mn::UnsignedInt sceneId,
                                       const Cr::Containers::StringView name,
                                       const Mn::Matrix4& transformation) {
  CORRADE_ASSERT(sceneId < state_->scenes.size(),
                 "Renderer::add(): index" << sceneId << "out of range for"
                                          << state_->scenes.size() << "scenes",
                 {});

  Scene& scene = state_->scenes[sceneId];
  /* Using a non-owning wrapper over the view to avoid an allocated string copy
     because yes hello STL you're uhhmazing */
  const auto found = state_->meshViewRangeForName.find(
      Cr::Containers::String::nullTerminatedView(name));
  CORRADE_ASSERT(found != state_->meshViewRangeForName.end(),
                 "Renderer::add(): name" << name << "not found", {});

  /* Add a top-level object */
  const std::size_t id = scene.transformations.size();
  // TODO this adds an empty draw, which is useless; do better (separate
  //  transforms from draws)
  arrayAppend(scene.parents, -1);
  arrayAppend(scene.transformations, transformation);
  arrayAppend(scene.absoluteTransformations, Cr::InPlaceInit);
  arrayAppend(scene.draws, Cr::InPlaceInit).setMaterialId(0);
  arrayAppend(scene.textureTransformations, Cr::InPlaceInit).setLayer(0);
  arrayAppend(scene.drawCommands, Cr::InPlaceInit, 0u, 0u);

  /* Add the whole hierarchy under this name */
  for (std::size_t i = found->second.first(); i != found->second.second();
       ++i) {
    const MeshView& meshView = state_->meshViews[i];
    /* The following meshes are children of the first one, inheriting its
       transformation */
    arrayAppend(scene.parents, id);
    arrayAppend(scene.transformations, meshView.transformation);

    /* The actual absolute transformation will get filled each time draw() is
       called */
    arrayAppend(scene.absoluteTransformations, Cr::InPlaceInit);

    arrayAppend(scene.draws, Cr::InPlaceInit)
        .setMaterialId(meshView.materialId);
    arrayAppend(scene.textureTransformations, Cr::InPlaceInit)
        .setTextureMatrix(
            state_->textureTransformations[meshView.materialId].transformation)
        .setLayer(state_->textureTransformations[meshView.materialId].layer);
    arrayAppend(scene.drawCommands, Cr::InPlaceInit,
                meshView.indexOffsetInBytes, meshView.indexCount);
  }

  /* Assuming add() is called relatively infrequently compared to draw(),
     upload the changed draw and texture transform buffers. Transformation
     buffer will be updated in draw(). */
  scene.drawUniform.setData(scene.draws);
  scene.textureTransformationUniform.setData(scene.textureTransformations);

  return id;
}

std::size_t Renderer::addMeshHierarchy(const Mn::UnsignedInt scene,
                                       const Cr::Containers::StringView name) {
  return addMeshHierarchy(scene, name, Mn::Matrix4{});
}

void Renderer::clear(const Mn::UnsignedInt sceneId) {
  CORRADE_ASSERT(sceneId < state_->scenes.size(),
                 "Renderer::clear(): index" << sceneId << "out of range for"
                                            << state_->scenes.size()
                                            << "scenes", );

  Scene& scene = state_->scenes[sceneId];
  // TODO have arrayClear()
  /* Resizing instead of `= {}` to not discard the memory */
  arrayResize(scene.parents, 0);
  arrayResize(scene.transformations, 0);
  /* Keep the root absolute transform here tho (same state as when initially
     constructed) */
  arrayResize(scene.absoluteTransformations, 1);
  arrayResize(scene.draws, 0);
  arrayResize(scene.textureTransformations, 0);
  arrayResize(scene.drawCommands, 0);
}

Mn::Matrix4& Renderer::camera(const Mn::UnsignedInt scene) {
  return state_->projections[scene].projectionMatrix;
}

Cr::Containers::StridedArrayView1D<Mn::Matrix4> Renderer::transformations(
    const Mn::UnsignedInt scene) {
  return state_->scenes[scene].transformations;
}

void Renderer::draw(Mn::GL::AbstractFramebuffer& framebuffer) {
  // TODO allow this (currently addFile() sets up shader limits)
  CORRADE_ASSERT(state_->mesh.id(), "Renderer::draw(): no file was added", );

  /* Calculate absolute transformations */
  for (std::size_t sceneId = 0; sceneId != state_->scenes.size(); ++sceneId) {
    Scene& scene = state_->scenes[sceneId];

    // TODO have a tool for this
    scene.absoluteTransformations[0].setTransformationMatrix(Mn::Matrix4{});
    for (std::size_t i = 0; i != scene.transformations.size(); ++i)
      scene.absoluteTransformations[i + 1].setTransformationMatrix(
          scene.absoluteTransformations[scene.parents[i] + 1]
              .transformationMatrix *
          scene.transformations[i]);
  }

  /* Upload projection and transformation uniforms, assuming they change every
     frame. Do it before the draw loop to minimize stalls. */
  state_->projectionUniform.setData(state_->projections);
  for (std::size_t sceneId = 0; sceneId != state_->scenes.size(); ++sceneId)
    // TODO have this somehow in a single buffer instead
    state_->scenes[sceneId].transformationUniform.setData(
        state_->scenes[sceneId].absoluteTransformations.exceptPrefix(1));

  for (Mn::Int y = 0; y != state_->tileCount.y(); ++y) {
    for (Mn::Int x = 0; x != state_->tileCount.x(); ++x) {
      framebuffer.setViewport(Mn::Range2Di::fromSize(
          Mn::Vector2i{x, y} * state_->tileSize, state_->tileSize));

      const std::size_t scene = y * state_->tileCount.x() + x;

      // TODO split by draw count limit
      state_
          ->shader
          // TODO bind all buffers together with a multi API
          .bindProjectionBuffer(state_->projectionUniform,
                                scene * sizeof(ProjectionPadded),
                                sizeof(ProjectionPadded))
          .bindTransformationBuffer(state_->scenes[scene].transformationUniform)
          .bindDrawBuffer(state_->scenes[scene].drawUniform);
      if (!(state_->flags & RendererFlag::NoTextures))
        state_->shader.bindTextureTransformationBuffer(
            state_->scenes[scene].textureTransformationUniform);
      state_->shader.draw(state_->mesh,
                          stridedArrayView(state_->scenes[scene].drawCommands)
                              .slice(&DrawCommand::indexCount),
                          nullptr,
                          stridedArrayView(state_->scenes[scene].drawCommands)
                              .slice(&DrawCommand::indexOffsetInBytes));
    }
  }
}

}  // namespace gfx_batch
}  // namespace esp
