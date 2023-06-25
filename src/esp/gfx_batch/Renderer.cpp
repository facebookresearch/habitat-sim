// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "Renderer.h"

#include <Corrade/Containers/Array.h>
#include <Corrade/Containers/GrowableArray.h>
#include <Corrade/Containers/Optional.h>
#include <Corrade/Containers/Pair.h>
#include <Corrade/Containers/Reference.h>
#include <Corrade/Containers/StringStl.h>
#include <Corrade/Containers/StringStlHash.h>
#include <Corrade/Containers/Triple.h>
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
#include <Magnum/MeshTools/Duplicate.h>
#include <Magnum/MeshTools/RemoveDuplicates.h>
#include <Magnum/PixelFormat.h>
#include <Magnum/PixelStorage.h>
#include <Magnum/SceneTools/Hierarchy.h>
#include <Magnum/Shaders/Generic.h>
#include <Magnum/Shaders/Phong.h>
#include <Magnum/Shaders/PhongGL.h>
#include <Magnum/Trade/AbstractImporter.h>
#include <Magnum/Trade/FlatMaterialData.h>
#include <Magnum/Trade/ImageData.h>
#include <Magnum/Trade/MeshData.h>
#include <Magnum/Trade/PhongMaterialData.h>
#include <Magnum/Trade/SceneData.h>
#include <Magnum/Trade/TextureData.h>
#include <esp/gfx_batch/DepthUnprojection.h>
#include <unordered_map>

namespace Cr = Corrade;
namespace Mn = Magnum;

namespace esp {
namespace gfx_batch {

// clang-tidy you're NOT HELPING
using namespace Cr::Containers::Literals;  // NOLINT
using namespace Mn::Math::Literals;        // NOLINT

struct RendererConfiguration::State {
  RendererFlags flags;
  Mn::Vector2i tileSize{128, 128};
  Mn::Vector2i tileCount{1, 1};
  Mn::UnsignedInt maxLightCount{0};
  Mn::Float ambientFactor{0.1f};
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

RendererConfiguration& RendererConfiguration::setMaxLightCount(
    Mn::UnsignedInt count) {
  state->maxLightCount = count;
  return *this;
}

RendererConfiguration& RendererConfiguration::setAmbientFactor(
    Mn::Float factor) {
  state->ambientFactor = factor;
  return *this;
}

namespace {

struct MeshView {
  Mn::UnsignedInt meshId;
  Mn::UnsignedInt indexOffsetInBytes;
  Mn::UnsignedInt indexCount;
  Mn::Int materialId; /* is never -1 tho */
  // TODO also parent, when we are able to fetch the whole hierarchy for a
  //  particular root object name instead of having the hierarchy flattened
  Mn::Matrix4 transformation;
};

struct Light {
  std::size_t node;
  RendererLightType type;
  Mn::Color3 color;
  Mn::Float range;
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

struct DrawBatch {
  Mn::Shaders::PhongGL::Flags shaderFlags;
  /* Caches access to the shader map (which is assumed to have stable pointers
     for the whole lifetime) */
  Cr::Containers::Reference<Mn::Shaders::PhongGL> shader;
  Mn::UnsignedInt meshId;
  Mn::UnsignedInt textureId;
  /* Here will eventually also be shader ID and other things like mask etc */
};

/* Finds a draw batch corresponding to given shader, mesh and texture or
   creates a new one, returning its ID. Yes, it's a linear search. You're not
   supposed to have too many meshes and textures, anyway. */
Mn::UnsignedInt drawBatchId(
    Cr::Containers::Array<DrawBatch>& drawBatches,
    std::unordered_map<Mn::UnsignedInt, Mn::Shaders::PhongGL>& shaders,
    Mn::Shaders::PhongGL::Flags shaderFlags,
    Mn::UnsignedInt meshId,
    Mn::UnsignedInt textureId) {
  for (std::size_t i = 0; i != drawBatches.size(); ++i) {
    if (drawBatches[i].shaderFlags == shaderFlags &&
        drawBatches[i].meshId == meshId &&
        drawBatches[i].textureId == textureId)
      return i;
  }

  arrayAppend(drawBatches, Cr::InPlaceInit, shaderFlags,
              shaders.at(Mn::UnsignedInt(shaderFlags)), meshId, textureId);
  return drawBatches.size() - 1;
}

struct Scene {
  /* Camera unprojection. Updated from updateCamera(). */
  Mn::Vector2 cameraUnprojection;

  /* Node parents and transformations. Appended to with add(). Some of these
     (but not all) are referenced from the transformationIds array below. */
  Cr::Containers::Array<Mn::Int> parents; /* parents[i] < i, always */
  Cr::Containers::Array<Mn::Matrix4> transformations;
  /* Lights, with node IDs referencing transformations from above */
  Cr::Containers::Array<Light> lights;

  /* Draw batches, each being a unique combination of a mesh and a texture,
     thus requiring a dedicated draw call. See also drawBatchId(). */
  // TODO might make sense to order this (by shader,) by mesh, then by texture
  Cr::Containers::Array<DrawBatch> drawBatches;

  /* Data for all draws, kept in the same order as was appended to with add().
     The transformationIds points to the transformations array from above,
     uniform buffers and the draw command get the data sorted by
     drawBatchIds. */
  Cr::Containers::Array<Mn::UnsignedInt> drawBatchIds;
  Cr::Containers::Array<Mn::UnsignedInt> transformationIds;
  Cr::Containers::Array<Mn::Shaders::PhongDrawUniform> draws;
  Cr::Containers::Array<Mn::Shaders::TextureTransformationUniform>
      textureTransformations;
  Cr::Containers::Array<DrawCommand> drawCommands;

  /* The transformationIds and drawCommands arrays sorted by meshIds. The
     textureTransformations array is uploaded to uniform buffers after sorting
     and not used from the CPU again so it doesn't need to be cached here. */
  Cr::Containers::Array<Mn::Shaders::PhongDrawUniform> drawsSorted;
  Cr::Containers::Array<Mn::UnsignedInt> transformationIdsSorted;
  // TODO make the layout match GL (... deinterleave) to avoid a copy in draw()
  //  or maybe not and just go with draw indirect directly
  Cr::Containers::Array<Mn::UnsignedInt> drawBatchOffsets;
  Cr::Containers::Array<DrawCommand> drawCommandsSorted;

  /* Updated every frame */
  // TODO make these two global, uploaded just once (plus accounting for
  //  padding)
  Mn::GL::Buffer transformationUniform;
  Mn::GL::Buffer lightUniform;
  /* Updated at most once a frame if dirty is true */
  // TODO split for lights vs nodes?
  bool dirty = false;
  Mn::GL::Buffer drawUniform;
  Mn::GL::Buffer textureTransformationUniform;
};

struct TextureTransformation {
  Mn::UnsignedInt textureId;
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
  Mn::UnsignedInt maxLightCount;
  Mn::Float ambientFactor;
  /* Indexed with Mn::Shaders::PhongGL::Flag, but I don't want to bother with
     writing a hash function for EnumSet */
  // TODO have a dedicated shader for flat materials
  // TODO add Containers/EnumSetHash.h for this
  std::unordered_map<Mn::UnsignedInt, Mn::Shaders::PhongGL> shaders;

  /* Filled upon addFile() */
  Cr::Containers::Array<Mn::GL::Texture2DArray> textures;
  /* Each mesh contains a set of flags it needs from the shader (such as
     enabling vertex colors) */
  Cr::Containers::Array<
      Cr::Containers::Pair<Mn::Shaders::PhongGL::Flags, Mn::GL::Mesh>>
      meshes;
  // TODO clear this array once/if the materialUniform is populated on first
  //  draw() and adding more files is forbidden
  Cr::Containers::Array<Mn::Shaders::PhongMaterialUniform> materials;
  /* Contains texture transform and layer for each material. Used by add() to
     populate the draw list. */
  Cr::Containers::Array<TextureTransformation> materialTextureTransformations;

  /* Mesh views (mesh ID, index byte offset and count), material IDs and
     initial transformations for draws. Used by add() to populate the draw
     list. */
  Cr::Containers::Array<MeshView> meshViews;
  /* Range of mesh views and materials corresponding to a particular name */
  std::unordered_map<Cr::Containers::String,
                     Cr::Containers::Pair<Mn::UnsignedInt, Mn::UnsignedInt>>
      meshViewRangeForName;

  /* Updated from addFile() */
  Mn::GL::Buffer materialUniform;
  /* Combined view and projection matrices. Updated from updateCamera() */
  Cr::Containers::Array<ProjectionPadded> cameraMatrices;
  /* Updated from draw() every frame */
  Mn::GL::Buffer projectionUniform;

  Cr::Containers::Array<Scene> scenes;

  /* Temporary per-frame state to avoid allocating inside each draw() */
  // TODO might be useful to have some per-frame bump allocator instead, for
  //  smaller peak memory use
  Cr::Containers::Array<Mn::Shaders::TransformationUniform3D>
      absoluteTransformations;
  Cr::Containers::Array<Mn::Shaders::TransformationUniform3D>
      absoluteTransformationsSorted;
  Cr::Containers::Array<Mn::Shaders::PhongLightUniform> absoluteLights;
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
  state_->maxLightCount = configuration.maxLightCount;
  state_->ambientFactor = configuration.ambientFactor;
  const std::size_t sceneCount = configuration.tileCount.product();
  state_->cameraMatrices = Cr::Containers::Array<ProjectionPadded>{sceneCount};
  state_->scenes = Cr::Containers::Array<Scene>{sceneCount};

  /* Texture 0 is reserved as a white pixel */
  // TODO drop this altogether and use an untextured shader instead? since it's
  //  causing a dedicated draw call anyway
  arrayAppend(state_->textures, Cr::InPlaceInit)
      .setMinificationFilter(Mn::SamplerFilter::Nearest,
                             Mn::SamplerMipmap::Base)
      .setMagnificationFilter(Mn::SamplerFilter::Nearest)
      .setWrapping(Mn::SamplerWrapping::Repeat)
      .setStorage(1, Mn::GL::TextureFormat::RGBA8, {1, 1, 1})
      .setSubImage(
          0, {},
          Mn::ImageView3D{
              Mn::PixelFormat::RGBA8Unorm, {1, 1, 1}, "\xff\xff\xff\xff"});

  /* Material 0 is reserved as a white ambient with no texture */
  arrayAppend(state_->materials, Cr::InPlaceInit)
      .setAmbientColor(0xffffff_rgbf);
  arrayAppend(state_->materialTextureTransformations, Cr::InPlaceInit);

  // TODO move this outside
  Mn::GL::Renderer::enable(Mn::GL::Renderer::Feature::FaceCulling);
  Mn::GL::Renderer::enable(Mn::GL::Renderer::Feature::DepthTest);
}

void Renderer::destroy() {
  state_ = {};
}

Renderer::~Renderer() = default;

RendererFlags Renderer::flags() const {
  return state_->flags;
}

Mn::Vector2i Renderer::tileCount() const {
  return state_->tileCount;
}

Mn::Vector2i Renderer::tileSize() const {
  return state_->tileSize;
}

std::size_t Renderer::sceneCount() const {
  return state_->scenes.size();
}

Mn::UnsignedInt Renderer::maxLightCount() const {
  return state_->maxLightCount;
}

bool Renderer::addFile(const Cr::Containers::StringView filename,
                       const RendererFileFlags flags) {
  return addFile(filename, "AnySceneImporter", flags);
}

bool Renderer::addFile(const Cr::Containers::StringView filename,
                       const RendererFileFlags flags,
                       Cr::Containers::StringView name) {
  return addFile(filename, "AnySceneImporter", flags, name);
}

bool Renderer::addFile(const Cr::Containers::StringView filename,
                       const Cr::Containers::StringView importerPlugin,
                       const RendererFileFlags flags) {
  return addFile(filename, importerPlugin, flags, {});
}

bool Renderer::addFile(const Cr::Containers::StringView filename,
                       const Cr::Containers::StringView importerPlugin,
                       const RendererFileFlags flags,
                       const Cr::Containers::StringView name) {
  Cr::PluginManager::Manager<Mn::Trade::AbstractImporter> manager;
  Cr::Containers::Pointer<Mn::Trade::AbstractImporter> importer =
      manager.loadAndInstantiate(importerPlugin);
  CORRADE_ASSERT(importer, "Renderer::addFile(): can't load importer plugin",
                 {});
  // TODO: gfx_batch logging subsystem needed: set importer flags if gfx logging
  // is quieted if(!isLevelEnabled(logging::Subsystem::gfx_batch,
  // logging::LoggingLevel::Debug)){
  //   importer->addFlags(Magnum::Trade::ImporterFlag::Quiet);
  // }else if (isLevelEnabled(logging::Subsystem::gfx_batch,
  //                    logging::LoggingLevel::VeryVerbose)){
  //  //set verbose flags if necessary
  //  importer->addFlags(Mn::Trade::ImporterFlag::Verbose);
  //}

  /* Set up options for glTF import. We can also import any other files (such
     as serialized magnum blobs or BPS files), assume these don't need any
     custom setup. */
  if (importerPlugin.contains("GltfImporter") ||
      (importerPlugin.contains("AnySceneImporter") &&
       (filename.contains(".gltf") || filename.contains(".glb")))) {
    // TODO implement and use a singular
    // ignoreRequiredExtension=MAGNUMX_mesh_views
    //  that doesn't produce warnings
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
  if (!importer->openFile(filename)) {
    Mn::Error{} << "Renderer::addFile(): can't open the file";
    return {};
  }

  /* Remember the count of data already present to offset the references with
     them */
  const Mn::UnsignedInt textureOffset = state_->textures.size();
  const Mn::UnsignedInt meshOffset = state_->meshes.size();
  const Mn::UnsignedInt meshViewOffset = state_->meshViews.size();
  const Mn::UnsignedInt materialOffset = state_->materials.size();

  /* Import all textures */
  if (!(state_->flags & RendererFlag::NoTextures)) {
    for (Mn::UnsignedInt i = 0, iMax = importer->textureCount(); i != iMax;
         ++i) {
      const Cr::Containers::Optional<Mn::Trade::TextureData> textureData =
          importer->texture(i);
      if (!textureData) {
        Mn::Error{} << "Renderer::addFile(): can't import texture" << i << "of"
                    << filename;
        return {};
      }

      /* 2D textures are imported as single-layer 2D array textures */
      Mn::GL::Texture2DArray texture;
      if (textureData->type() == Mn::Trade::TextureType::Texture2DArray) {
        const Mn::UnsignedInt levelCount =
            importer->image3DLevelCount(textureData->image());
        Cr::Containers::Optional<Mn::Trade::ImageData3D> image =
            importer->image3D(textureData->image());
        if (!image) {
          Mn::Error{} << "Renderer::addFile(): can't import 3D image"
                      << textureData->image() << "of" << filename;
          return {};
        }

        /* Generate a full mipmap if there's just one level and if the image is
           not compressed. It's opt-in to force people to learn how to make
           assets Vulkan-ready. */
        const bool generateMipmap =
            levelCount == 1 && (flags & RendererFileFlag::GenerateMipmap) &&
            !image->isCompressed();
        const Mn::UnsignedInt desiredLevelCount =
            generateMipmap ? Mn::Math::log2(image->size().xy().min()) + 1
                           : levelCount;

        texture
            .setMinificationFilter(textureData->minificationFilter(),
                                   textureData->mipmapFilter())
            .setMagnificationFilter(textureData->magnificationFilter())
            .setWrapping(textureData->wrapping().xy());
        if (image->isCompressed()) {
          texture
              .setStorage(levelCount,
                          Mn::GL::textureFormat(image->compressedFormat()),
                          image->size())
              .setCompressedSubImage(0, {}, *image);
          for (Mn::UnsignedInt level = 1; level != levelCount; ++level) {
            Cr::Containers::Optional<Mn::Trade::ImageData3D> levelImage =
                importer->image3D(textureData->image(), level);
            CORRADE_INTERNAL_ASSERT(levelImage && levelImage->isCompressed() &&
                                    levelImage->compressedFormat() ==
                                        image->compressedFormat());
            texture.setCompressedSubImage(level, {}, *levelImage);
          }
        } else {
          texture
              .setStorage(desiredLevelCount,
                          Mn::GL::textureFormat(image->format()), image->size())
              .setSubImage(0, {}, *image);
          if (generateMipmap)
            texture.generateMipmap();
        }
      } else if (textureData->type() == Mn::Trade::TextureType::Texture2D) {
        const Mn::UnsignedInt levelCount =
            importer->image2DLevelCount(textureData->image());
        Cr::Containers::Optional<Mn::Trade::ImageData2D> image =
            importer->image2D(textureData->image());
        if (!image) {
          Mn::Error{} << "Renderer::addFile(): can't import 2D image"
                      << textureData->image() << "of" << filename;
          return {};
        }

        /* Generate a full mipmap if there's just one level and if the image is
           not compressed. It's opt-in to force people to learn how to make
           assets Vulkan-ready. */
        const bool generateMipmap =
            levelCount == 1 && (flags & RendererFileFlag::GenerateMipmap) &&
            !image->isCompressed();
        const Mn::UnsignedInt desiredLevelCount =
            generateMipmap ? Mn::Math::log2(image->size().min()) + 1
                           : levelCount;

        texture
            .setMinificationFilter(textureData->minificationFilter(),
                                   textureData->mipmapFilter())
            .setMagnificationFilter(textureData->magnificationFilter())
            .setWrapping(textureData->wrapping().xy());
        if (image->isCompressed()) {
          texture
              .setStorage(levelCount,
                          Mn::GL::textureFormat(image->compressedFormat()),
                          {image->size(), 1})
              .setCompressedSubImage(0, {}, Mn::CompressedImageView2D{*image});
          for (Mn::UnsignedInt level = 1; level != levelCount; ++level) {
            Cr::Containers::Optional<Mn::Trade::ImageData2D> levelImage =
                importer->image2D(textureData->image(), level);
            CORRADE_INTERNAL_ASSERT(levelImage && levelImage->isCompressed() &&
                                    levelImage->compressedFormat() ==
                                        image->compressedFormat());
            texture.setCompressedSubImage(
                level, {}, Mn::CompressedImageView2D{*levelImage});
          }
        } else {
          texture
              .setStorage(desiredLevelCount,
                          Mn::GL::textureFormat(image->format()),
                          {image->size(), 1})
              .setSubImage(0, {}, Mn::ImageView2D{*image});
          if (generateMipmap)
            texture.generateMipmap();
        }
      } else
        CORRADE_INTERNAL_ASSERT_UNREACHABLE(); /* LCOV_EXCL_LINE */

      arrayAppend(state_->textures, std::move(texture));
    }
  }

  /* Import all meshes */
  for (Mn::UnsignedInt i = 0, iMax = importer->meshCount(); i != iMax; ++i) {
    Cr::Containers::Optional<Mn::Trade::MeshData> mesh = importer->mesh(i);
    if (!mesh) {
      Mn::Error{} << "Renderer::addFile(): can't import mesh" << i << "of"
                  << filename;
      return {};
    }

    /* Make the mesh indexed if it isn't */
    if (!mesh->isIndexed())
      mesh = Mn::MeshTools::removeDuplicates(*mesh);

    /* Decide what extra shader feature the mesh needs. Currently just vertex
       colors. */
    Mn::Shaders::PhongGL::Flags flags;
    if (mesh->hasAttribute(Mn::Trade::MeshAttribute::Color))
      flags |= Mn::Shaders::PhongGL::Flag::VertexColor;

    arrayAppend(state_->meshes, Cr::InPlaceInit, flags,
                Mn::MeshTools::compile(*mesh));
  }

  /* Immutable material data. Save texture IDs, transformations and layers to a
     temporary array to apply them to draws instead */
  {
    CORRADE_INTERNAL_ASSERT(state_->materials.size() ==
                            state_->materialTextureTransformations.size());
    Cr::Containers::ArrayView<Mn::Shaders::PhongMaterialUniform> materials =
        arrayAppend(state_->materials, Cr::NoInit, importer->materialCount());
    Cr::Containers::ArrayView<TextureTransformation>
        materialTextureTransformations =
            arrayAppend(state_->materialTextureTransformations, Cr::NoInit,
                        importer->materialCount());
    for (std::size_t i = 0; i != importer->materialCount(); ++i) {
      const Cr::Containers::Optional<Mn::Trade::MaterialData> material =
          importer->material(i);
      if (!material) {
        Mn::Error{} << "Renderer::addFile(): can't import material" << i << "of"
                    << filename;
        return {};
      }

      /* Set up flat shading either if the material is marked as flat or if
         we have no lights enabled */
      if (material->types() & Mn::Trade::MaterialType::Flat ||
          !state_->maxLightCount) {
        const auto& flatMaterial = material->as<Mn::Trade::FlatMaterialData>();
        materials[i] = Mn::Shaders::PhongMaterialUniform{}
                           .setAmbientColor(flatMaterial.color())
                           /* Diffuse is zero so lights (if enabled) have no
                              effect on this material  */
                           .setDiffuseColor(0x00000000_rgbaf);

        /* Untextured materials get the first reserved texture (a white
           pixel) */
        if (!flatMaterial.hasTexture())
          materialTextureTransformations[i] = {0, 0, {}};
        else
          materialTextureTransformations[i] = {
              flatMaterial.texture() + textureOffset,
              flatMaterial.textureLayer(), flatMaterial.textureMatrix()};
      } else {
        const auto& phongMaterial =
            material->as<Mn::Trade::PhongMaterialData>();
        materials[i] = Mn::Shaders::PhongMaterialUniform{}
                           .setAmbientColor(phongMaterial.diffuseColor() *
                                            state_->ambientFactor)
                           .setDiffuseColor(phongMaterial.diffuseColor())
            /* Specular not used (and shader compiled with NoSpecular). Much
               plastic. Very fugly. Thus we also don't need shininess for
               anything (and it's not imported from the glTF anyway). */
            ;

        /* Untextured materials get the first reserved texture (a white
           pixel) */
        if (!phongMaterial.hasAttribute(
                Mn::Trade::MaterialAttribute::DiffuseTexture))
          materialTextureTransformations[i] = {0, 0, {}};
        else
          materialTextureTransformations[i] = {
              phongMaterial.diffuseTexture() + textureOffset,
              phongMaterial.diffuseTextureLayer(),
              phongMaterial.diffuseTextureMatrix()};
      }
    }

    // TODO immutable buffer storage how? or populate on first draw() and then
    //  FORBID adding more files?
    state_->materialUniform.setData(state_->materials);
  }

  /* Fill initial projection data for each view. Will be uploaded afresh every
     draw. */
  state_->cameraMatrices = Cr::Containers::Array<ProjectionPadded>{
      Cr::DefaultInit, std::size_t(state_->tileCount.product())};
  // TODO (mutable) buffer storage

  /* Scene-less files are assumed to contain a single material-less mesh (such
     as STL files) */
  if (!importer->sceneCount()) {
    if (importer->meshCount() != 1 || importer->materialCount()) {
      Mn::Error{} << "Renderer::addFile(): expected exactly one mesh and no "
                     "material for a scene-less file"
                  << filename;
      return {};
    }

    /* Material 0 is reserved for such purposes */
    MeshView& view = arrayAppend(state_->meshViews, Cr::InPlaceInit);
    view.meshId = meshOffset;
    view.indexOffsetInBytes = 0;
    view.indexCount = state_->meshes[meshOffset].second().count();
    view.materialId = 0;

    /* Adding a scene-less file as a whole should be explicitly requested to
       avoid accidents */
    CORRADE_ASSERT(flags & RendererFileFlag::Whole,
                   "Renderer::addFile(): scene-less file"
                       << filename
                       << "has to be added with RendererFileFlag::Whole",
                   {});

    /* If no name is specified, the full filename is used */
    const Cr::Containers::StringView usedName = name ? name : filename;
    if (!state_->meshViewRangeForName
             .insert({usedName, {meshViewOffset, meshViewOffset + 1}})
             .second) {
      Mn::Error{} << "Renderer::addFile(): node name" << usedName << "in"
                  << filename << "already exists";
      return {};
    }

  } else {
    if (importer->sceneCount() != 1) {
      Mn::Error{} << "Renderer::addFile(): expected exactly one scene, got"
                  << importer->sceneCount() << "in" << filename;
      return {};
    }

    Cr::Containers::Optional<Mn::Trade::SceneData> scene = importer->scene(0);
    CORRADE_INTERNAL_ASSERT(scene);

    /* Populate the mesh and material list. SceneData and copy() will assert if
       the types or sizes don't match, so we don't have to */
    Cr::Containers::StridedArrayView1D<MeshView> meshViews =
        arrayAppend(state_->meshViews, Cr::NoInit,
                    scene->fieldSize(Mn::Trade::SceneField::Mesh));
    Cr::Utility::copy(
        scene->field<Mn::UnsignedInt>(Mn::Trade::SceneField::Mesh),
        meshViews.slice(&MeshView::meshId));

    /* Add mesh offset to all mesh IDs */
    for (Mn::UnsignedInt& meshId : meshViews.slice(&MeshView::meshId))
      meshId += meshOffset;

    /* If there are mesh view fields, it's a batch-friendly file. This is
       independent of the RendererFileFlag::Whole setting, as ultimately mesh
       views will be a builtin feature and thus any imported file can have
       them. */
    if (const Cr::Containers::Optional<Mn::UnsignedInt>
            meshViewIndexOffsetFieldId = scene->findFieldId(
                importer->sceneFieldForName("meshViewIndexOffset"))) {
      const Cr::Containers::Optional<Mn::UnsignedInt>
          meshViewIndexCountFieldId = scene->findFieldId(
              importer->sceneFieldForName("meshViewIndexCount"));
      if (!meshViewIndexCountFieldId) {
        Mn::Error{} << "Renderer::addFile(): no meshViewIndexCount field in "
                       "the scene in"
                    << filename;
        return {};
      }

      const Cr::Containers::Optional<Mn::UnsignedInt> meshViewMaterialFieldId =
          scene->findFieldId(importer->sceneFieldForName("meshViewMaterial"));
      if (!meshViewMaterialFieldId) {
        Mn::Error{}
            << "Renderer::addFile(): no meshViewMaterial field in the scene in"
            << filename;
        return {};
      }

      Cr::Utility::copy(
          scene->field<Mn::UnsignedInt>(*meshViewIndexOffsetFieldId),
          meshViews.slice(&MeshView::indexOffsetInBytes));
      Cr::Utility::copy(
          scene->field<Mn::UnsignedInt>(*meshViewIndexCountFieldId),
          meshViews.slice(&MeshView::indexCount));
      Cr::Utility::copy(scene->field<Mn::Int>(*meshViewMaterialFieldId),
                        meshViews.slice(&MeshView::materialId));

      /* If there are no mesh view fields, offsets are always zero, counts go
         directly from meshes and materials from the builtin attribute */
    } else {
      for (MeshView& view : meshViews) {
        view.indexOffsetInBytes = 0;
        view.indexCount = state_->meshes[view.meshId].second().count();
      }
      /* Not accessing SceneField::MeshMaterial directly, as it might not even
         be there */
      scene->meshesMaterialsInto(nullptr, nullptr,
                                 meshViews.slice(&MeshView::materialId));
    }

    /* Add material offset to all material IDs. If the material is -1, use the
       default material (0). */
    for (Mn::Int& materialId : meshViews.slice(&MeshView::materialId)) {
      if (materialId == -1)
        materialId = 0;
      else
        materialId += materialOffset;
    }

    /* Unless the file is treated as a whole, root scene nodes are used as
       "named templates" to be referenced from addNodeHierarchy(). */
    if (!(flags & RendererFileFlag::Whole)) {
      /* Transformations of all objects in the scene. Objects that don't have
         this field default to an identity transform. */
      Cr::Containers::Array<Mn::Matrix4> transformations{
          std::size_t(scene->mappingBound())};
      for (Cr::Containers::Pair<Mn::UnsignedInt, Mn::Matrix4> transformation :
           scene->transformations3DAsArray()) {
        transformations[transformation.first()] = transformation.second();
      }

      /* Populate transforms of all mesh views. Assuming all mesh-related
         fields (mesh, mesh view index count/offset, mesh material) have the
         same mapping. */
      const Cr::Containers::StridedArrayView1D<const Mn::UnsignedInt>
          meshViewMapping =
              scene->mapping<Mn::UnsignedInt>(Mn::Trade::SceneField::Mesh);
      for (std::size_t i = 0; i != meshViewMapping.size(); ++i) {
        meshViews[i].transformation = transformations[meshViewMapping[i]];
      }

      /* Templates are the root objects with their names. Their immediate
         children are the actual meshes. Assumes the order matches the order of
         the custom fields. */
      // TODO hacky and brittle! doesn't handle nested children properly,
      //  doesn't account for a different order of the field vs the child lists
      Mn::UnsignedInt offset = meshViewOffset;
      for (Mn::UnsignedLong root : scene->childrenFor(-1)) {
        Cr::Containers::Array<Mn::UnsignedLong> children =
            scene->childrenFor(root);

        Cr::Containers::String name = importer->objectName(root);
        if (!name) {
          Mn::Error{} << "Renderer::addFile(): node" << root << "in" << filename
                      << "has no name";
          return {};
        }
        if (!state_->meshViewRangeForName
                 .insert({name,
                          {offset, offset + Mn::UnsignedInt(children.size())}})
                 .second) {
          Mn::Error{} << "Renderer::addFile(): node name" << name << "in"
                      << filename << "already exists";
          return {};
        }

        offset += children.size();
      }
      CORRADE_INTERNAL_ASSERT(offset == state_->meshViews.size());

      /* Files treated as a whole have their hierarchy flattened and added under
         a single name, which is the filename */
    } else {
      /* The returned transformations are in the same order as the Mesh field,
         which is the same order as the meshViews array */
      Cr::Containers::Array<Mn::Matrix4> transformations =
          Mn::SceneTools::absoluteFieldTransformations3D(
              *scene, Mn::Trade::SceneField::Mesh);
      for (std::size_t i = 0; i != transformations.size(); ++i) {
        meshViews[i].transformation = transformations[i];
      }
      /* If no name is specified, the full filename is used */
      const Cr::Containers::StringView usedName = name ? name : filename;
      if (!state_->meshViewRangeForName
               .insert(
                   {usedName,
                    {meshViewOffset,
                     meshViewOffset + Mn::UnsignedInt(transformations.size())}})
               .second) {
        Mn::Error{} << "Renderer::addFile(): node name" << usedName
                    << "already exists";
        return {};
      }
      CORRADE_INTERNAL_ASSERT(meshViewOffset + transformations.size() ==
                              state_->meshViews.size());
    }
  }

  /* Setup a zero-light (flat) shader in desired combinations. For simplicity
     and stutter-free experience instantiate all possibly needed combinations
     upfront instead of lazy-compiling them once needed. */
  // TODO don't do this after adding each and every file, it's wasteful ...
  //  do lazily on first draw() and then FORBID adding more files?
  // TODO also might make sense to use async compilation when the combination
  //  count grows further
  for (Mn::Shaders::PhongGL::Flags extraFlags :
       {{}, Mn::Shaders::PhongGL::Flag::VertexColor}) {
    Mn::Shaders::PhongGL::Flags shaderFlags =
        extraFlags | Mn::Shaders::PhongGL::Flag::MultiDraw |
        Mn::Shaders::PhongGL::Flag::UniformBuffers |
        Mn::Shaders::PhongGL::Flag::NoSpecular |
        Mn::Shaders::PhongGL::Flag::LightCulling;
    if (!(state_->flags >= RendererFlag::NoTextures)) {
      shaderFlags |= Mn::Shaders::PhongGL::Flag::AmbientTexture |
                     Mn::Shaders::PhongGL::Flag::TextureArrays |
                     Mn::Shaders::PhongGL::Flag::TextureTransformation;
      /* Enable also a diffuse texture if we're rendering with lights */
      if (state_->maxLightCount)
        shaderFlags |= Mn::Shaders::PhongGL::Flag::DiffuseTexture;
    }
    /* Not using emplace() -- if a stale shader already exists in the map, it
       wouldn't replace it */
    // TODO 1024 is 64K divided by 64 bytes needed for one draw uniform, have
    //  that fetched from actual GL limits instead once I get to actually
    //  splitting draws by this limit
    state_->shaders[Mn::UnsignedInt(extraFlags)] = Mn::Shaders::PhongGL{
        Mn::Shaders::PhongGL::Configuration{}
            .setFlags(shaderFlags)
            .setLightCount(state_->maxLightCount)
            .setMaterialCount(Mn::UnsignedInt(state_->materials.size()))
            .setDrawCount(1024)};
  }

  /* Bind buffers that don't change per-view. All shaders share the same
     binding points so it's fine to use an arbitrary one */
  state_->shaders.begin()->second.bindMaterialBuffer(state_->materialUniform);
  return true;
}

bool Renderer::hasNodeHierarchy(const Cr::Containers::StringView name) const {
  /* Using a non-owning wrapper over the view to avoid an allocated string copy
     because yes hello STL you're uhhmazing */
  // TODO return Optional<UnsignedInt> that can be reused in a subsequent
  //  addNodeHierarchy() call, in case the lookup proves to be too slow or in
  //  case the same string is about to be added many times
  return state_->meshViewRangeForName.find(
             Cr::Containers::String::nullTerminatedView(name)) !=
         state_->meshViewRangeForName.end();
}

std::size_t Renderer::addNodeHierarchy(const Mn::UnsignedInt sceneId,
                                       const Cr::Containers::StringView name,
                                       const Mn::Matrix4& bakeTransformation) {
  CORRADE_ASSERT(sceneId < state_->scenes.size(),
                 "Renderer::addNodeHierarchy(): index"
                     << sceneId << "out of range for" << state_->scenes.size()
                     << "scenes",
                 {});

  Scene& scene = state_->scenes[sceneId];
  /* Using a non-owning wrapper over the view to avoid an allocated string copy
     because yes hello STL you're uhhmazing */
  const auto found = state_->meshViewRangeForName.find(
      Cr::Containers::String::nullTerminatedView(name));
  CORRADE_ASSERT(found != state_->meshViewRangeForName.end(),
                 "Renderer::add(): name" << name << "not found", {});

  /* The parent and transformation arrays should have the same size */
  CORRADE_INTERNAL_ASSERT(scene.transformations.size() == scene.parents.size());

  /* The per-draw arrays should have the same size */
  CORRADE_INTERNAL_ASSERT(scene.transformationIds.size() ==
                          scene.drawBatchIds.size());
  CORRADE_INTERNAL_ASSERT(scene.draws.size() == scene.drawBatchIds.size());
  CORRADE_INTERNAL_ASSERT(scene.textureTransformations.size() ==
                          scene.drawBatchIds.size());
  CORRADE_INTERNAL_ASSERT(scene.drawCommands.size() ==
                          scene.drawBatchIds.size());
  CORRADE_INTERNAL_ASSERT(scene.drawsSorted.size() ==
                          scene.drawBatchIds.size());
  CORRADE_INTERNAL_ASSERT(scene.transformationIdsSorted.size() ==
                          scene.drawBatchIds.size());
  CORRADE_INTERNAL_ASSERT(scene.drawCommandsSorted.size() ==
                          scene.drawBatchIds.size());

  /* Add a top-level object with no attached mesh */
  const std::size_t topLevelId = scene.transformations.size();
  arrayAppend(scene.parents, -1);
  arrayAppend(scene.transformations, Cr::InPlaceInit);

  /* Add the whole hierarchy under this name, with a mesh for each */
  // TODO the hierarchy can eventually also have meshless "grouping nodes" or
  //  also more meshes per node, account for that
  for (std::size_t i = found->second.first(); i != found->second.second();
       ++i) {
    const MeshView& meshView = state_->meshViews[i];
    /* The following meshes are children of the first one, inheriting its
       transformation */
    const std::size_t id = scene.transformations.size();
    arrayAppend(scene.parents, topLevelId);
    arrayAppend(scene.transformations,
                bakeTransformation * meshView.transformation);

    /* Get a batch ID for given shader/mesh/texture combination */
    const Mn::UnsignedInt batchId = drawBatchId(
        scene.drawBatches, state_->shaders,
        state_->meshes[meshView.meshId].first(), meshView.meshId,
        state_->materialTextureTransformations[meshView.materialId].textureId);

    arrayAppend(scene.drawBatchIds, batchId);
    arrayAppend(scene.transformationIds, id);
    arrayAppend(scene.draws, Cr::InPlaceInit)
        .setMaterialId(meshView.materialId);
    arrayAppend(scene.textureTransformations, Cr::InPlaceInit)
        .setTextureMatrix(
            state_->materialTextureTransformations[meshView.materialId]
                .transformation)
        .setLayer(
            state_->materialTextureTransformations[meshView.materialId].layer);
    arrayAppend(scene.drawCommands, Cr::InPlaceInit,
                meshView.indexOffsetInBytes, meshView.indexCount);
    /* Just to have them with the right size, they get filled in a next dirty
       state update in draw() */
    arrayAppend(scene.drawsSorted, Cr::NoInit, 1);
    arrayAppend(scene.transformationIdsSorted, Cr::NoInit, 1);
    arrayAppend(scene.drawCommandsSorted, Cr::NoInit, 1);
  }

  /* Schedule an update next time draw() is called */
  scene.dirty = true;
  return topLevelId;
}

std::size_t Renderer::addNodeHierarchy(const Mn::UnsignedInt scene,
                                       const Cr::Containers::StringView name) {
  return addNodeHierarchy(scene, name, Mn::Matrix4{});
}

std::size_t Renderer::addEmptyNode(const Mn::UnsignedInt sceneId) {
  CORRADE_ASSERT(sceneId < state_->scenes.size(),
                 "Renderer::addEmptyNode(): index"
                     << sceneId << "out of range for" << state_->scenes.size()
                     << "scenes",
                 {});

  Scene& scene = state_->scenes[sceneId];

  /* The parent and transformation arrays should have the same size */
  CORRADE_INTERNAL_ASSERT(scene.transformations.size() == scene.parents.size());

  /* Add a top-level object with no attached mesh */
  const std::size_t id = scene.transformations.size();
  arrayAppend(scene.parents, -1);
  arrayAppend(scene.transformations, Cr::InPlaceInit);

  /* Not marking the dirty bit as nothing changed rendering-wise, and the
     transformations are processed every frame anyway */
  return id;
}

std::size_t Renderer::addLight(const Mn::UnsignedInt sceneId,
                               const std::size_t nodeId,
                               const RendererLightType type) {
  CORRADE_ASSERT(state_->maxLightCount,
                 "Renderer::addLight(): max light count is zero", {});
  CORRADE_ASSERT(sceneId < state_->scenes.size(),
                 "Renderer::addLight(): index" << sceneId << "out of range for"
                                               << state_->scenes.size()
                                               << "scenes",
                 {});

  Scene& scene = state_->scenes[sceneId];
  CORRADE_ASSERT(nodeId < scene.parents.size(),
                 "Renderer::addLight(): index" << nodeId << "out of range for"
                                               << scene.parents.size()
                                               << "nodes in scene" << sceneId,
                 {});

  const std::size_t id = scene.lights.size();
  arrayAppend(scene.lights, Cr::InPlaceInit, nodeId, type, 0xffffff_rgbf,
              Mn::Constants::inf());

  /* Not marking the dirty bit as lights are processed with updated
     transformations every frame anyway */
  return id;
}

void Renderer::clear(const Mn::UnsignedInt sceneId) {
  CORRADE_ASSERT(sceneId < state_->scenes.size(),
                 "Renderer::clear(): index" << sceneId << "out of range for"
                                            << state_->scenes.size()
                                            << "scenes", );

  Scene& scene = state_->scenes[sceneId];
  // TODO have arrayClear()!!!
  /* Resizing instead of `= {}` to not discard the memory */
  arrayResize(scene.parents, 0);
  arrayResize(scene.transformations, 0);
  arrayResize(scene.lights, 0);
  arrayResize(scene.drawBatchIds, 0);
  arrayResize(scene.transformationIds, 0);
  arrayResize(scene.drawBatches, Cr::NoInit, 0);
  arrayResize(scene.draws, 0);
  arrayResize(scene.textureTransformations, 0);
  arrayResize(scene.drawCommands, 0);
  arrayResize(scene.drawsSorted, 0);
  arrayResize(scene.transformationIdsSorted, 0);
  arrayResize(scene.drawCommandsSorted, 0);

  /* There's nothing in the scene, so there's no dirty state to process */
  scene.dirty = false;
}

void Renderer::clearLights(const Mn::UnsignedInt sceneId) {
  CORRADE_ASSERT(sceneId < state_->scenes.size(),
                 "Renderer::clear(): index" << sceneId << "out of range for"
                                            << state_->scenes.size()
                                            << "scenes", );

  Scene& scene = state_->scenes[sceneId];
  // TODO have arrayClear()!!!
  /* Resizing instead of `= {}` to not discard the memory */
  arrayResize(scene.lights, 0);

  /* Not marking the dirty bit as lights are processed with updated
     transformations every frame anyway */
}

Magnum::Matrix4 Renderer::camera(Magnum::UnsignedInt sceneId) const {
  CORRADE_ASSERT(sceneId < state_->scenes.size(),
                 "Renderer::camera(): index" << sceneId << "out of range for"
                                             << state_->scenes.size()
                                             << "scenes",
                 {});

  return state_->cameraMatrices[sceneId].projectionMatrix;
}

Magnum::Vector2 Renderer::cameraDepthUnprojection(
    Magnum::UnsignedInt sceneId) const {
  CORRADE_ASSERT(sceneId < state_->scenes.size(),
                 "Renderer::cameraDepthUnprojection(): index"
                     << sceneId << "out of range for" << state_->scenes.size()
                     << "scenes",
                 {});

  return state_->scenes[sceneId].cameraUnprojection;
}

void Renderer::updateCamera(Magnum::UnsignedInt sceneId,
                            const Magnum::Matrix4& projection,
                            const Magnum::Matrix4& view) {
  CORRADE_ASSERT(sceneId < state_->scenes.size(),
                 "Renderer::updateCamera(): index"
                     << sceneId << "out of range for" << state_->scenes.size()
                     << "scenes", );

  state_->cameraMatrices[sceneId].projectionMatrix = projection * view;
  state_->scenes[sceneId].cameraUnprojection =
      calculateDepthUnprojection(projection);
}

Cr::Containers::StridedArrayView1D<Mn::Matrix4> Renderer::transformations(
    const Mn::UnsignedInt sceneId) {
  CORRADE_ASSERT(sceneId < state_->scenes.size(),
                 "Renderer::transformations(): index"
                     << sceneId << "out of range for" << state_->scenes.size()
                     << "scenes",
                 {});

  return state_->scenes[sceneId].transformations;
}

Cr::Containers::StridedArrayView1D<Mn::Color3> Renderer::lightColors(
    const Mn::UnsignedInt sceneId) {
  CORRADE_ASSERT(sceneId < state_->scenes.size(),
                 "Renderer::lightColors(): index"
                     << sceneId << "out of range for" << state_->scenes.size()
                     << "scenes",
                 {});

  return stridedArrayView(state_->scenes[sceneId].lights).slice(&Light::color);
}

Cr::Containers::StridedArrayView1D<Mn::Float> Renderer::lightRanges(
    const Mn::UnsignedInt sceneId) {
  CORRADE_ASSERT(sceneId < state_->scenes.size(),
                 "Renderer::lightRanges(): index"
                     << sceneId << "out of range for" << state_->scenes.size()
                     << "scenes",
                 {});

  return stridedArrayView(state_->scenes[sceneId].lights).slice(&Light::range);
}

void Renderer::draw(Mn::GL::AbstractFramebuffer& framebuffer) {
  /* If addFile() was not called, we don't have the shaders set up yet. In that
     case there should be no meshes to render from either, so nothing to do. */
  if (state_->shaders.empty()) {
    CORRADE_INTERNAL_ASSERT(state_->meshes.isEmpty());
    return;
  }

  /* Process scenes that are marked as dirty */
  // TODO this could be a separate step to allow the user to control when it
  //  runs
  {
    // TODO have a dedicated bump allocator for this to avoid allocating on
    //  every dirty processing
    Cr::Containers::Array<Mn::Shaders::TextureTransformationUniform>
        textureTransformationsSorted;
    for (std::size_t sceneId = 0; sceneId != state_->scenes.size(); ++sceneId) {
      Scene& scene = state_->scenes[sceneId];
      if (!scene.dirty)
        continue;

      /* Resize temp arrays if too small */
      // TODO make this unconditional to catch OOB access?
      if (textureTransformationsSorted.size() < scene.draws.size()) {
        arrayResize(textureTransformationsSorted, Cr::NoInit,
                    scene.draws.size());
      }

      /* Draw batch offsets with two implicit 0s at the begin. Reset all values
         to 0 in case the array wasn't empty before. */
      // TODO Utility::fill() instead
      arrayResize(scene.drawBatchOffsets, Cr::NoInit, 0);
      arrayResize(scene.drawBatchOffsets, Cr::ValueInit,
                  scene.drawBatches.size() + 2);

      // TODO make a counting sort utility in corrade? i'm reusing the temp
      //  offset array here in a quite specific way tho
      /* Count how many uses of each draw batch are there; meshOffsets[0] and
         [1] stays 0 */
      for (Mn::UnsignedInt i : scene.drawBatchIds)
        ++scene.drawBatchOffsets[i + 2];

      /* Convert that to an offset array, again meshOffsets[0] and [1] stays
         0 */
      std::size_t offset = 0;
      for (Mn::UnsignedInt& i : scene.drawBatchOffsets) {
        const std::size_t count = i;
        i += offset;
        offset += count;
      }
      CORRADE_INTERNAL_ASSERT(scene.drawBatchOffsets.front() == 0);
      CORRADE_INTERNAL_ASSERT(scene.drawBatchOffsets.back() ==
                              scene.draws.size());

      /* Reorder the draw and texture transformation arrays based on mesh IDs,
         now just meshOffsets[0] stays 0 */
      for (std::size_t i = 0; i != scene.draws.size(); ++i) {
        Mn::UnsignedInt& offset =
            scene.drawBatchOffsets[scene.drawBatchIds[i] + 1];

        scene.drawsSorted[offset] = scene.draws[i];
        textureTransformationsSorted[offset] = scene.textureTransformations[i];
        scene.transformationIdsSorted[offset] = scene.transformationIds[i];
        scene.drawCommandsSorted[offset] = scene.drawCommands[i];
        ++offset;
      }
      CORRADE_INTERNAL_ASSERT(scene.drawBatchOffsets.front() == 0);
      CORRADE_INTERNAL_ASSERT(scene.drawBatchOffsets.back() ==
                              scene.draws.size());

      /* Upload the (temporary) sorted data to uniforms */
      scene.textureTransformationUniform.setData(textureTransformationsSorted);
      scene.dirty = false;
    }
  }

  /* Upload projection uniform, assuming it changes every frame. Do it early to
     minimize stalls. */
  state_->projectionUniform.setData(state_->cameraMatrices);

  /* Calculate absolute transformations */
  for (std::size_t sceneId = 0; sceneId != state_->scenes.size(); ++sceneId) {
    Scene& scene = state_->scenes[sceneId];

    /* The first slot holds the root transformation for easier dealing with
       `parent == -1`. Resize if it's too small. */
    if (state_->absoluteTransformations.size() <
        scene.transformations.size() + 1)
      arrayResize(state_->absoluteTransformations, Cr::NoInit,
                  scene.transformations.size() + 1);

    // TODO have a tool for this! AVX512!!
    state_->absoluteTransformations[0].setTransformationMatrix(Mn::Matrix4{});
    for (std::size_t i = 0; i != scene.transformations.size(); ++i)
      state_->absoluteTransformations[i + 1].setTransformationMatrix(
          state_->absoluteTransformations[scene.parents[i] + 1]
              .transformationMatrix *
          scene.transformations[i]);

    /* Copy transformations referenced by actual draws. Resize if destination
       is too small. */
    if (state_->absoluteTransformationsSorted.size() <
        scene.transformationIdsSorted.size())
      arrayResize(state_->absoluteTransformationsSorted, Cr::NoInit,
                  scene.transformationIdsSorted.size());
    // TODO the casting situation is GETTING OUT OF HAND
    Mn::MeshTools::duplicateInto(
        Cr::Containers::StridedArrayView1D<const Mn::UnsignedInt>{
            scene.transformationIdsSorted},
        Cr::Containers::StridedArrayView1D<
            const Mn::Shaders::TransformationUniform3D>{
            state_->absoluteTransformations.exceptPrefix(1)},
        stridedArrayView(state_->absoluteTransformationsSorted.prefix(
            scene.transformationIdsSorted.size())));

    /* Upload the transformation uniforms, as they get overwritten in the
       next loop. OTOH, interleaving it with the calculation could hide the
       driver stalls. */
    // TODO have this somehow in a single huge buffer instead so we can upload
    // everything at once? ... but that would need the insane alignment
    // requirements, not great either :(
    scene.transformationUniform.setData(
        state_->absoluteTransformationsSorted.prefix(
            scene.transformationIdsSorted.size()));

    /* Finish transformation-dependent per-draw info, upload it */
    for (std::size_t i = 0; i != scene.transformationIds.size(); ++i) {
      scene
          .drawsSorted[i]
          /* Extract normal matrix */
          .setNormalMatrix(state_->absoluteTransformationsSorted[i]
                               .transformationMatrix.normalMatrix())
          // TODO light culling should happen here
          .setLightOffsetCount(0, scene.lights.size());
    }
    // TODO have a single buffer for this
    scene.drawUniform.setData(scene.drawsSorted);

    /* Copy light properties and cherry-pick transformations for them. Resize
       the temp destination if it's too small. */
    if (state_->absoluteLights.size() < scene.lights.size())
      arrayResize(state_->absoluteLights, Cr::NoInit, scene.lights.size());
    for (std::size_t i = 0; i != scene.lights.size(); ++i) {
      const Light& light = scene.lights[i];
      state_->absoluteLights[i]
          .setColor(light.color)
          .setSpecularColor(light.color)
          .setRange(light.range);
      if (light.type == RendererLightType::Directional)
        state_->absoluteLights[i].setPosition(
            Mn::Vector4{-state_->absoluteTransformations[light.node + 1]
                             .transformationMatrix.backward(),
                        0.0f});
      else if (light.type == RendererLightType::Point)
        state_->absoluteLights[i].setPosition(
            Mn::Vector4{state_->absoluteTransformations[light.node + 1]
                            .transformationMatrix.translation(),
                        1.0f});
      else
        CORRADE_INTERNAL_ASSERT_UNREACHABLE(); /* LCOV_EXCL_LINE */
    }

    /* Upload the light uniforms, as they get overwritten in the next loop. */
    // TODO again, have a single buffer for this
    if (!scene.lights.isEmpty())
      scene.lightUniform.setData(
          state_->absoluteLights.prefix(scene.lights.size()));
  }

  /* Remember the original viewport to set it back to where it was after.
     Important if we're not the only code that renders to it, such as when
     rendering directly to a GUI application framebuffer and the application
     wants to draw HUD etc. on top. */
  const Mn::Range2Di previousViewport = framebuffer.viewport();

  for (Mn::Int y = 0; y != state_->tileCount.y(); ++y) {
    for (Mn::Int x = 0; x != state_->tileCount.x(); ++x) {
      framebuffer.setViewport(Mn::Range2Di::fromSize(
          Mn::Vector2i{x, y} * state_->tileSize, state_->tileSize));

      const std::size_t sceneId = y * state_->tileCount.x() + x;
      Scene& scene = state_->scenes[sceneId];

      /* Bind buffers. Again, all shaders share the same binding points so it
         doesn't matter which one is used. */
      // TODO split by draw count limit? hard to do with those batches now, heh
      //  also hard to do due to the insane alignment rules
      state_->shaders.begin()
          ->second
          // TODO bind all buffers together with a multi API
          .bindProjectionBuffer(state_->projectionUniform,
                                sceneId * sizeof(ProjectionPadded),
                                sizeof(ProjectionPadded))
          .bindTransformationBuffer(scene.transformationUniform)
          .bindLightBuffer(scene.lightUniform)
          .bindDrawBuffer(scene.drawUniform);
      if (!(state_->flags & RendererFlag::NoTextures))
        state_->shaders.begin()->second.bindTextureTransformationBuffer(
            scene.textureTransformationUniform);

      /* Submit all draw batches */
      for (std::size_t i = 0; i != scene.drawBatches.size(); ++i) {
        const DrawBatch& drawBatch = scene.drawBatches[i];

        if (!(state_->flags >= RendererFlag::NoTextures)) {
          drawBatch.shader->bindAmbientTexture(
              state_->textures[drawBatch.textureId]);
          if (state_->maxLightCount)
            drawBatch.shader->bindDiffuseTexture(
                state_->textures[drawBatch.textureId]);
        }

        const Mn::UnsignedInt drawBatchOffset = scene.drawBatchOffsets[i];
        const Mn::UnsignedInt nextDrawBatchOffset =
            scene.drawBatchOffsets[i + 1];
        const Cr::Containers::StridedArrayView1D<DrawCommand>
            drawBatchCommands =
                // TODO if unsorted scene.drawCommands is here, the unit test
                //  still passes -- fix!
            scene.drawCommandsSorted.slice(drawBatchOffset,
                                           nextDrawBatchOffset);

        drawBatch.shader->setDrawOffset(drawBatchOffset)
            .draw(state_->meshes[drawBatch.meshId].second(),
                  drawBatchCommands.slice(&DrawCommand::indexCount), nullptr,
                  drawBatchCommands.slice(&DrawCommand::indexOffsetInBytes));
      }
    }
  }

  framebuffer.setViewport(previousViewport);
}

SceneStats Renderer::sceneStats(Mn::UnsignedInt sceneId) const {
  CORRADE_ASSERT(sceneId < state_->scenes.size(),
                 "Renderer::sceneStats(): index"
                     << sceneId << "out of range for" << state_->scenes.size()
                     << "scenes",
                 {});

  const Scene& scene = state_->scenes[sceneId];

  SceneStats out;
  out.nodeCount = scene.transformations.size();
  out.drawCount = scene.draws.size();
  /* This one will be up-to-date only after calling draw(). Another option
     would be to explicitly perform a cleanup in case the scene is dirty, but
     considering there will be more stats like number of culled draws -- which
     again would be up-to-date only after draw() -- people should just learn to
     only fetch stats after a draw, and not before. */
  out.drawBatchCount = scene.drawBatches.size();
  return out;
}

}  // namespace gfx_batch
}  // namespace esp
