// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <unordered_map>
#include <Corrade/Containers/BitArray.h>
#include <Corrade/Containers/BitArrayView.h>
#include <Corrade/Containers/GrowableArray.h>
#include <Corrade/Containers/Iterable.h>
#include <Corrade/Containers/Pair.h>
#include <Corrade/Containers/Triple.h>
#include <Corrade/Containers/StringStlHash.h>
#include <Corrade/Utility/Arguments.h>
#include <Corrade/Utility/Configuration.h>
#include <Corrade/Utility/DebugStl.h>
#include <Corrade/Utility/Format.h>
#include <Corrade/Utility/Path.h>
#include <Corrade/Utility/Sha1.h>
#include <Magnum/ImageView.h>
#include <Magnum/Mesh.h>
#include <Magnum/PixelFormat.h>
#include <Magnum/Math/ConfigurationValue.h>
#include <Magnum/Math/Matrix3.h>
#include <Magnum/MaterialTools/Filter.h>
#include <Magnum/MaterialTools/Merge.h>
#include <Magnum/MeshTools/Duplicate.h>
#include <Magnum/MeshTools/RemoveDuplicates.h>
#include <Magnum/SceneTools/FlattenMeshHierarchy.h>
#include <Magnum/Trade/ImageData.h>
#include <Magnum/Trade/MaterialData.h>
#include <Magnum/Trade/MeshData.h>
#include <Magnum/Trade/SceneData.h>
#include <Magnum/Trade/TextureData.h>

#include "CompositorState.h"

namespace Cr = Corrade;
namespace Mn = Magnum;
using namespace Cr::Containers::Literals;
using namespace Mn::Math::Literals;

namespace {

Cr::Utility::Sha1::Digest imageChecksum(const Mn::Trade::ImageData2D& image) {
  union Metadata {
    explicit Metadata() {}
    struct {
      Mn::Vector2i size;
      Mn::PixelFormat format;
    } data;
    char bytes[12];
  } metadata;
  metadata.data.size = image.size();
  metadata.data.format = image.format();

  Cr::Utility::Sha1 hasher;
  hasher << Cr::Containers::ArrayView<const char>{metadata.bytes};

  for(Cr::Containers::StridedArrayView2D<const char> row: image.pixels())
    hasher << row.asContiguous();

  return hasher.digest();
}

}

int main(int argc, char** argv) {
  Cr::Utility::Arguments args;
  args.addArgument("input").setHelp("input", "input file prefix")
    .addArgument("output").setHelp("output", "output file")
    .parse(argc, argv);

  // TODO make higher once we don't need repeat for all
  constexpr Mn::Vector2i TextureAtlasSize{128};
  constexpr Mn::Int ImageLayerCountLimit = 2048;
  esp::CompositorState s{args.value("output")};
  esp::CompositorSceneState ss;
  esp::CompositorDataState ds{TextureAtlasSize};
  Cr::Containers::Pointer<Mn::Trade::AbstractImporter> importer = s.importerManager.loadAndInstantiate("GltfImporter");
  CORRADE_INTERNAL_ASSERT(importer);

  Cr::Containers::Pointer<Mn::Trade::AbstractImageConverter> imageResizer = s.imageConverterManager.loadAndInstantiate("StbResizeImageConverter");
  CORRADE_INTERNAL_ASSERT(imageResizer);
  // TODO some images need a repeat, so (up)scaling them all to 256x256 to make
  //  them render properly
  // TODO split itno two textures, one a larger atlas with non-repeated and one
  //  a smaller with repeated
  imageResizer->configuration().setValue("size", TextureAtlasSize);
  // TODO have a processing step to figure out which images need repeat instead
  //  of upsampling all
  imageResizer->configuration().setValue("upsample", true);

  Cr::Containers::Pointer<Mn::Trade::AbstractImageConverter> imageCompressor = s.imageConverterManager.loadAndInstantiate("StbDxtImageConverter");
  CORRADE_INTERNAL_ASSERT(imageCompressor);
  imageCompressor->configuration().setValue("highQuality", true);

  Cr::Containers::Pointer<Mn::Trade::AbstractSceneConverter> meshOptimizer = s.converterManager.loadAndInstantiate("MeshOptimizerSceneConverter");
  CORRADE_INTERNAL_ASSERT(meshOptimizer);
  /* Sloppy optimization is enabled only after all arch files get processed,
     for those it'd cause a mess */
  meshOptimizer->configuration().setValue("simplify", true);
  meshOptimizer->configuration().setValue("simplifyTargetError", 1.0e-1);

  // constexpr Mn::Float MeshDensityThreshold = 42*42*4;
  constexpr Mn::Float MeshDensityThreshold = 1000;

  // TODO use something else than a std::string to represent digests
  std::unordered_map<std::string, Mn::UnsignedInt> uniqueImages;

  Mn::UnsignedInt inputImageCount = 0;

  const auto import = [&](
    Cr::Containers::StringView filename,
    Cr::Containers::StringView name) {
    CORRADE_INTERNAL_ASSERT_OUTPUT(importer->openFile(filename));
    CORRADE_INTERNAL_ASSERT(importer->sceneCount() == 1);

    Mn::Debug{} << "Importing" << Cr::Utility::Path::split(filename).second() << "with" << importer->meshCount() << "meshes and" << importer->image2DCount() << "images";

    Cr::Containers::Optional<Mn::Trade::SceneData> scene = importer->scene(0);
    CORRADE_INTERNAL_ASSERT(scene);

    /* Top-level object, parent of the others */
    esp::Parent root;
    root.mapping = ss.parents.size();
    root.parent = -1;
    arrayAppend(ss.parents, root);
    s.converter->setObjectName(root.mapping, name);

    /* Assuming materials are shared among meshes, remember the ID of already
       imported materials */
    Cr::Containers::Array<Cr::Containers::Optional<Mn::UnsignedInt>> importedMaterialIds{importer->materialCount()};

    Cr::Containers::Array<Cr::Containers::Optional<Cr::Containers::Pair<Mn::UnsignedInt, Mn::UnsignedInt>>> importedMeshes{Cr::ValueInit, importer->meshCount()};

    /* Node mesh/material assignments. Each entry will be one child of the
       top-level object. */
    for(Cr::Containers::Triple<Mn::UnsignedInt, Mn::Int, Mn::Matrix4> transformationMeshMaterial: Mn::SceneTools::flattenMeshHierarchy3D(*scene)) {
      // TODO drop the names?
      Cr::Containers::String meshName = importer->meshName(transformationMeshMaterial.first());

      /* Process the mesh, if not imported yet */
      if(!importedMeshes[transformationMeshMaterial.first()]) {
        Cr::Containers::Optional<Mn::Trade::MeshData> mesh = importer->mesh(transformationMeshMaterial.first());
        CORRADE_INTERNAL_ASSERT(mesh);
        /* Skip non-triangle meshes */
        if(mesh->primitive() != Mn::MeshPrimitive::Triangles &&
          mesh->primitive() != Mn::MeshPrimitive::TriangleFan &&
          mesh->primitive() != Mn::MeshPrimitive::TriangleStrip) {
          Mn::Warning{} << "Mesh" << meshName << "in" << Cr::Utility::Path::split(filename).second() << "is" << mesh->primitive() << Mn::Debug::nospace << ", skipping";
          continue;
        }

        /* Needed by some furniture */
        if(!mesh->isIndexed())
          mesh = Mn::MeshTools::removeDuplicates(*mesh);
        CORRADE_INTERNAL_ASSERT(mesh->primitive() == Mn::MeshPrimitive::Triangles);

        /* Calculate total triangle area / count ratio and simplify if it's too
          much */
        {
          // TODO do w/o allocation if possible?
          const Cr::Containers::Array<Mn::UnsignedInt> indices = mesh->indicesAsArray();
          const Cr::Containers::Array<Mn::Vector3> positions = mesh->positions3DAsArray();
          const std::size_t triangleCount = indices.size()/3;
          Mn::Float totalArea = 0.0f;
          for(std::size_t i = 0; i != triangleCount; ++i) {
            const Mn::Vector3 a = positions[indices[i*3 + 0]];
            const Mn::Vector3 b = positions[indices[i*3 + 1]];
            const Mn::Vector3 c = positions[indices[i*3 + 2]];

            /* Triangle area is half of the cross product of its two vectors */
            totalArea += Mn::Math::cross(b - a, c - a).length()*0.5f;
          }

          const Mn::Float target = MeshDensityThreshold/(triangleCount/totalArea);
          if(target < 1.0f) {
            meshOptimizer->configuration().setValue("simplifyTargetIndexCountThreshold", target);
            Mn::UnsignedInt vertexCount = mesh->vertexCount();
            mesh = meshOptimizer->convert(*mesh);
            Mn::Debug{} << "Decimated mesh" << transformationMeshMaterial.first() << "to" << Cr::Utility::format("{:.1f}%", mesh->vertexCount()*100.0f/vertexCount);
          }
          // TODO if >= 1, do at least an optimization (as a subsequent composite
          //  version, so it can be seen how it improves the overall perf)
        }

        importedMeshes[transformationMeshMaterial.first()] = Cr::Containers::pair(ds.indexOffset, mesh->indexCount());

        ds.indexOffset += mesh->indexCount()*4; // TODO ints hardcoded

        arrayAppend(ds.inputMeshes, *std::move(mesh));
      }

      esp::Parent o;
      o.mapping = ss.parents.size();
      o.parent = root.mapping;
      arrayAppend(ss.parents, o);
      arrayAppend(ss.transformations, Cr::InPlaceInit,
        o.mapping,
        transformationMeshMaterial.third());
      /* Save the nested object name as well, for debugging purposes. It'll be
         duplicated among different scenes but that's no problem. */
      // TODO not doing this, too much noise that's obscuring errors in the top-level names
      // s.converter->setObjectName(o.mapping, meshName);

      esp::Mesh m;
      m.mapping = o.mapping;
      m.mesh = 0;
      m.meshIndexOffset = importedMeshes[transformationMeshMaterial.first()]->first();
      m.meshIndexCount = importedMeshes[transformationMeshMaterial.first()]->second();

      /* If the material is already parsed, reuse its ID */
      if(const Cr::Containers::Optional<Mn::UnsignedInt> materialId = importedMaterialIds[transformationMeshMaterial.second()]) {
        m.meshMaterial = *materialId;

      /* Otherwise parse it. If textured, extract the image as well. */
      } else {
        Cr::Containers::Optional<Mn::Trade::MaterialData> material = importer->material(transformationMeshMaterial.second());
        CORRADE_INTERNAL_ASSERT(material);

        // TODO for filtering, add everything there
        // Cr::Containers::BitArray attributesToKeep{Cr::DirectInit, material->attributeData().size(), true};

        /* Not calling releaseAttributeData() yet either, as we need to ask
           hasAttribute() first TODO drop, merge() */
        Cr::Containers::Array<Mn::Trade::MaterialAttributeData> attributes;
        if(const Cr::Containers::Optional<Mn::UnsignedInt> baseColorTextureAttributeId = material->findAttributeId(Mn::Trade::MaterialAttribute::BaseColorTexture)) {
          // Mn::Debug{} << "New textured material for" << meshName << "in" << Cr::Utility::Path::split(filename).second();

          Mn::UnsignedInt& textureId = material->mutableAttribute<Mn::UnsignedInt>(*baseColorTextureAttributeId);

          Cr::Containers::Optional<Mn::Trade::TextureData> texture = importer->texture(textureId);
          CORRADE_INTERNAL_ASSERT(texture && texture->type() == Mn::Trade::TextureType::Texture2D);

          ++inputImageCount;
          Cr::Containers::Optional<Mn::Trade::ImageData2D> image = importer->image2D(texture->image());
          CORRADE_INTERNAL_ASSERT(image);

          /* Check if we already have the same image contents */
          std::string sha1 = imageChecksum(*image).hexString();

          auto found = uniqueImages.find(sha1);
          if(found == uniqueImages.end()) {
            image = imageResizer->convert(*image);
            CORRADE_INTERNAL_ASSERT(image);

            found = uniqueImages.emplace(sha1, ds.inputImages.size()).first;
            arrayAppend(ds.inputImages, *std::move(image));
          } else {
            Mn::Debug{} << "Reusing existing image #" << Mn::Debug::nospace << found->second << sha1;
          }

          const Mn::UnsignedInt uniqueImageId = found->second;
          const Mn::Trade::ImageData2D& uniqueImage = ds.inputImages[found->second];

          /* Resize the image if it's too big */

          // CORRADE_ASSERT((image->size() & (image->size() - Mn::Vector2i{1})).isZero(),
          //   "Image not power-of-two:" << image->size(), );

          // TODO detection of single-color images here?

          /* Add texture scaling if the image is smaller */
          if((uniqueImage.size() < TextureAtlasSize).any()) {
            // TODO this shouldn't be used right now
            CORRADE_ASSERT_UNREACHABLE("This shouldn't be used right now, all images have the same size", );

            const Mn::Matrix3 matrix = Mn::Matrix3::scaling(Mn::Vector2(uniqueImage.size())/Mn::Vector2(TextureAtlasSize));
            // TODO MaterialTools::merge() instead!!!
            if(const Cr::Containers::Optional<Mn::UnsignedInt> baseColorTextureMatrixAttributeId = material->findAttributeId(Mn::Trade::MaterialAttribute::BaseColorTextureMatrix)) {
              Mn::Matrix3& existingMatrix =
              material->mutableAttribute<Mn::Matrix3>(*baseColorTextureMatrixAttributeId);
              existingMatrix = matrix*existingMatrix;
            } else {
              arrayAppend(attributes, Mn::InPlaceInit, Mn::Trade::MaterialAttribute::BaseColorTextureMatrix, matrix);
            }
          }

          /* Patch the material to use the zero texture but add a layer as
             well, referencing the just-added image */
          textureId = 0;
          arrayAppend(attributes, Cr::InPlaceInit, Mn::Trade::MaterialAttribute::BaseColorTextureLayer, uniqueImageId);

        #if 1
        /* Otherwise make it reference the first image, which is all plain
           white */
        } else {
          // Mn::Debug{} << "New untextured material for" << meshName << "in" << Cr::Utility::Path::split(filename).second();

          arrayAppend(attributes, {
            Mn::Trade::MaterialAttributeData{Mn::Trade::MaterialAttribute::BaseColorTexture, 0u},
            Mn::Trade::MaterialAttributeData{Mn::Trade::MaterialAttribute::BaseColorTextureLayer, 0u},
            // Mn::Trade::MaterialAttributeData{Mn::Trade::MaterialAttribute::BaseColorTextureMatrix,
            //   Mn::Matrix3::scaling(Mn::Vector2{0.0f})}
          });
        }
        #else
        /* Otherwise add a 1x1 image containing the base color and make the
           material use it as a texture instead. Set the texture matrix scaling
           to zero to make it work for repeated textures as well. */
        } else {
        //   // Mn::Debug{} << "New untextured material for" << meshName << "in" << Cr::Utility::Path::split(filename).second() << ;

          // TODO this loses alpha
          Mn::Trade::ImageData2D image{Mn::PixelFormat::RGB8Unorm, {1, 1}, Cr::Containers::Array<char>{Cr::NoInit, 4}};
          if(const Cr::Containers::Optional<Mn::UnsignedInt> baseColorId = material->findAttributeId(Mn::Trade::MaterialAttribute::BaseColor)) {
            image.mutablePixels<Mn::Color3ub>()[0][0] = Mn::Math::pack<Mn::Color3ub>(material->attribute<Mn::Color4>(*baseColorId).rgb());
            attributesToKeep.reset(*baseColorId);
          } else
            image.mutablePixels<Mn::Color3ub>()[0][0] = 0xffffff_rgb;

          // TODO deduplicate those similarly to above
          const Mn::UnsignedInt uniqueImageId = ds.inputImages.size();
          arrayAppend(ds.inputImages, std::move(image));
          arrayAppend(attributes, {
            {Mn::Trade::MaterialAttribute::BaseColorTexture, 0u},
            {Mn::Trade::MaterialAttribute::BaseColorTextureLayer, uniqueImageId},
            {Mn::Trade::MaterialAttribute::BaseColorTextureMatrix, Mn::Matrix3::scaling(Mn::Vector2{0.0f})}
          });

        }

        /* And filter away the base color attribute */
        // TODO and others
        material = Mn::MaterialTools::filterAttributes(*material, attributesToKeep);
        #endif

        // TODO MaterialTools::merge() instead of this!!!!
        // CORRADE_INTERNAL_ASSERT(material->layerCount() == 1); // TODO is broken for furniture!!
        // arrayAppend(attributes, material->attributeData().prefix(material->attributeCount(0)));

        material = Mn::MaterialTools::merge(*material, Mn::Trade::MaterialData{{}, Mn::Trade::DataFlags{}, attributes});

        // material = Mn::Trade::MaterialData{material->types(), std::move(attributes)};

        importedMaterialIds[transformationMeshMaterial.second()] = m.meshMaterial = ds.inputMaterials.size();
        arrayAppend(ds.inputMaterials, *std::move(material));
      }

      arrayAppend(ss.meshes, m);
    }
  };

  #if 1
  // TODO use the "really no doors" instead
  Cr::Containers::String archPath = Cr::Utility::Path::join(args.value("input"), "glb-arch-only");
  Cr::Containers::Optional<Cr::Containers::Array<Cr::Containers::String>> archFiles = Cr::Utility::Path::list(archPath, Cr::Utility::Path::ListFlag::SkipDirectories|Cr::Utility::Path::ListFlag::SortAscending);
  for(Cr::Containers::String name: (*archFiles)) {//->prefix(10)) { // TODO remove the limit
    CORRADE_INTERNAL_ASSERT(name.hasSuffix(".glb"_s));
    // TODO what's the path prefix used by Habitat / gfx-replay?
    import(Cr::Utility::Path::join(archPath, name), "../stages/"_s + name);
  }
  #elif 0
  // TODO this was used back when SSBOs weren't a thing
  /* https://cvmlp.slack.com/archives/C0460NTKM4G/p1675443726846979?thread_ts=1675358050.326359&cid=C0460NTKM4G */
  for(const char* name: { // TODO use all instead
    "103997478_171030525.glb",
    "108736611_177263226.glb",
    "105515175_173104107.glb",
    "107734080_175999881.glb",
    "103997562_171030642.glb",
    "108294624_176710203.glb",
    "108294816_176710461.glb",
    "108736722_177263382.glb",
    "108294465_176709960.glb",
    "106879044_174887172.glb"
  }) {
    // TODO what's the path prefix used by Habitat / gfx-replay?
    import(Cr::Utility::Path::join(archPath, name), "../stages/"_s + name);
  }
  #endif

  /* Enabled for furniture only, for the arch files it makes a mess */
  meshOptimizer->configuration().setValue("simplifySloppy", true);

  #if 1
  // TODO use the original below once we have time to wait for slow resizes
  #if 1
  Cr::Containers::String furniturePath = Cr::Utility::Path::join(args.value("input"), "object-noq-tex256");
  Cr::Containers::Optional<Cr::Containers::Array<Cr::Containers::String>> furnitureFiles = Cr::Utility::Path::list(furniturePath, Cr::Utility::Path::ListFlag::SkipDirectories|Cr::Utility::Path::ListFlag::SortAscending);
  for(Cr::Containers::StringView name: *furnitureFiles) {
    CORRADE_INTERNAL_ASSERT(name.hasSuffix(".glb"_s));
    /* This one contains also the openings, skip them */
    if(name.size() < 40)
      continue;
    // TODO what's the path prefix used by Habitat / gfx-replay?
    import(Cr::Utility::Path::join(furniturePath, name),
      Cr::Utility::format("../objects/{}/{}.glb", name.prefix(1), name.prefix(40)));
  }
  #elif 0
  // Cr::Containers::String furniturePath = Cr::Utility::Path::join(args.value("input"), "furniture-original");
  Cr::Containers::Optional<Cr::Containers::Array<Cr::Containers::String>> furnitureFiles = Cr::Utility::Path::list(furniturePath, Cr::Utility::Path::ListFlag::SkipDirectories|Cr::Utility::Path::ListFlag::SortAscending);
  for(Cr::Containers::StringView name: *furnitureFiles) {
    CORRADE_INTERNAL_ASSERT(name.hasSuffix(".glb"_s));
    // TODO what's the path prefix used by Habitat / gfx-replay?
    import(Cr::Utility::Path::join(furniturePath, name),
      Cr::Utility::format("../objects/{}/{}", name.prefix(1), name));
  }
  #endif

  // TODO these generate an awful lot of data because most of them contain the
  //  meshes an insane amount of times, such as
  //  2a6fa282275e6ddb1fc35ebdc6a9ef91ae93a8ae_part_16.glb containing 934
  //  meshes for a plant that should have just SEVEN
  #if 0
  //  duplicated for no reason
  Cr::Containers::String furnitureDecomposedPath = Cr::Utility::Path::join(args.value("input"), "decomposed"); // TODO ugh
  Cr::Containers::Optional<Cr::Containers::Array<Cr::Containers::String>> furnitureDecomposedDirs = Cr::Utility::Path::list(furnitureDecomposedPath, Cr::Utility::Path::ListFlag::SkipDotAndDotDot|Cr::Utility::Path::ListFlag::SkipFiles|Cr::Utility::Path::ListFlag::SortAscending);
  {
    for(Cr::Containers::StringView dir: *furnitureDecomposedDirs) {
      Cr::Containers::Optional<Cr::Containers::Array<Cr::Containers::String>> furnitureDecomposedFiles = Cr::Utility::Path::list(Cr::Utility::Path::join(furnitureDecomposedPath, dir), Cr::Utility::Path::ListFlag::SkipDirectories|Cr::Utility::Path::ListFlag::SortAscending);
      for(Cr::Containers::StringView name: *furnitureDecomposedFiles) {
        if(name.contains("dedup"))
          continue;
        if(!name.hasSuffix(".glb"_s)) // TODO UGH
          continue;
        // TODO what's the path prefix used by Habitat / gfx-replay?
        import(Cr::Utility::Path::join({furnitureDecomposedPath, dir, name}),
            Cr::Utility::format("../objects/decomposed/{}/{}", dir, name));
      }
    }
  }
  #endif

  #if 1
  Cr::Containers::String furnitureOpeningsPath = Cr::Utility::Path::join(args.value("input"), "openings-variants"); // TODO ugh
  Cr::Containers::Optional<Cr::Containers::Array<Cr::Containers::String>> furnitureOpeningsFiles = Cr::Utility::Path::list(furnitureOpeningsPath, Cr::Utility::Path::ListFlag::SkipDirectories|Cr::Utility::Path::ListFlag::SortAscending);
  for(Cr::Containers::StringView name: *furnitureOpeningsFiles) {
    CORRADE_INTERNAL_ASSERT(name.hasSuffix(".glb"_s));
    // TODO what's the path prefix used by Habitat / gfx-replay?
    import(Cr::Utility::Path::join(furnitureOpeningsPath, name),
      "../objects/openings/"_s + name);
  }
  #endif
  #endif

  Mn::Debug{} << inputImageCount << "images deduplicated to" << uniqueImages.size();

  /* Convert images first so we can free them and have more memory for mesh
     concatenation. Split them into multiple as this goes over the max GPU
     texture array layer count. */
  Mn::Trade::ImageData3D image = ds.finalizeImage(ds.inputMaterials, ImageLayerCountLimit);
  ds.inputImages = {};
  for(Mn::Int i = 0, iMax = (image.size().z() + ImageLayerCountLimit - 1)/ImageLayerCountLimit; i != iMax; ++i) {
    Mn::ImageView3D imagePart{
      Mn::PixelStorage{}.setSkip({0, 0, Mn::Int(i*ImageLayerCountLimit)}),
      image.format(),
      {image.size().xy(), Mn::Math::min(image.size().z() - i*ImageLayerCountLimit, ImageLayerCountLimit)},
      image.data(),
      Mn::ImageFlag3D::Array
    };

    /* Compress the image */
    Cr::Containers::Optional<Mn::Trade::ImageData3D> imageCompressed = imageCompressor->convert(imagePart);
    Mn::Debug{} << "Image" << i << "of size" << imagePart.size() << "compressed to" << imageCompressed->compressedFormat();

    CORRADE_INTERNAL_ASSERT(s.converter->add(*imageCompressed));
    // TODO why even have this?!
    // CORRADE_INTERNAL_ASSERT(s.converter->add(ds.finalizeTexture()));
    CORRADE_INTERNAL_ASSERT(s.converter->add( Mn::Trade::TextureData{Mn::Trade::TextureType::Texture2DArray,
      Mn::SamplerFilter::Linear, Mn::SamplerFilter::Linear, Mn::SamplerMipmap::Linear,
      Mn::SamplerWrapping::Repeat, Mn::UnsignedInt(i)}));
  }
  image = Mn::Trade::ImageData3D{Mn::PixelFormat::R8I, {}, nullptr};

  /* Concatenate meshes and free them right after as well */
  // TODO some mesh optimization first?
  CORRADE_INTERNAL_ASSERT(s.converter->add(ds.finalizeMesh()));
  ds.inputMeshes = {};

  /* Filter away unused material attributes */
  for(Mn::Trade::MaterialData& material: ds.inputMaterials) {
    Cr::Containers::BitArray attributesToKeep{Cr::DirectInit, material.attributeData().size(), true};
    // TODO 623
    // for(const char* name: {"fp_class_name", "fp_name"}) {
    //   if(const Cr::Containers::Optional<Mn::UnsignedInt> attributeId = material.findAttributeId(name))
    //     attributesToKeep.reset(*attributeId);
    // }
    // TODO this is just to check if moving the atlas transform to the scene
    //  helped with anything
    // for(Mn::Trade::MaterialAttribute name: {Mn::Trade::MaterialAttribute::BaseColorTextureMatrix, Mn::Trade::MaterialAttribute::BaseColorTextureLayer}) {
    //   if(const Cr::Containers::Optional<Mn::UnsignedInt> attributeId = material.findAttributeId(name))
    //     attributesToKeep.reset(*attributeId);
    // }
    for(Mn::Trade::MaterialAttribute name: {Mn::Trade::MaterialAttribute::NoneRoughnessMetallicTexture,
      Mn::Trade::MaterialAttribute::NormalTexture,
      Mn::Trade::MaterialAttribute::NormalTextureScale,
      Mn::Trade::MaterialAttribute::EmissiveTexture,
      Mn::Trade::MaterialAttribute::OcclusionTexture,
      Mn::Trade::MaterialAttribute::Metalness,
      Mn::Trade::MaterialAttribute::Roughness,
    }) {
      if(const Cr::Containers::Optional<Mn::UnsignedInt> attributeId = material.findAttributeId(name))
        attributesToKeep.reset(*attributeId);
    }
    Cr::Containers::BitArray layersToKeep{Cr::DirectInit, material.layerCount(), false};
    layersToKeep.set(0); // TODO only base layer now
    material = Mn::MaterialTools::filterAttributesLayers(material, attributesToKeep, layersToKeep);
    // material = Mn::MaterialTools::filterAttributes(material, attributesToKeep);
  }

  #if 0
  /* Canonicalize materials */
  std::size_t attributesBefore = 0;
  std::size_t attributesAfter = 0;
  for(Mn::Trade::MaterialData& material: ds.inputMaterials) {
    attributesBefore += material.attributeData().size();
    material = Mn::MaterialTools::canonicalize(material);
    attributesAfter += material.attributeData().size();
  }
  Mn::Debug{} << ds.inputMaterials.size() << "materials canonicalized from" << attributesBefore << "to" << attributesAfter << "attributes in total";

  /* Deduplicate materials, update their indices */
  Cr::Containers::Pair<Cr::Containers::Array<Mn::UnsignedInt>, std::size_t> materialDuplicates =
  Mn::MaterialTools::removeDuplicatesInPlace(ds.inputMaterials);
  for(esp::Mesh& mesh: ss.meshes)
    if(mesh.meshMaterial != -1)
      mesh.meshMaterial = materialDuplicates.first()[mesh.meshMaterial];;
  Mn::Debug{} << ds.inputMaterials.size() << "materials deduplicated to" << materialDuplicates.second();

  for(const Mn::Trade::MaterialData& i: ds.inputMaterials.prefix(materialDuplicates.second()))
    CORRADE_INTERNAL_ASSERT(s.converter->add(i));
  #else
  for(const Mn::Trade::MaterialData& i: ds.inputMaterials)
    CORRADE_INTERNAL_ASSERT(s.converter->add(i));
  #endif

  CORRADE_INTERNAL_ASSERT(s.converter->add(ss.finalizeScene()));

  return s.converter->endFile() ? 0 : 1;
}
