// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <unordered_map>
#include <Corrade/Containers/GrowableArray.h>
#include <Corrade/Containers/Pair.h>
#include <Corrade/Containers/Triple.h>
#include <Corrade/Containers/StringStlHash.h>
#include <Corrade/Utility/Arguments.h>
#include <Corrade/Utility/Configuration.h>
#include <Corrade/Utility/Path.h>
#include <Magnum/Mesh.h>
#include <Magnum/Math/ConfigurationValue.h>
#include <Magnum/Math/Matrix3.h>
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

int main(int argc, char** argv) {
  Cr::Utility::Arguments args;
  args.addArgument("input").setHelp("input", "input file prefix")
    .addArgument("output").setHelp("output", "output file")
    .parse(argc, argv);

  constexpr Mn::Vector2i TextureAtlasSize{512};
  esp::CompositorState s{args.value("output")};
  esp::CompositorSceneState ss;
  esp::CompositorDataState ds{TextureAtlasSize};
  // TODO instead of forcing the importer name, it should fall back to
  //  detection based on file magic if the `.glb.orig` is unrecognizable
  Cr::Containers::Pointer<Mn::Trade::AbstractImporter> importer = s.importerManager.loadAndInstantiate("GltfImporter");
  CORRADE_INTERNAL_ASSERT(importer);

  Cr::Containers::Pointer<Mn::Trade::AbstractImageConverter> imageResizer = s.imageConverterManager.loadAndInstantiate("StbResizeImageConverter");
  CORRADE_INTERNAL_ASSERT(imageResizer);
  imageResizer->configuration().setValue("size", Mn::Vector2i{256}); // TODO even less?
  imageResizer->configuration().setValue("upsample", false);

  std::unordered_map<Cr::Containers::String, Cr::Containers::Pair<Mn::UnsignedInt, Mn::UnsignedInt>> uniqueMeshes;

  const auto import = [&](
    Cr::Containers::StringView filename,
    Cr::Containers::StringView name) {
    CORRADE_INTERNAL_ASSERT_OUTPUT(importer->openFile(filename));
    CORRADE_INTERNAL_ASSERT(importer->sceneCount() == 1);

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

    /* Node mesh/material assignments. Each entry will be one child of the
       top-level object. */
    for(Cr::Containers::Triple<Mn::UnsignedInt, Mn::Int, Mn::Matrix4> transformationMeshMaterial: Mn::SceneTools::flattenMeshHierarchy3D(*scene)) {
      Cr::Containers::Optional<Mn::Trade::MeshData> mesh = importer->mesh(transformationMeshMaterial.first());
      // TODO drop the names, useless
      Cr::Containers::String meshName = importer->meshName(transformationMeshMaterial.first());

      /* Skip non-triangle meshes */
      if(mesh->primitive() != Mn::MeshPrimitive::Triangles &&
         mesh->primitive() != Mn::MeshPrimitive::TriangleFan &&
         mesh->primitive() != Mn::MeshPrimitive::TriangleStrip) {
        Mn::Warning{} << "Mesh" << meshName << "in" << Cr::Utility::Path::split(filename).second() << "is" << mesh->primitive() << Mn::Debug::nospace << ", skipping";
        continue;
      }

      // TODO convert from a strip/... if not triangles
      CORRADE_INTERNAL_ASSERT(mesh && mesh->isIndexed() && mesh->primitive() == Mn::MeshPrimitive::Triangles);

      esp::Parent o;
      o.mapping = ss.parents.size();
      o.parent = root.mapping;
      arrayAppend(ss.parents, o);
      arrayAppend(ss.transformations, Cr::InPlaceInit,
        o.mapping,
        transformationMeshMaterial.third());
      /* Save the nested object name as well, for debugging purposes. It'll be
         duplicated among different scenes but that's no problem. */
      s.converter->setObjectName(o.mapping, meshName);

      esp::Mesh m;
      m.mapping = o.mapping;
      m.mesh = 0;
      m.meshIndexCount = mesh->indexCount();

      Mn::Debug{} << "New mesh" << meshName << "in" << Cr::Utility::Path::split(filename).second();
      uniqueMeshes.insert({std::move(meshName), {ds.indexOffset, mesh->indexCount()}});

      m.meshIndexOffset = ds.indexOffset;
      ds.indexOffset += mesh->indexCount()*4; // TODO ints hardcoded

      // TODO convert from a strip/... if not triangles
      arrayAppend(ds.inputMeshes, *std::move(mesh));

      /* If the material is already parsed, reuse its ID */
      // TODO drop material deduplication, useless
      if(const Cr::Containers::Optional<Mn::UnsignedInt> materialId = importedMaterialIds[transformationMeshMaterial.second()]) {
        m.meshMaterial = *materialId;

      /* Otherwise parse it. If textured, extract the image as well. */
      } else {
        Cr::Containers::Optional<Mn::Trade::MaterialData> material = importer->material(transformationMeshMaterial.second());
        CORRADE_INTERNAL_ASSERT(material);

        /* Not calling releaseAttributeData() yet either, as we need to ask
           hasAttribute() first */
        Cr::Containers::Array<Mn::Trade::MaterialAttributeData> attributes;
        if(const Cr::Containers::Optional<Mn::UnsignedInt> baseColorTextureAttributeId = material->findAttributeId(Mn::Trade::MaterialAttribute::BaseColorTexture)) {
          Mn::Debug{} << "New textured material for" << meshName << "in" << Cr::Utility::Path::split(filename).second();

          Mn::UnsignedInt& textureId = material->mutableAttribute<Mn::UnsignedInt>(*baseColorTextureAttributeId);

          Cr::Containers::Optional<Mn::Trade::TextureData> texture = importer->texture(textureId);
          CORRADE_INTERNAL_ASSERT(texture && texture->type() == Mn::Trade::TextureType::Texture2D);

          // TODO textures referencing the same image ID -> deduplicate;
          Cr::Containers::Optional<Mn::Trade::ImageData2D> image = importer->image2D(texture->image());

          /* Resize the image if it's too big */
          CORRADE_INTERNAL_ASSERT(image);
          image = imageResizer->convert(*image);
          CORRADE_INTERNAL_ASSERT(image);

          // TODO detection of single-color images here?

          /* Add texture scaling if the image is smaller */
          if((image->size() < TextureAtlasSize).any()) {
            const Mn::Matrix3 matrix = Mn::Matrix3::scaling(Mn::Vector2(image->size())/Mn::Vector2(TextureAtlasSize));
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
          arrayAppend(attributes, Cr::InPlaceInit, Mn::Trade::MaterialAttribute::BaseColorTextureLayer, Mn::UnsignedInt(ds.inputImages.size()));

          arrayAppend(ds.inputImages, *std::move(image));

        /* Otherwise make it reference the first image, which is a 1x1 white
           pixel */
        } else {
          // TODO not needed here, clean up
          CORRADE_INTERNAL_ASSERT_UNREACHABLE();
          Mn::Debug{} << "New untextured material for" << meshName << "in" << Cr::Utility::Path::split(filename).second();
          //
          // arrayAppend(attributes, {
          //   Mn::Trade::MaterialAttributeData{Mn::Trade::MaterialAttribute::BaseColorTexture, 0u},
          //   Mn::Trade::MaterialAttributeData{Mn::Trade::MaterialAttribute::BaseColorTextureLayer, 0u},
          //   Mn::Trade::MaterialAttributeData{Mn::Trade::MaterialAttribute::BaseColorTextureMatrix,
          //     Mn::Matrix3::scaling(Mn::Vector2{1}/Mn::Vector2(TextureAtlasSize))}
          // });
        }

        // TODO MaterialTools::merge() instead of this!!!!
        CORRADE_INTERNAL_ASSERT(material->layerCount() == 1);
        arrayAppend(attributes, material->attributeData());

        /* Make it just Flat */
        material = Mn::Trade::MaterialData{Mn::Trade::MaterialType::Flat, std::move(attributes)};

        importedMaterialIds[transformationMeshMaterial.second()] = m.meshMaterial = ds.inputMaterials.size();
        arrayAppend(ds.inputMaterials, *std::move(material));
      }

      arrayAppend(ss.meshes, m);
    }
  };

  Cr::Containers::String ycbPath = Cr::Utility::Path::join(args.value("input"), "hab_ycb_v1.2/meshes");
  for(const char* name: {
    "002_master_chef_can/google_16k/textured.glb",
    "003_cracker_box/google_16k/textured.glb",
    "004_sugar_box/google_16k/textured.glb",
    "005_tomato_soup_can/google_16k/textured.glb",
    "006_mustard_bottle/google_16k/textured.glb",
    "007_tuna_fish_can/google_16k/textured.glb",
    "008_pudding_box/google_16k/textured.glb",
    "009_gelatin_box/google_16k/textured.glb",
    "010_potted_meat_can/google_16k/textured.glb",
    "011_banana/google_16k/textured.glb",
    "012_strawberry/google_16k/textured.glb",
    "013_apple/google_16k/textured.glb",
    "014_lemon/google_16k/textured.glb",
    "015_peach/google_16k/textured.glb",
    "016_pear/google_16k/textured.glb",
    "017_orange/google_16k/textured.glb",
    "018_plum/google_16k/textured.glb",
    "019_pitcher_base/google_16k/textured.glb",
    "021_bleach_cleanser/google_16k/textured.glb",
    "022_windex_bottle/google_16k/textured.glb",
    "024_bowl/google_16k/textured.glb",
    "025_mug/google_16k/textured.glb",
    "026_sponge/google_16k/textured.glb",
    "027_skillet/google_16k/textured.glb",
    "028_skillet_lid/google_16k/textured.glb",
    "029_plate/google_16k/textured.glb",
    "030_fork/google_16k/textured.glb",
    "031_spoon/google_16k/textured.glb",
    "032_knife/google_16k/textured.glb",
    "033_spatula/google_16k/textured.glb",
    "035_power_drill/google_16k/textured.glb",
    "036_wood_block/google_16k/textured.glb",
    "037_scissors/google_16k/textured.glb",
    "038_padlock/google_16k/textured.glb",
    "040_large_marker/google_16k/textured.glb",
    "042_adjustable_wrench/google_16k/textured.glb",
    "043_phillips_screwdriver/google_16k/textured.glb",
    "044_flat_screwdriver/google_16k/textured.glb",
    "048_hammer/google_16k/textured.glb",
    "050_medium_clamp/google_16k/textured.glb",
    "051_large_clamp/google_16k/textured.glb",
    "052_extra_large_clamp/google_16k/textured.glb",
    "053_mini_soccer_ball/google_16k/textured.glb",
    "054_softball/google_16k/textured.glb",
    "055_baseball/google_16k/textured.glb",
    "056_tennis_ball/google_16k/textured.glb",
    "057_racquetball/google_16k/textured.glb",
    "058_golf_ball/google_16k/textured.glb",
    "059_chain/google_16k/textured.glb",
    "061_foam_brick/google_16k/textured.glb",
    "062_dice/google_16k/textured.glb",
    "063-a_marbles/google_16k/textured.glb",
    "063-b_marbles/google_16k/textured.glb",
    "065-a_cups/google_16k/textured.glb",
    "065-b_cups/google_16k/textured.glb",
    "065-c_cups/google_16k/textured.glb",
    "065-d_cups/google_16k/textured.glb",
    "065-e_cups/google_16k/textured.glb",
    "065-f_cups/google_16k/textured.glb",
    "065-g_cups/google_16k/textured.glb",
    "065-h_cups/google_16k/textured.glb",
    "065-i_cups/google_16k/textured.glb",
    "065-j_cups/google_16k/textured.glb",
    "070-a_colored_wood_blocks/google_16k/textured.glb",
    "070-b_colored_wood_blocks/google_16k/textured.glb",
    "071_nine_hole_peg_test/google_16k/textured.glb",
    "072-a_toy_airplane/google_16k/textured.glb",
    "072-b_toy_airplane/google_16k/textured.glb",
    "072-c_toy_airplane/google_16k/textured.glb",
    "072-d_toy_airplane/google_16k/textured.glb",
    "072-e_toy_airplane/google_16k/textured.glb",
    "073-a_lego_duplo/google_16k/textured.glb",
    "073-b_lego_duplo/google_16k/textured.glb",
    "073-c_lego_duplo/google_16k/textured.glb",
    "073-d_lego_duplo/google_16k/textured.glb",
    "073-e_lego_duplo/google_16k/textured.glb",
    "073-f_lego_duplo/google_16k/textured.glb",
    "073-g_lego_duplo/google_16k/textured.glb",
    "077_rubiks_cube/google_16k/textured.glb"
  })
    import(Cr::Utility::Path::join(ycbPath, name + ".orig"_s), "data/objects/ycb/configs/../meshes/"_s + name);

  // TODO some mesh optimization first?
  CORRADE_INTERNAL_ASSERT(s.converter->add(ds.finalizeMesh()));

  // TODO dxt compression first?
  CORRADE_INTERNAL_ASSERT(s.converter->add(ds.finalizeImage(ds.inputMaterials)));
  CORRADE_INTERNAL_ASSERT(s.converter->add(ds.finalizeTexture()));

  // TODO deduplication first?
  for(const Mn::Trade::MaterialData& i: ds.inputMaterials)
    CORRADE_INTERNAL_ASSERT(s.converter->add(i));

  CORRADE_INTERNAL_ASSERT(s.converter->add(ss.finalizeScene()));

  return s.converter->endFile() ? 0 : 1;
}
