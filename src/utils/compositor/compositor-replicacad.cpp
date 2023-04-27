// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <unordered_map>
#include <Corrade/Containers/GrowableArray.h>
#include <Corrade/Containers/Pair.h>
#include <Corrade/Containers/Triple.h>
#include <Corrade/Containers/StringStlHash.h>
#include <Corrade/Utility/Arguments.h>
#include <Corrade/Utility/Path.h>
#include <Magnum/Mesh.h>
#include <Magnum/Math/Matrix3.h>
#include <Magnum/MaterialTools/PhongToPbrMetallicRoughness.h>
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

  constexpr Mn::Vector2i TextureAtlasSize{4096}; // TODO resize this!!
  esp::CompositorState s{args.value("output")};
  esp::CompositorSceneState ss;
  esp::CompositorDataState ds{TextureAtlasSize};
  Cr::Containers::Pointer<Mn::Trade::AbstractImporter> importer = s.importerManager.loadAndInstantiate("AnySceneImporter");
  CORRADE_INTERNAL_ASSERT(importer);

  std::unordered_map<Cr::Containers::String, Cr::Containers::Pair<Mn::UnsignedInt, Mn::UnsignedInt>> uniqueMeshes;

  const auto import = [&](
    Cr::Containers::StringView filename,
    Cr::Containers::StringView name,
    Mn::Trade::MaterialType materialType)
  {
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

    /* Meshes are unfortunately named in a useless way, so override them with
       names from the objects referencing them instead */
    // TODO this isn't strictly needed, the names are just for debugging
    //  purposes ... but deduplication relies on them :/
    Cr::Containers::Array<Cr::Containers::String> meshNames{importer->meshCount()};
    for(Cr::Containers::Pair<Mn::UnsignedInt, Cr::Containers::Pair<Mn::UnsignedInt, Mn::Int>> objectMeshMaterial: scene->meshesMaterialsAsArray()) {
      const Mn::UnsignedInt meshId = objectMeshMaterial.second().first();
      Cr::Containers::String objectName = importer->objectName(objectMeshMaterial.first());
      if(!objectName)
        Mn::Fatal{} << "No name found for object" << objectMeshMaterial.first() << "referencing mesh" << importer->meshName(meshId) << "in" << Cr::Utility::Path::split(filename).second();

      if(meshNames[meshId] && meshNames[meshId] != objectName)
        Mn::Fatal{} << "Conflicting name for mesh" << importer->meshName(meshId) << Mn::Debug::nospace << ":" << meshNames[meshId] << "vs" << objectName << "in" << Cr::Utility::Path::split(filename).second();

      meshNames[meshId] = std::move(objectName);
    }

    /* Assuming materials are shared among meshes, remember the ID of already
       imported materials */
    Cr::Containers::Array<Cr::Containers::Optional<Mn::UnsignedInt>> importedMaterialIds{importer->materialCount()};

    /* Node mesh/material assignments. Each entry will be one child of the
       top-level object. */
    for(Cr::Containers::Triple<Mn::UnsignedInt, Mn::Int, Mn::Matrix4> transformationMeshMaterial: Mn::SceneTools::flattenMeshHierarchy3D(*scene)) {
      Cr::Containers::Optional<Mn::Trade::MeshData> mesh = importer->mesh(transformationMeshMaterial.first());
      Cr::Containers::StringView meshName = meshNames[transformationMeshMaterial.first()];

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

      /* Check if a mesh of the same name is already present and reuse it in
         that case, otherwise add to the map. Not using
         `importer->meshName(transformationMeshMaterial.first())`
         because the mesh names are useless `Mesh.XYZ` that don't match between
         files. */
      // TODO avoid string copies by making the map over StringViews? but then
      //  it'd have to be changed again once we don't have the extra meshNames
      //  array
      bool duplicate = false;
      std::unordered_map<Cr::Containers::String, Cr::Containers::Pair<Mn::UnsignedInt, Mn::UnsignedInt>>::iterator found = uniqueMeshes.find(meshName);
      if(found != uniqueMeshes.end()) {
        if(mesh->indexCount() == found->second.second()) {
          m.meshIndexOffset = found->second.first();
          duplicate = true;
        } else {
          Mn::Warning{} << "Mesh" << meshName << "in" << Cr::Utility::Path::split(filename).second() << "has" << mesh->indexCount() << "indices but expected" << found->second.second() << Mn::Debug::nospace << ", adding a new copy";
        }
      }

      // TODO broken (meshes with the same name being totally different across
      //  the door files), use this only for the Stage_* files
      if(true || !duplicate) {
        Mn::Debug{} << "New mesh" << meshName << "in" << Cr::Utility::Path::split(filename).second();
        uniqueMeshes.insert({std::move(meshName), {ds.indexOffset, mesh->indexCount()}});

        m.meshIndexOffset = ds.indexOffset;
        ds.indexOffset += mesh->indexCount()*4; // TODO ints hardcoded

        // TODO convert from a strip/... if not triangles
        arrayAppend(ds.inputMeshes, *std::move(mesh));
      } else {
        Mn::Debug{} << "Mesh" << meshName << "in" << Cr::Utility::Path::split(filename).second() << "already present";
      }

      /* If the material is already parsed, reuse its ID */
      if(const Cr::Containers::Optional<Mn::UnsignedInt> materialId = importedMaterialIds[transformationMeshMaterial.second()]) {
        m.meshMaterial = *materialId;

      /* Otherwise parse it. If textured, extract the image as well. */
      } else {
        Cr::Containers::Optional<Mn::Trade::MaterialData> material = importer->material(transformationMeshMaterial.second());
        CORRADE_INTERNAL_ASSERT(material);

        /* Convert to a PBR metallic/roughness if it's a Phong material */
        if(material->types() & Mn::Trade::MaterialType::Phong)
          material = Mn::MaterialTools::phongToPbrMetallicRoughness(*material,
            Mn::MaterialTools::PhongToPbrMetallicRoughnessFlag::DropUnconvertableAttributes);

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
          // TODO detection of single-color images here
          // TODO StbResizeImageConverter instead of the assert
          CORRADE_INTERNAL_ASSERT((image->size() <= TextureAtlasSize).all());
          /* Add texture scaling if the image is smaller than 2K */
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
          Mn::Debug{} << "New untextured material for" << meshName << "in" << Cr::Utility::Path::split(filename).second();

          arrayAppend(attributes, {
            {Mn::Trade::MaterialAttribute::BaseColorTexture, 0u},
            {Mn::Trade::MaterialAttribute::BaseColorTextureLayer, 0u},
            {Mn::Trade::MaterialAttribute::BaseColorTextureMatrix,
              Mn::Matrix3::scaling(Mn::Vector2{1}/Mn::Vector2(TextureAtlasSize))}
          });
        }

        // TODO MaterialTools::merge() instead of this!!!!
        CORRADE_INTERNAL_ASSERT(material->layerCount() == 1);
        arrayAppend(attributes, material->attributeData());

        /* The material type is overriden to Flat for some */
        material = Mn::Trade::MaterialData{materialType, std::move(attributes)};

        importedMaterialIds[transformationMeshMaterial.second()] = m.meshMaterial = ds.inputMaterials.size();
        arrayAppend(ds.inputMaterials, *std::move(material));
      }

      arrayAppend(ss.meshes, m);
    }
  };

  Cr::Containers::String replicaPath = Cr::Utility::Path::join(args.value("input"), "ReplicaCAD_dataset_v1.5");

  /* Stage_v3_sc*_staging are already pre-baked so Flat, frl_apartment_stage
     is not */
  import(Cr::Utility::Path::join({replicaPath, "stages/frl_apartment_stage.glb"}), "data/replica_cad/configs/stages/../../stages/frl_apartment_stage.glb", Mn::Trade::MaterialType::PbrMetallicRoughness);
  for(const char* name: {
    "Stage_v3_sc0_staging.glb",
    "Stage_v3_sc1_staging.glb",
    "Stage_v3_sc2_staging.glb",
    "Stage_v3_sc3_staging.glb"
  })
    import(Cr::Utility::Path::join({replicaPath, "stages", name}), "data/replica_cad/configs/stages/../../stages/"_s + name, Mn::Trade::MaterialType::Flat);

  for(const char* name: {
    "frl_apartment_basket.glb",
    "frl_apartment_beanbag.glb",
    "frl_apartment_bike_01.glb",
    "frl_apartment_bike_02.glb",
    "frl_apartment_bin_01.glb",
    "frl_apartment_bin_02.glb",
    "frl_apartment_bin_03.glb",
    "frl_apartment_book_01.glb",
    "frl_apartment_book_02.glb",
    "frl_apartment_book_03.glb",
    "frl_apartment_book_04.glb",
    "frl_apartment_book_05.glb",
    "frl_apartment_book_06.glb",
    "frl_apartment_bowl_01.glb",
    "frl_apartment_bowl_02.glb",
    "frl_apartment_bowl_03.glb",
    "frl_apartment_bowl_06.glb",
    "frl_apartment_bowl_07.glb",
    "frl_apartment_box.glb",
    "frl_apartment_cabinet.glb",
    "frl_apartment_camera_02.glb",
    "frl_apartment_clock.glb",
    "frl_apartment_clothes_hanger_01.glb",
    "frl_apartment_clothes_hanger_02.glb",
    "frl_apartment_cloth_01.glb",
    "frl_apartment_cloth_02.glb",
    "frl_apartment_cloth_03.glb",
    "frl_apartment_cloth_04.glb",
    "frl_apartment_cup_01.glb",
    "frl_apartment_cup_02.glb",
    "frl_apartment_cup_03.glb",
    "frl_apartment_cup_05.glb",
    "frl_apartment_cushion_01.glb",
    "frl_apartment_cushion_03.glb",
    "frl_apartment_handbag.glb",
    "frl_apartment_chair_01.glb",
    "frl_apartment_chair_04.glb",
    "frl_apartment_chair_05.glb",
    "frl_apartment_choppingboard_02.glb",
    "frl_apartment_indoor_plant_01.glb",
    "frl_apartment_indoor_plant_02.glb",
    "frl_apartment_kitchen_utensil_01.glb",
    "frl_apartment_kitchen_utensil_02.glb",
    "frl_apartment_kitchen_utensil_03.glb",
    "frl_apartment_kitchen_utensil_04.glb",
    "frl_apartment_kitchen_utensil_05.glb",
    "frl_apartment_kitchen_utensil_06.glb",
    "frl_apartment_kitchen_utensil_08.glb",
    "frl_apartment_kitchen_utensil_09.glb",
    "frl_apartment_knifeblock.glb",
    "frl_apartment_lamp_01.glb",
    "frl_apartment_lamp_02.glb",
    "frl_apartment_mat.glb",
    "frl_apartment_monitor.glb",
    "frl_apartment_monitor_stand.glb",
    "frl_apartment_pan_01.glb",
    "frl_apartment_picture_01.glb",
    "frl_apartment_picture_02.glb",
    "frl_apartment_picture_03.glb",
    "frl_apartment_picture_04.glb",
    "frl_apartment_plate_01.glb",
    "frl_apartment_plate_02.glb",
    "frl_apartment_rack_01.glb",
    "frl_apartment_refrigerator.glb",
    "frl_apartment_remote-control_01.glb",
    "frl_apartment_rug_01.glb",
    "frl_apartment_rug_02.glb",
    "frl_apartment_setupbox.glb",
    "frl_apartment_shoebox_01.glb",
    "frl_apartment_shoe_01.glb",
    "frl_apartment_shoe_02.glb",
    "frl_apartment_shoe_03.glb",
    "frl_apartment_shoe_04.glb",
    "frl_apartment_small_appliance_01.glb",
    "frl_apartment_small_appliance_02.glb",
    "frl_apartment_sofa.glb",
    "frl_apartment_sponge_dish.glb",
    "frl_apartment_stool_02.glb",
    "frl_apartment_table_01.glb",
    "frl_apartment_table_02.glb",
    "frl_apartment_table_03.glb",
    "frl_apartment_table_04.glb",
    "frl_apartment_towel.glb",
    "frl_apartment_tv_object.glb",
    "frl_apartment_tv_screen.glb",
    "frl_apartment_tvstand.glb",
    "frl_apartment_umbrella.glb",
    "frl_apartment_vase_01.glb",
    "frl_apartment_vase_02.glb",
    "frl_apartment_wall_cabinet_01.glb",
    "frl_apartment_wall_cabinet_02.glb",
    "frl_apartment_wall_cabinet_03.glb"
  })
    import(Cr::Utility::Path::join({replicaPath, "objects", name}), "data/replica_cad/configs/objects/../../objects/"_s + name, Mn::Trade::MaterialType::PbrMetallicRoughness);

  for(const char* name: {
    "doors/double_door_R.glb",
    "doors/door2.glb",
    "doors/door1.glb",
    "doors/door3.glb",
    "doors/double_door_L.glb",
    "cabinet/door.glb",
    "cabinet/cabinet.glb",
    "chest_of_drawers/chestOfDrawers_DrawerBot.glb",
    "chest_of_drawers/chestOfDrawers_base.glb",
    "chest_of_drawers/chestOfDrawers_DrawerTop.glb",
    "chest_of_drawers/chestOfDrawers_DrawerMid.glb",
    "kitchen_counter/drawer1.glb",
    "kitchen_counter/kitchen_counter.glb",
    "kitchen_counter/drawer4.glb",
    "kitchen_counter/drawer2.glb",
    "kitchen_counter/drawer3.glb",
    "fridge/bottom_door.glb",
    "fridge/body.glb",
    "fridge/top_door.glb",
    "kitchen_cupboards/kitchencupboard_doorWindow_R.glb",
    "kitchen_cupboards/kitchencupboard_base.glb",
    "kitchen_cupboards/kitchencupboard_doorWhole_R.glb",
    "kitchen_cupboards/kitchencupboard_doorWhole_L.glb",
    "kitchen_cupboards/kitchencupboard_doorWindow_L.glb"
  })
    import(Cr::Utility::Path::join({replicaPath, "urdf", name}), "data/replica_cad/urdf/"_s + name, Mn::Trade::MaterialType::PbrMetallicRoughness);

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
