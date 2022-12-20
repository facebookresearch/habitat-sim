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
#include <Magnum/MaterialTools/PhongToPbrMetallicRoughness.h>
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

int main(int argc, char** argv) {
  Cr::Utility::Arguments args;
  args.addArgument("input").setHelp("input", "input file prefix")
    .addArgument("output").setHelp("output", "output file")
    .parse(argc, argv);

  constexpr Mn::Vector2i TextureAtlasSize{512};
  esp::CompositorState s{args.value("output")};
  esp::CompositorSceneState ss;
  esp::CompositorDataState ds{TextureAtlasSize};
  Cr::Containers::Pointer<Mn::Trade::AbstractImporter> importer = s.importerManager.loadAndInstantiate("AnySceneImporter");
  CORRADE_INTERNAL_ASSERT(importer);

  Cr::Containers::Pointer<Mn::Trade::AbstractImageConverter> imageResizer = s.imageConverterManager.loadAndInstantiate("StbResizeImageConverter");
  CORRADE_INTERNAL_ASSERT(imageResizer);
  imageResizer->configuration().setValue("size", Mn::Vector2i{512});
  imageResizer->configuration().setValue("upsample", false);

  std::unordered_map<Cr::Containers::String, Cr::Containers::Pair<Mn::UnsignedInt, Mn::UnsignedInt>> uniqueMeshes;
  const auto importSingleMesh = [&](
    Cr::Containers::StringView filename,
    Cr::Containers::StringView name,
    const Mn::Color4& color
  ) {
    CORRADE_INTERNAL_ASSERT_OUTPUT(importer->openFile(filename));
    CORRADE_INTERNAL_ASSERT(importer->meshCount() == 1);

    esp::Parent root;
    root.mapping = ss.parents.size();
    root.parent = -1;
    arrayAppend(ss.parents, root);
    s.converter->setObjectName(root.mapping, name);

    esp::Parent o;
    o.mapping = ss.parents.size();
    o.parent = root.mapping;
    arrayAppend(ss.parents, o);

    Cr::Containers::Optional<Mn::Trade::MeshData> mesh = importer->mesh(0);
    CORRADE_INTERNAL_ASSERT(mesh && mesh->primitive() == Mn::MeshPrimitive::Triangles);
    /* Assuming STL files, which are not indexed and thus with many duplicate
       vertices. Create an index buffer by deduplicating them. */
    // TODO possibly useful to filter away and recreate normals also
    //  --> disable perFaceToPerVertex for stlimporter?
    mesh = Mn::MeshTools::removeDuplicates(*mesh);

    esp::Mesh m;
    m.mapping = o.mapping;
    m.mesh = 0;
    m.meshIndexCount = mesh->indexCount();
    m.meshMaterial = ds.inputMaterials.size();
    m.meshIndexOffset = ds.indexOffset;
    ds.indexOffset += mesh->indexCount()*sizeof(Mn::UnsignedInt);

    // TODO convert from a strip/... if not triangles
    arrayAppend(ds.inputMeshes, *std::move(mesh));

    /* Add an empty colored material */
    arrayAppend(ds.inputMaterials, Mn::Trade::MaterialData{Mn::Trade::MaterialType::PbrMetallicRoughness, {
      {Mn::Trade::MaterialAttribute::BaseColor, color},
      {Mn::Trade::MaterialAttribute::BaseColorTexture, 0u},
      {Mn::Trade::MaterialAttribute::BaseColorTextureLayer, 0u},
      {Mn::Trade::MaterialAttribute::BaseColorTextureMatrix,
        Mn::Matrix3::scaling(Mn::Vector2{1}/Mn::Vector2(TextureAtlasSize))}
    }});

    arrayAppend(ss.meshes, m);
  };

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
      // TODO drop material deduplication, useless here
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

        material = Mn::Trade::MaterialData{material->types(), std::move(attributes)};

        importedMaterialIds[transformationMeshMaterial.second()] = m.meshMaterial = ds.inputMaterials.size();
        arrayAppend(ds.inputMaterials, *std::move(material));
      }

      arrayAppend(ss.meshes, m);
    }
  };

  Cr::Containers::String fetchPath = Cr::Utility::Path::join(args.value("input"), "hab_fetch_v1.0/meshes/");
  // TODO fetch (HAHA) the colors automagically from the URDF, this is insufferable
  for(auto i: std::initializer_list<Cr::Containers::Pair<const char*, Mn::Color4>>{
    {"bellows_link.STL", {0.0f, 0.0f, 0.0f}},
    {"estop_link.STL", {0.8f, 0.0f, 0.0f}},
    {"laser_link.STL", {0.792156862745098f, 0.819607843137255f, 0.933333333333333f}},
    {"l_gripper_finger_link_opt.stl", {0.356f, 0.361f, 0.376f}},
    {"l_gripper_finger_link.STL", {0.356f, 0.361f, 0.376f}},
    {"l_wheel_link.STL", {0.086f, 0.506f, 0.767f}},
    {"r_gripper_finger_link_opt.stl", {0.356f, 0.361f, 0.376f}},
    {"r_gripper_finger_link.STL", {0.356f, 0.361f, 0.376f}},
    {"r_wheel_link.STL", {0.086f, 0.506f, 0.767f}},
  })
    importSingleMesh(Cr::Utility::Path::join(fetchPath, i.first()), "./data/robots/hab_fetch/robots/../meshes/"_s + i.first(), i.second());

  // TODO remove!
  // importSingleMesh(Cr::Utility::Path::join(args.value("input"), "cubeSolid.gltf"), "./data/cubeSolid.gltf", 0xffffff_rgbf);

  for(const char* name: {
    "base_link.dae",
    "elbow_flex_link.dae",
    "estop_link.dae",
    "forearm_roll_link.dae",
    "gripper_link.dae",
    "head_pan_link.dae",
    "head_tilt_link.dae",
    "shoulder_lift_link.dae",
    "shoulder_pan_link.dae",
    "suctionCup.glb",
    "torso_fixed_link.dae",
    "torso_lift_link.dae",
    "upperarm_roll_link.dae",
    "wrist_flex_link.dae",
    "wrist_roll_link.dae",
  })
    import(Cr::Utility::Path::join(fetchPath, name), "./data/robots/hab_fetch/robots/../meshes/"_s + name);

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
