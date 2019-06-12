// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <Magnum/GL/TextureFormat.h>
#include <Magnum/Math/Color.h>

#include <Magnum/MeshTools/Compile.h>
#include <Magnum/MeshTools/Transform.h>
#include <Magnum/SceneGraph/MatrixTransformation3D.h>

#include "Asset.h"
#include "BaseMesh.h"
#include "GltfMeshData.h"
#include "MeshMetaData.h"
#include "PhysicsManager.h"
#include "esp/scene/SceneNode.h"

// forward declarations
namespace Magnum {
namespace Trade {
class AbstractImporter;
class AbstractShaderProgram;
class PhongMaterialData;
}  // namespace Trade
}  // namespace Magnum

namespace esp {
namespace gfx {
class Drawable;
}
namespace scene {
class SceneConfiguration;
}
namespace assets {

class ResourceManager {
 public:
  // TODO:
  // Singleton
  // a common design pattern for implementing
  // subsystems such as "resource manager", thats make up an engine is
  // to define a singleton class;
  explicit ResourceManager(){};
  ~ResourceManager() { LOG(INFO) << "Deconstructing ResourceManager"; }

  // Stores references to a set of drawable elements
  using DrawableGroup = Magnum::SceneGraph::DrawableGroup3D;
  // Convenience typedef for Importer class
  using Importer = Magnum::Trade::AbstractImporter;

  inline void compressTextures(bool newVal) { compressTextures_ = newVal; };

  // load data from given AssetInfo descriptor, and store internally
  // adding loaded assets as children of given SceneNode parent
  // if drawables is provided, add all drawable nodes to it
  bool loadScene(const AssetInfo& info,
                 scene::SceneNode* parent = nullptr,
                 DrawableGroup* drawables = nullptr);

  bool loadPhysicalScene(const AssetInfo& info,
                         PhysicsManager& _physicsManager,
                         scene::SceneNode* parent = nullptr,
                         bool attach_physics = false,
                         DrawableGroup* drawables = nullptr);

  // load an object. Eventually this will load from AssetInfo descriptor,
  // whereas currently it only loads dumb cubes
  bool loadObject(const AssetInfo& info,
                  PhysicsManager& _physicsManager,
                  scene::SceneNode* parent = nullptr,
                  bool attach_physics = true,
                  DrawableGroup* drawables = nullptr,
                  physics::BulletRigidObject** physNode = nullptr);

 protected:
  //! Load textures from importer into assets, and update metaData
  void loadTextures(Importer& importer, MeshMetaData* metaData);

  //! Load meshes from importer into assets, and update metaData
  void loadMeshes(Importer& importer,
                  MeshMetaData* metaData,
                  bool shiftOrigin = false);

  //! Load materials from importer into assets, and update metaData
  void loadMaterials(Importer& importer, MeshMetaData* metaData);

  //! Loads scene assets described by metaData into given SceneGraph, optionally
  //! forcing reload of GPU-side assets. Returns whether succeeded.
  bool createScene(Importer& importer,
                   const AssetInfo& info,
                   const MeshMetaData& metaData,
                   scene::SceneNode& sceneNode,
                   DrawableGroup* drawables,
                   bool forceReload = false);

  //! Loads object with given objectId from importer and metaData, and add to
  //! passed parent node within sceneGraph
  void createObject(Importer& importer,
                    const AssetInfo& info,
                    const MeshMetaData& metaData,
                    scene::SceneNode& parent,
                    DrawableGroup* drawables,
                    int objectId);

  bool loadPTexMeshData(const AssetInfo& info,
                        scene::SceneNode* parent,
                        DrawableGroup* drawables);

  bool loadInstanceMeshData(const AssetInfo& info,
                            scene::SceneNode* parent,
                            DrawableGroup* drawables);

  bool loadGeneralMeshData(const AssetInfo& info,
                           scene::SceneNode* parent,
                           DrawableGroup* drawables,
                           bool shiftOrigin = false,
                           scene::SceneNode* node = nullptr);

  bool loadSUNCGHouseFile(const AssetInfo& info,
                          scene::SceneNode* parent,
                          DrawableGroup* drawables);

  void shiftMeshDataToOrigin(GltfMeshData* meshDataGL);

  // ==== geometry data ====
  // shared_ptr is used here, instead of Corrade::Containers::Optional, or
  // std::optional because shared_ptr is reference type, not value type, and
  // thus we can avoiding duplicated loading
  std::vector<std::shared_ptr<BaseMesh>> meshes_;
  std::vector<std::shared_ptr<Magnum::GL::Texture2D>> textures_;
  std::vector<std::shared_ptr<Magnum::Trade::PhongMaterialData>> materials_;

  Magnum::GL::Mesh* instance_mesh;

  // a dictionary to check if a mesh has been loaded
  std::map<std::string, MeshMetaData> resourceDict_;

  //! Types of supported Shader programs
  enum ShaderType {
    INSTANCE_MESH_SHADER = 0,
    PTEX_MESH_SHADER = 1,
    COLORED_SHADER = 2,
    VERTEX_COLORED_SHADER = 3,
    TEXTURED_SHADER = 4,
  };

  // maps a name to the shader program
  std::map<ShaderType, std::shared_ptr<Magnum::GL::AbstractShaderProgram>>
      shaderPrograms_;

  //! Return Shader of given type, creating if necessary
  Magnum::GL::AbstractShaderProgram* getShaderProgram(ShaderType type);

  //! Create a Drawable with given ShaderType for the given Mesh and SceneNode
  //! If DrawableGroup3D group is given add created Drawable to the group
  //! Optional Texture2D, objectId and color arguments set relevant shader
  //! parameters
  gfx::Drawable& createDrawable(
      const ShaderType shaderType,
      Magnum::GL::Mesh& mesh,
      scene::SceneNode& node,
      Magnum::SceneGraph::DrawableGroup3D* group = nullptr,
      Magnum::GL::Texture2D* texture = nullptr,
      int objectId = ID_UNDEFINED,
      const Magnum::Color4& color = Magnum::Color4{1});

  bool compressTextures_ = false;
};

}  // namespace assets
}  // namespace esp
