
#include <cmath>

#include "VoxelWrapper.h"

namespace Mn = Magnum;
namespace Cr = Corrade;

namespace esp {
namespace geo {

#ifdef ESP_BUILD_WITH_VHACD
VoxelWrapper::VoxelWrapper(std::string& renderAssetHandle,
                           esp::scene::SceneNode* sceneNode,
                           esp::assets::ResourceManager& resourceManager_,
                           int resolution = 1000000)
    : SceneNode(sceneNode) {
  std::string voxelGridHandle =
      renderAssetHandle + "_" + std::to_string(resolution);
  // check for existence of specified VoxelGrid
  if (resourceManager_.voxelGridExists(
          voxelGridHandle)) {  // if it exists, simply point the wrapper to it.
    voxelGrid = resourceManager_.getVoxelGrid(voxelGridHandle);
  } else {  // if not, create a new voxel
    std::unique_ptr<esp::assets::MeshData> objMesh =
        esp::assets::MeshData::create_unique();
    objMesh = resourceManager_.createJoinedCollisionMesh(renderAssetHandle);
    voxelGrid =
        std::make_shared<VoxelGrid>(objMesh, renderAssetHandle, resolution);
    resourceManager_.registerVoxelGrid(voxelGridHandle, voxelGrid);
  }
}
#endif

VoxelWrapper::VoxelWrapper(std::string& handle,
                           esp::scene::SceneNode* sceneNode,
                           esp::assets::ResourceManager& resourceManager_,
                           Mn::Vector3& voxelSize,
                           Mn::Vector3i& voxelDimensions)
    : SceneNode(sceneNode) {
  int resolution = voxelDimensions[0] * voxelDimensions[1] * voxelDimensions[2];
  std::string voxelGridHandle = handle + "_" + std::to_string(resolution);
  // check for existence of specified VoxelGrid
  if (resourceManager_.voxelGridExists(
          voxelGridHandle)) {  // if it exists, simply point the wrapper to it.
    voxelGrid = resourceManager_.getVoxelGrid(voxelGridHandle);
  } else {  // if not, create a new voxel
    voxelGrid = std::make_shared<VoxelGrid>(voxelSize, voxelDimensions);
    resourceManager_.registerVoxelGrid(voxelGridHandle, voxelGrid);
  }
}

Mn::Vector3i VoxelWrapper::getVoxelIndexFromGlobalCoords(Mn::Vector3 coords) {
  // get absolute transform of Rigid Body.
  Mn::Matrix4 absTransform = SceneNode->Magnum::SceneGraph::AbstractObject<
      3, float>::absoluteTransformationMatrix();

  Mn::Vector4 paddedCoords = Mn::Vector4(coords[0], coords[1], coords[2], 1);
  // Apply inverse of the transformation to coords
  Mn::Vector4 paddedTransformedCoords = absTransform.inverted() * paddedCoords;
  // subtract VoxelGrid's m_offset
  Mn::Vector3 transformedCoords =
      Mn::Vector3(paddedTransformedCoords[0], paddedTransformedCoords[1],
                  paddedTransformedCoords[2]);
  transformedCoords -= voxelGrid->getOffset();
  // Divide by voxel size
  transformedCoords /= voxelGrid->getVoxelSize();
  // return the coords as Vector3i
  return Mn::Vector3i(transformedCoords + Mn::Vector3(0.5, 0.5, 0.5));
}

Mn::Vector3 VoxelWrapper::getGlobalCoordsFromVoxelIndex(Mn::Vector3i index) {
  Mn::Vector3 globalCoords = voxelGrid->getGlobalCoords(index);
  Mn::Vector4 paddedCoords =
      Mn::Vector4(globalCoords[0], globalCoords[1], globalCoords[2], 1);
  Mn::Matrix4 absTransform = SceneNode->Magnum::SceneGraph::AbstractObject<
      3, float>::absoluteTransformationMatrix();
  Mn::Vector4 paddedTransformedCoords = absTransform * paddedCoords;
  return Mn::Vector3(paddedTransformedCoords[0], paddedTransformedCoords[1],
                     paddedTransformedCoords[2]);
}

std::shared_ptr<VoxelGrid> VoxelWrapper::getVoxelGrid() {
  return voxelGrid;
}

/*void VoxelWrapper::setDraw(const std::string& gridName,
esp::gfx::DrawableGroup* drawables, bool drawVoxelization){
  esp::physics::Rigidbase* rigidBase;
  for (const auto& abstractFeature : SceneNode->features()) {
      auto feature = dynamic_cast<esp::physics::Rigidbase*>(&abstractFeature);
      if (feature)
        rigidBase = feature;
  }

  if (rigidBase->VoxelNode_ && !drawVoxelization) {
    // destroy the node
    delete rigidBase->VoxelNode_;
    rigidBase->VoxelNode_ = nullptr;

  } else if (drawVoxelization && rigidBase->visualNode_) {
    if (rigidBase->VoxelNode_) {
      // if the VoxelNode is already rendering something, destroy it.
      delete rigidBase->VoxelNode_;
    }

    // re-create the voxel node
    rigidBase->VoxelNode_ = &rigidBase->visualNode_->createChild();

    esp::geo::VoxelWrapper* voxelWrapper_ = rigidBase->voxelWrapper.get();
    gfx::Drawable::Flags meshAttributeFlags{};
    resourceManager_.createDrawable(
        voxelWrapper_->getVoxelGrid()->getMeshGL(gridName), meshAttributeFlags,
        *rigidBase->VoxelNode_, DEFAULT_LIGHTING_KEY,
        PER_VERTEX_OBJECT_ID_MATERIAL_KEY, drawables);

    // If the RigidBase is a stage, need to set the BB to make culling work.
    if (dynamic_cast<esp::physics::RigidStage*>(rigidBase) != nullptr) {
      // set bounding box for the node to be the bb computed by vhacd
      Mn::Range3D bb{rigidBase->voxelWrapper->getVoxelGrid()->getOffset(),
                     rigidBase->voxelWrapper->getVoxelGrid()->getMaxOffset()};
      rigidBase->VoxelNode_->setMeshBB(bb);
      //
      rigidBase->node().computeCumulativeBB();
    }
  }

}*/

}  // namespace geo
}  // namespace esp
