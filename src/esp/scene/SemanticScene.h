// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_SCENE_SEMANTICSCENE_H_
#define ESP_SCENE_SEMANTICSCENE_H_

#include <Corrade/Utility/Path.h>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "esp/core/Esp.h"
#include "esp/geo/OBB.h"
#include "esp/io/Json.h"

namespace esp {
namespace scene {

//! Represents a semantic category
class SemanticCategory {
 public:
  virtual ~SemanticCategory() = default;
  //! Return index of SemanticCategory under given mapping
  virtual int index(const std::string& mapping = "") const = 0;
  //! Return name of SemanticCategory under given mapping
  virtual std::string name(const std::string& mapping = "") const = 0;

  ESP_SMART_POINTERS(SemanticCategory)
};

// forward declarations
class SemanticObject;

// semantic object only used to represent characteristics of Connected
// Components of mesh verts conditioned on semantic ID/color.
class CCSemanticObject;
class SemanticRegion;
class SemanticLevel;

//! Represents a scene with containing semantically annotated
//! levels, regions and objects
class SemanticScene {
 public:
  ~SemanticScene() { ESP_DEBUG() << "Deconstructing SemanticScene"; }
  //! return axis aligned bounding box of this House
  box3f aabb() const { return bbox_; }

  //! return total number of given element type
  int count(const std::string& element) const {
    return elementCounts_.at(element);
  }

  //! return all SemanticCategories of objects in this House
  const std::vector<std::shared_ptr<SemanticCategory>>& categories() const {
    return categories_;
  }

  //! return all Levels in this House
  const std::vector<std::shared_ptr<SemanticLevel>>& levels() const {
    return levels_;
  }

  //! return all Regions in this House
  const std::vector<std::shared_ptr<SemanticRegion>>& regions() const {
    return regions_;
  }

  //! return all Objects in this House
  const std::vector<std::shared_ptr<SemanticObject>>& objects() const {
    return objects_;
  }

  const std::unordered_map<int, int>& getSemanticIndexMap() const {
    return segmentToObjectIndex_;
  }

  //! convert semantic mesh mask index to object index or ID_UNDEFINED if
  //! not mapped
  inline int semanticIndexToObjectIndex(int maskIndex) const {
    auto segmentToObjIdxIter = segmentToObjectIndex_.find(maskIndex);
    if (segmentToObjIdxIter != segmentToObjectIndex_.end()) {
      return segmentToObjIdxIter->second;
    } else {
      return ID_UNDEFINED;
    }
  }

  /**
   * @brief Attempt to load SemanticScene descriptor from an unknown file type.
   * @param filename the name of the semantic scene descriptor (house file) to
   * attempt to load
   * @param scene reference to sceneNode to assign semantic scene to
   * @param rotation rotation to apply to semantic scene upon load.
   * @return successfully loaded
   */
  static bool loadSemanticSceneDescriptor(
      const std::string& filename,
      SemanticScene& scene,
      const quatf& rotation = quatf::FromTwoVectors(-vec3f::UnitZ(),
                                                    geo::ESP_GRAVITY));

  /**
   * @brief Attempt to load SemanticScene from a Gibson dataset house format
   * file
   * @param filename the name of the semantic scene descriptor (house file) to
   * attempt to load
   * @param scene reference to sceneNode to assign semantic scene to
   * @param rotation rotation to apply to semantic scene upon load.
   * @return successfully loaded
   */
  static bool loadGibsonHouse(
      const std::string& filename,
      SemanticScene& scene,
      const quatf& rotation = quatf::FromTwoVectors(-vec3f::UnitZ(),
                                                    geo::ESP_GRAVITY));

  /**
   * @brief Attempt to load SemanticScene from a HM3D dataset house
   * format file
   * @param filename the name of the semantic scene descriptor (house file) to
   * attempt to load
   * @param scene reference to sceneNode to assign semantic scene to
   * @param rotation rotation to apply to semantic scene upon load (currently
   * not used for HM3D)
   * @return successfully loaded
   */
  static bool loadHM3DHouse(const std::string& filename,
                            SemanticScene& scene,
                            CORRADE_UNUSED const quatf& rotation =
                                quatf::FromTwoVectors(-vec3f::UnitZ(),
                                                      geo::ESP_GRAVITY));

  /**
   * @brief Attempt to load SemanticScene from a Matterport3D dataset house
   * format file
   * @param filename the name of the semantic scene descriptor (house file) to
   * attempt to load
   * @param scene reference to sceneNode to assign semantic scene to
   * @param rotation rotation to apply to semantic scene upon load.
   * @return successfully loaded
   */
  static bool loadMp3dHouse(
      const std::string& filename,
      SemanticScene& scene,
      const quatf& rotation = quatf::FromTwoVectors(-vec3f::UnitZ(),
                                                    geo::ESP_GRAVITY));

  /**
   * @brief Attempt to load SemanticScene from a Replica dataset house format
   * file
   * @param filename the name of the semantic scene descriptor (house file) to
   * attempt to load
   * @param scene reference to sceneNode to assign semantic scene to
   * @param rotation rotation to apply to semantic scene upon load.
   * @return successfully loaded
   */
  static bool loadReplicaHouse(
      const std::string& filename,
      SemanticScene& scene,
      const quatf& rotation = quatf::FromTwoVectors(-vec3f::UnitZ(),
                                                    geo::ESP_GRAVITY));

  /**
   * @brief Builds a mapping of connected component-driven bounding boxes (via
   * @ref CCSemanticObject), keyed by criteria used to decide connectivity (the
   * per-vertex attribute suche as color).  If a @p semanticScene is passed, key
   * for resultant map will be semanticID of object with specified color,
   * otherwise key is hex color value.
   * @param verts Ref to the vertex buffer holding all vertex positions in the
   * mesh.
   * @param clrsToComponents an unordered map, keyed by tag/color value encoded
   * as uint, where the value is a vector of all sets of CCs consisting of verts
   * with specified tag/"color". (see @ref findCCsByGivenColor).
   * @param semanticScene The SSD for the current semantic mesh.  Used to query
   * semantic objs. If nullptr, this function returns hex-color-keyed map,
   * otherwise returns SemanticID-keyed map.
   * @return A map keyed by a representattion of the per-vertex "color" where
   * each entry contains a vector of values for all the CCs of verts having the
   * "color" attribute specified by the key.  Each element in the vector is a
   * smart pointer to a @ref CCSemanticObject, being used to facilitate
   * collecting pertinent data.
   */
  static std::unordered_map<uint32_t,
                            std::vector<std::shared_ptr<CCSemanticObject>>>
  buildCCBasedSemanticObjs(
      const std::vector<Mn::Vector3>& verts,
      const std::unordered_map<uint32_t, std::vector<std::set<uint32_t>>>&
          clrsToComponents,
      const std::shared_ptr<SemanticScene>& semanticScene);

  /**
   * @brief Build semantic OBBs based on presence of semantic IDs on vertices.
   * @param verts Ref to the vertex buffer holding all vertex positions in the
   * mesh.
   * @param vertSemanticIDs Ref to per-vertex semantic IDs persent on source
   * mesh, both known in semantic scene descriptor, and unknown.  Known IDs are
   * expected to start at 1 and be contiguous, followed by unknown semantic IDs
   * @param ssdObjs The known semantic scene descriptor objects for the mesh
   * @param msgPrefix Debug message prefix, referencing caller.
   * @return vector of semantic object IDXs that have no vertex mapping/presence
   * in the source mesh.
   */
  static std::vector<uint32_t> buildSemanticOBBs(
      const std::vector<Mn::Vector3>& verts,
      const std::vector<uint16_t>& vertSemanticIDs,
      const std::vector<std::shared_ptr<SemanticObject>>& ssdObjs,
      const std::string& msgPrefix);

  /**
   * @brief Build semantic object OBBs based on the accumulated
   * per-semantic-color CCs, and some criteria specified in @p semanticScene .
   * @param verts Ref to the vertex buffer holding all vertex positions in the
   * mesh.
   * @param clrsToComponents an unordered map, keyed by tag/color value encoded
   * as uint, where the value is a vector of all sets of CCs consisting of verts
   * with specified tag/"color". (see @ref findCCsByGivenColor).
   * @param semanticScene The SSD for the current semantic mesh.  Used to query
   * semantic objs. If nullptr, this function returns hex-color-keyed map,
   * otherwise returns SemanticID-keyed map.
   * @param maxVolFraction Fraction of maximum volume bbox CC to include in bbox
   * calc.
   * @param msgPrefix Debug message prefix, referencing caller.
   * @return vector of semantic object IDXs that have no vertex mapping/presence
   * in the source mesh.
   */
  static std::vector<uint32_t> buildSemanticOBBsFromCCs(
      const std::vector<Mn::Vector3>& verts,
      const std::unordered_map<uint32_t, std::vector<std::set<uint32_t>>>&
          clrsToComponents,
      const std::shared_ptr<SemanticScene>& semanticScene,
      float maxVolFraction,
      const std::string& msgPrefix);

  /**
   * @brief Whether the source file assigns colors to verts for Semantic
   * Mesh.
   */
  bool hasVertColorsDefined() const { return hasVertColors_; }

  /**
   * @brief return a read-only reference to the semantic color map, where value
   * is annotation color and index corresponds to id for that color. (HM3D only
   * currently)
   */
  const std::vector<Mn::Vector3ub>& getSemanticColorMap() const {
    return semanticColorMapBeingUsed_;
  }

  /**
   * @brief return a read-only reference to a mapping of semantic
   * color-as-integer to id and region id.(HM3D only currently)
   */
  const std::unordered_map<uint32_t, std::pair<int, int>>&
  getSemanticColorToIdAndRegionMap() const {
    return semanticColorToIdAndRegion_;
  }

  /**
   * @brief whether or not we should build bounding boxes around vertex
   * annotations on semantic asset load. Currently used for HM3D.
   */

  bool buildBBoxFromVertColors() const { return needBBoxFromVertColors_; }

  /**
   * @brief What fraction of largest connected component bbox to use
   * for bbox generation.  Will always use single largest, along with this
   * fraction of remaining. If set to 0.0, use all CCs (i.e. will bypass CC calc
   * and just use naive vert position mechanism for bbox), if set to 1.0, only
   * use single CC with largest volume.  Only used if @ref
   * needBBoxFromVertColors_ is true.
   */
  float CCFractionToUseForBBox() const { return ccLargestVolToUseForBBox_; }

 protected:
  /**
   * @brief Verify a requested file exists.
   * @param filename the file to attempt to load
   * @param srcFunc calling function name to be displayed in failure message
   * @return whether found or not
   */
  static bool checkFileExists(const std::string& filename,
                              const std::string& srcFunc) {
    if (!Cr::Utility::Path::exists(filename)) {
      ESP_WARNING(Mn::Debug::Flag::NoSpace)
          << "::" << srcFunc << ": File" << filename
          << "does not exist.  Aborting load.";
      return false;
    }
    return true;
  }  // checkFileExists

  /**
   * @brief Build the HM3D semantic data from the passed file stream.  File
   * being streamed is expected to be appropriate format.
   * @param ifs The opened file stream describing the HM3D semantic annotations.
   * @param scene reference to sceneNode to assign semantic scene to
   * @param rotation rotation to apply to semantic scene upon load (currently
   * not used for HM3D)
   * @return successfully built. Currently only returns true, but retaining
   * return value for future support.
   */
  static bool buildHM3DHouse(std::ifstream& ifs,
                             SemanticScene& scene,
                             CORRADE_UNUSED const quatf& rotation =
                                 quatf::FromTwoVectors(-vec3f::UnitZ(),
                                                       geo::ESP_GRAVITY));
  /**
   * @brief Build the mp3 semantic data from the passed file stream. File being
   * streamed is expected to be appropriate format.
   * @param ifs The opened file stream describing the Mp3d semantic annotations.
   * @param scene reference to sceneNode to assign semantic scene to
   * @param rotation rotation to apply to semantic scene upon load.
   * @return successfully built. Currently only returns true, but retaining
   * return value for future support.
   */
  static bool buildMp3dHouse(
      std::ifstream& ifs,
      SemanticScene& scene,
      const quatf& rotation = quatf::FromTwoVectors(-vec3f::UnitZ(),
                                                    geo::ESP_GRAVITY));

  /**
   * @brief Build SemanticScene from a Gibson dataset house JSON. JSON is
   * expected to have been verified already.
   * @param jsonDoc the JSON document describing the semantic annotations.
   * @param scene reference to sceneNode to assign semantic scene to
   * @param rotation rotation to apply to semantic scene upon load.
   * @return successfully built. Currently only returns true, but retaining
   * return value for future support.
   */
  static bool buildGibsonHouse(
      const io::JsonDocument& jsonDoc,
      SemanticScene& scene,
      const quatf& rotation = quatf::FromTwoVectors(-vec3f::UnitZ(),
                                                    geo::ESP_GRAVITY));

  /**
   * @brief Build SemanticScene from a Replica dataset house JSON. JSON is
   * expected to have been verified already.
   * @param jsonDoc the JSON document describing the semantic annotations.
   * @param scene reference to sceneNode to assign semantic scene to
   * @param objectsExist whether objects cell exists in json. This cell will not
   * exist in ReplicaCAD semantic lexicon.
   * @param rotation rotation to apply to semantic scene upon load.
   * @return successfully built. Currently only returns true, but retaining
   * return value for future support.
   */
  static bool buildReplicaHouse(
      const io::JsonDocument& jsonDoc,
      SemanticScene& scene,
      bool objectsExist,
      const quatf& rotation = quatf::FromTwoVectors(-vec3f::UnitZ(),
                                                    geo::ESP_GRAVITY));

  // Currently only supported by HM3D semantic files.
  bool hasVertColors_ = false;

  /**
   * @Brief whether or not we should build bounding boxes around vertex
   * annotations on semantic asset load. Currently used for HM3D.
   */
  bool needBBoxFromVertColors_ = false;

  /**
   * @brief Fraction of vert count for largest CC to use for BBox synth.  0->use
   * all (bypass CC calc), 1->use only largest CC.
   */
  float ccLargestVolToUseForBBox_ = 0.0f;

  std::string name_;
  std::string label_;
  box3f bbox_;
  std::map<std::string, int> elementCounts_;
  std::vector<std::shared_ptr<SemanticCategory>> categories_;
  std::vector<std::shared_ptr<SemanticLevel>> levels_;
  std::vector<std::shared_ptr<SemanticRegion>> regions_;
  std::vector<std::shared_ptr<SemanticObject>> objects_;
  //! map from combined region-segment id to objectIndex for semantic mesh
  std::unordered_map<int, int> segmentToObjectIndex_;
  /**
   * @brief List of mapped vertex colors, where index corresponds to object
   * Index/semantic ID (HM3D only currently)
   */
  std::vector<Mn::Vector3ub> semanticColorMapBeingUsed_{};
  /**
   * @brief Map of integer color to segment and region ids. Used for
   * transforming provided vertex colors. (HM3D only currently)
   */
  std::unordered_map<uint32_t, std::pair<int, int>>
      semanticColorToIdAndRegion_{};

  ESP_SMART_POINTERS(SemanticScene)
};

//! Represents a level of a SemanticScene
class SemanticLevel {
 public:
  virtual ~SemanticLevel() = default;
  virtual std::string id() const { return std::to_string(index_); }

  const std::vector<std::shared_ptr<SemanticRegion>>& regions() const {
    return regions_;
  }

  const std::vector<std::shared_ptr<SemanticObject>>& objects() const {
    return objects_;
  }

  box3f aabb() const { return bbox_; }

 protected:
  int index_{};
  std::string labelCode_;
  vec3f position_;
  box3f bbox_;
  std::vector<std::shared_ptr<SemanticObject>> objects_;
  std::vector<std::shared_ptr<SemanticRegion>> regions_;
  friend SemanticScene;
  ESP_SMART_POINTERS(SemanticLevel)
};

//! Represents a region (typically room) in a level of a house
class SemanticRegion {
 public:
  virtual ~SemanticRegion() = default;
  virtual std::string id() const {
    if (level_ != nullptr) {
      return level_->id() + "_" + std::to_string(index_);
    } else {
      return "_" + std::to_string(index_);
    }
  }
  int getIndex() const { return index_; }
  SemanticLevel::ptr level() const { return level_; }

  const std::vector<std::shared_ptr<SemanticObject>>& objects() const {
    return objects_;
  }

  box3f aabb() const { return bbox_; }

  SemanticCategory::ptr category() const { return category_; }

 protected:
  int index_{};
  int parentIndex_{};
  std::shared_ptr<SemanticCategory> category_;
  vec3f position_;
  box3f bbox_;
  vec3f floorNormal_;
  std::vector<vec3f> floorPoints_;
  std::vector<std::shared_ptr<SemanticObject>> objects_;
  std::shared_ptr<SemanticLevel> level_;
  friend SemanticScene;
  ESP_SMART_POINTERS(SemanticRegion)
};

//! Represents a distinct semantically annotated object
class SemanticObject {
 public:
  virtual ~SemanticObject() = default;
  virtual std::string id() const {
    if (region_ != nullptr) {
      return region_->id() + "_" + std::to_string(index_);
    } else {
      return "_" + std::to_string(index_);
    }
  }

  /**
   * @brief Retrieve the unique semantic ID corresponding to this object
   */
  int semanticID() const { return index_; }

  SemanticRegion::ptr region() const { return region_; }

  box3f aabb() const { return obb_.toAABB(); }

  geo::OBB obb() const { return obb_; }

  SemanticCategory::ptr category() const { return category_; }

  void setObb(const esp::vec3f& center,
              const esp::vec3f& dimensions,
              const esp::quatf& rotation = quatf::Identity()) {
    obb_ = geo::OBB{center, dimensions, rotation};
  }
  void setObb(const geo::OBB& otr) { obb_ = geo::OBB{otr}; }
  Mn::Vector3ub getColor() const { return color_; }

  void setColor(Mn::Vector3ub _color) {
    color_ = _color;
    // update colorAsInt_
    colorAsInt_ = geo::getValueAsUInt(color_);
  }

  uint32_t getColorAsInt() const { return colorAsInt_; }

  void setColorAsInt(const uint32_t _colorAsInt) {
    colorAsInt_ = _colorAsInt;
    // update color_ vector
    color_ = {uint8_t((colorAsInt_ >> 16) & 0xff),
              uint8_t((colorAsInt_ >> 8) & 0xff), uint8_t(colorAsInt_ & 0xff)};
  }

 protected:
  /**
   * @brief The unique semantic ID corresponding to this object
   */
  int index_{};
  /**
   * @brief specified color for this object instance.
   */
  Mn::Vector3ub color_{};
  /**
   * @brief  specified color as unsigned int
   */
  uint32_t colorAsInt_{};
  /**
   * @brief References the parent region for this object
   */
  int parentIndex_{};
  std::shared_ptr<SemanticCategory> category_;
  geo::OBB obb_;
  std::shared_ptr<SemanticRegion> region_;
  friend SemanticScene;
  ESP_SMART_POINTERS(SemanticObject)
};  // class SemanticObject

/**
 * @brief This class exists to facilitate semantic object data access for bboxes
 * derived from connected component analysis.
 */

class CCSemanticObject : public SemanticObject {
 public:
  CCSemanticObject(uint32_t _colorInt, const std::set<uint32_t>& _vertSet)
      : SemanticObject(), vertSet_(_vertSet), numSrcVerts_(vertSet_.size()) {
    // need to set index manually when mapping from color/colorInt is known
    index_ = ID_UNDEFINED;
    // sets both colorAsInt_ and updates color_ vector to match
    setColorAsInt(_colorInt);
  }

  void setIndex(int _index) { index_ = _index; }
  uint32_t getNumSrcVerts() const { return numSrcVerts_; }

  const std::set<uint32_t>& getVertSet() const { return vertSet_; }

 protected:
  // reference to vertex set used to build this CCSemantic object
  const std::set<uint32_t>& vertSet_;

  /**
   * @brief Number of verts in source mesh of CC used for this Semantic Object
   */
  uint32_t numSrcVerts_;

  ESP_SMART_POINTERS(CCSemanticObject)
};  // class CCSemanticObject

}  // namespace scene
}  // namespace esp

#endif  // ESP_SCENE_SEMANTICSCENE_H_
