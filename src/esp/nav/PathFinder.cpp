// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "PathFinder.h"
#include <algorithm>
#include <cstddef>
#include <numeric>
#include <stack>
#include <unordered_map>

#include <Magnum/Magnum.h>
#include <Magnum/Math/Functions.h>
#include <Magnum/Math/Vector3.h>

#include <Corrade/Containers/Optional.h>
#include <Corrade/Utility/Path.h>

#include <cstdio>

// NOLINTNEXTLINE
#define _USE_MATH_DEFINES
#include <cmath>
#include <limits>
#include <utility>

#include "esp/assets/MeshData.h"
#include "esp/core/Esp.h"
#include "esp/core/EspEigen.h"

#include "DetourCommon.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourNavMeshQuery.h"
#include "DetourNode.h"
#include "Recast.h"

#include <rapidjson/document.h>
#include "esp/core/Check.h"
#include "esp/io/Json.h"
#include "esp/io/JsonAllTypes.h"

namespace Mn = Magnum;
namespace Cr = Corrade;

namespace esp {
namespace nav {

bool operator==(const NavMeshSettings& a, const NavMeshSettings& b) {
#define CLOSE(name) (std::abs(a.name - b.name) < 1e-5f)
#define EQ(name) (a.name == b.name)

  return CLOSE(cellSize) && CLOSE(cellHeight) && CLOSE(agentHeight) &&
         CLOSE(agentRadius) && CLOSE(agentMaxClimb) && CLOSE(agentMaxSlope) &&
         CLOSE(regionMinSize) && CLOSE(regionMinSize) && CLOSE(edgeMaxLen) &&
         CLOSE(edgeMaxError) && CLOSE(vertsPerPoly) &&
         CLOSE(detailSampleDist) && CLOSE(detailSampleMaxError) &&
         EQ(filterLowHangingObstacles) && EQ(filterLedgeSpans) &&
         EQ(filterWalkableLowHeightSpans) && EQ(includeStaticObjects);

#undef CLOSE
#undef EQ
}

bool operator!=(const NavMeshSettings& a, const NavMeshSettings& b) {
  return !(a == b);
}

void NavMeshSettings::readFromJSON(const std::string& jsonFile) {
  if (!Corrade::Utility::Path::exists(jsonFile.data())) {
    ESP_ERROR() << "File" << jsonFile << "not found.";
    return;
  }
  try {
    auto newDoc = esp::io::parseJsonFile(jsonFile);

    esp::io::fromJsonValue(newDoc, *this);

  } catch (...) {
    ESP_ERROR() << "Failed to parse keyframes from" << jsonFile << ".";
  }
}

void NavMeshSettings::writeToJSON(const std::string& jsonFile) const {
  rapidjson::Document d(rapidjson::kObjectType);
  rapidjson::Document::AllocatorType& allocator = d.GetAllocator();
  auto jsonObj = esp::io::toJsonValue(*this, allocator);
  d.Swap(jsonObj);
  ESP_CHECK(!d.ObjectEmpty(), "Error writing JSON. Shouldn't happen.");

  const float maxDecimalPlaces = 7;
  auto ok = esp::io::writeJsonToFile(d, jsonFile, true, maxDecimalPlaces);
  ESP_CHECK(ok, "writeSavedKeyframesToFile: unable to write to " << jsonFile);
}

struct MultiGoalShortestPath::Impl {
  std::vector<Mn::Vector3> requestedEnds;

  std::vector<dtPolyRef> endRefs;
  //! Tracks whether an endpoint is valid or not as determined by setup to avoid
  //! extra work or issues later.
  std::vector<bool> endIsValid;
  std::vector<Mn::Vector3> pathEnds;

  std::vector<float> minTheoreticalDist;
  Mn::Vector3 prevRequestedStart;
};

MultiGoalShortestPath::MultiGoalShortestPath()
    : pimpl_{spimpl::make_unique_impl<Impl>()} {}

void MultiGoalShortestPath::setRequestedEnds(
    const std::vector<Mn::Vector3>& newEnds) {
  pimpl_->endRefs.clear();
  pimpl_->pathEnds.clear();
  pimpl_->requestedEnds = newEnds;

  pimpl_->minTheoreticalDist.assign(newEnds.size(), 0);
}

const std::vector<Mn::Vector3>& MultiGoalShortestPath::getRequestedEnds()
    const {
  return pimpl_->requestedEnds;
}

namespace {
template <typename T>
std::tuple<dtStatus, dtPolyRef, Mn::Vector3> projectToPoly(
    const T& pt,
    const dtNavMeshQuery* navQuery,
    const dtQueryFilter* filter) {
  // Defines size of the bounding box to search in for the nearest polygon.  If
  // there is no polygon inside the bounding box, the status is set to failure
  // and polyRef == 0
  constexpr float polyPickExt[3] = {2, 4, 2};  // [2 * dx, 2 * dy, 2 * dz]
  dtPolyRef polyRef = 0;
  // Initialize with all NANs at dtStatusSucceed(status) == true does NOT mean
  // that it found a point to project to..........
  Mn::Vector3 polyXYZ = Mn::Vector3(Mn::Constants::nan());
  dtStatus status = navQuery->findNearestPoly(pt.data(), polyPickExt, filter,
                                              &polyRef, polyXYZ.data());

  // So let's call it a failure if it didn't actually find a point....
  if (std::isnan(polyXYZ[0]))
    status = DT_FAILURE;

  return std::make_tuple(status, polyRef, polyXYZ);
}
}  // namespace

namespace impl {

typedef unsigned short int ushort;  // NOLINT

//! (flag & flag) operator wrapper for function pointers
inline ushort andFlag(ushort curFlags, ushort flag) {
  return curFlags & flag;
}
//! (flag | flag) operator wrapper for function pointers
inline ushort orFlag(ushort curFlags, ushort flag) {
  return curFlags | flag;
}

// Runs connected component analysis on the navmesh to figure out which polygons
// are connected This gives O(1) lookup for if a path between two polygons
// exists or not
// Takes O(npolys) to construct
class IslandSystem {
 public:
  IslandSystem(const dtNavMesh* navMesh, const dtQueryFilter* filter) {
    std::vector<Mn::Vector3> islandVerts;

    // Iterate over all tiles
    for (int iTile = 0; iTile < navMesh->getMaxTiles(); ++iTile) {
      const dtMeshTile* tile = navMesh->getTile(iTile);
      if (!tile)
        continue;

      // Iterate over all polygons in a tile
      for (int jPoly = 0; jPoly < tile->header->polyCount; ++jPoly) {
        // Get the polygon reference from the tile and polygon id
        dtPolyRef startRef = navMesh->encodePolyId(tile->salt, iTile, jPoly);

        // If the polygon ref is valid, and we haven't seen it yet,
        // start connected component analysis from this polygon
        if (navMesh->isValidPolyRef(startRef) &&
            (polyToIsland_.find(startRef) == polyToIsland_.end())) {
          uint32_t newIslandId = islandRadius_.size();
          expandFrom(navMesh, filter, newIslandId, startRef, islandVerts);

          // The radius is calculated as the max deviation from the mean for all
          // points in the island
          Mn::Vector3 centroid;
          for (auto& v : islandVerts) {
            centroid += v;
          }
          centroid /= islandVerts.size();

          float maxRadius = 0.0;
          for (auto& v : islandVerts) {
            maxRadius = std::max(maxRadius, (v - centroid).length());
          }

          islandRadius_.emplace_back(maxRadius);
        }
      }
    }
  }

  inline bool hasConnection(dtPolyRef startRef, dtPolyRef endRef) const {
    // If both polygons are on the same island, there must be a path between
    // them
    auto itStart = polyToIsland_.find(startRef);
    if (itStart == polyToIsland_.end())
      return false;

    auto itEnd = polyToIsland_.find(endRef);
    if (itEnd == polyToIsland_.end())
      return false;

    return itStart->second == itEnd->second;
  }

  //! check that island index is valid. indexOptional allows ID_UNDEFINED as
  //! valid.
  inline void assertValidIsland(int islandIndex, bool indexOptional = true) {
    if (indexOptional && islandIndex == ID_UNDEFINED) {
      return;
    }
    CORRADE_ASSERT(
        (islandIndex >= 0 && islandIndex < islandRadius_.size()),
        islandIndex << " not a valid index for this island system.", );
  }

  inline float islandRadius(int islandIndex) {
    assertValidIsland(islandIndex, /*indexOptional*/ false);
    return islandRadius_[islandIndex];
  }

  inline float polyIslandRadius(dtPolyRef ref) const {
    auto itRef = polyToIsland_.find(ref);
    if (itRef == polyToIsland_.end())
      return 0.0;

    return islandRadius_[itRef->second];
  }

  //! Get the area of an island.
  //! islandIndex=ID_UNDEFINED specifies the full NavMesh area.
  inline float getNavigableArea(int islandIndex) {
    assertValidIsland(islandIndex);
    return islandsToArea_[islandIndex];
  }

  inline int numIslands() const {
    // TODO: better way to track number of islands
    return islandRadius_.size();
  }

  /**
   * @brief Sets a specified poly flag for all polys specified by the
   * islandIndex.
   *
   * @param[in] navMesh The navmesh to operate on.
   * @param[in] flag The flag to set or clear.
   * @param[in] islandIndex Specify the island. islandIndex == ID_UNDEFINED
   * specifies all islands.
   * @param[in] setFlag If true, set the flag(currentFlags OR newFlag),
   * otherwise clear the flag(currentFlags AND ~newFlag).
   * @param[in] invert If true, set or clear the flag for all islands except the
   * specified one. Has no effect if islandIndex == ID_UNDEFINED.
   */
  inline void setPolyFlagForIsland(dtNavMesh* navMesh,
                                   ushort flag,
                                   int islandIndex = ID_UNDEFINED,
                                   bool setFlag = true,
                                   bool invert = false) {
    assertValidIsland(islandIndex);
    CORRADE_ASSERT(navMesh != nullptr, "invalid navMesh pointer", );
    std::vector<int> islands;

    if (islandIndex == ID_UNDEFINED) {
      // all islands
      islands.reserve(islandsToPolys_.size());
      for (auto& itr : islandsToPolys_) {
        islands.push_back(itr.first);
      }
    } else if (invert) {
      // all but a single island
      islands.reserve(islandsToPolys_.size());
      for (auto& itr : islandsToPolys_) {
        if (itr.first != islandIndex) {
          islands.push_back(itr.first);
        }
      }
    } else {
      // a single island
      islands.push_back(islandIndex);
    }

    // Pull this check and adjustment logic outside of the main loop
    ushort (*op)(ushort, ushort) = nullptr;
    op = setFlag ? orFlag : andFlag;
    ushort modFlag = setFlag ? flag : ~flag;

    // for each island
    for (int island : islands) {
      // for each poly
      for (auto& polyRef : islandsToPolys_[island]) {
        // get current flags
        ushort f = 0;
        navMesh->getPolyFlags(polyRef, &f);
        // set the modified flags
        navMesh->setPolyFlags(polyRef, op(f, modFlag));
      }
    }
  }

  /**
   * @brief Sets a specified poly flag for all polys specified by the
   * islandIndex within range of a given circle.
   *
   * @param[in] navMesh The navmesh to operate on.
   * @param[in] flag The flag to set or clear.
   * @param[in] circleCenter The center of the circle.
   * @param[in] radius The radius of the circle.
   * @param[in] islandIndex Specify the island. islandIndex == ID_UNDEFINED
   * specifies all islands.
   */
  inline void setPolyFlagForIslandCircle(dtNavMesh* navMesh,
                                         ushort flag,
                                         const Mn::Vector3& circleCenter,
                                         const float radius,
                                         int islandIndex = ID_UNDEFINED) {
    assertValidIsland(islandIndex);
    CORRADE_ASSERT(navMesh != nullptr, "invalid navMesh pointer", );
    std::vector<int> islands;

    float radSqr = radius * radius;

    // all islands
    islands.reserve(islandsToPolys_.size());
    for (auto& itr : islandsToPolys_) {
      islands.push_back(itr.first);
    }

    // for each island
    for (int island : islands) {
      // for each poly
      for (auto& polyRef : islandsToPolys_[island]) {
        // remove all off-island polys immediately
        if (islandIndex != ID_UNDEFINED && islandIndex != island) {
          // get current flags
          ushort f = 0;
          navMesh->getPolyFlags(polyRef, &f);
          // set the modified flags
          navMesh->setPolyFlags(polyRef, orFlag(f, flag));
          continue;
        }

        // poly is on island, check if it is outside of range
        const dtMeshTile* tile = nullptr;
        const dtPoly* poly = nullptr;
        navMesh->getTileAndPolyByRefUnsafe(polyRef, &tile, &poly);

        // check if this poly is within range of the circle
        bool inRange = false;
        for (int iVert = 0; iVert < poly->vertCount; ++iVert) {
          int nVert = (iVert + 1) % 3;
          float tseg = 0;
          float distSqr = dtDistancePtSegSqr2D(
              circleCenter.data(),
              &tile->verts[static_cast<size_t>(poly->verts[iVert]) * 3],
              &tile->verts[static_cast<size_t>(poly->verts[nVert]) * 3], tseg);
          if (distSqr < radSqr) {
            // skip this poly if any edge is within radius
            inRange = true;
            break;
          }
        }
        if (!inRange) {
          // get current flags
          ushort f = 0;
          navMesh->getPolyFlags(polyRef, &f);
          // set the modified flags
          navMesh->setPolyFlags(polyRef, orFlag(f, flag));
        }
      }
    }
  }

  // Some polygons have zero area for some reason.  When we navigate into a zero
  // area polygon, things crash.  So we find all zero area polygons and mark
  // them as disabled/not navigable.
  // Also compute the NavMesh areas for later query.
  void removeZeroAreaPolys(dtNavMesh* navMesh);

  //! return the island for a navmesh polygon
  inline int getPolyIsland(dtPolyRef polyRef) { return polyToIsland_[polyRef]; }

 private:
  //! map islands to area for quick query
  std::unordered_map<uint32_t, float> islandsToArea_;
  //! map islands to lists of polys for quick query and enumeration
  std::unordered_map<uint32_t, std::vector<dtPolyRef>> islandsToPolys_;
  //! map polygons to their island for quick look-up
  std::unordered_map<dtPolyRef, uint32_t> polyToIsland_;
  std::vector<float> islandRadius_;

  void expandFrom(const dtNavMesh* navMesh,
                  const dtQueryFilter* filter,
                  const uint32_t newIslandId,
                  const dtPolyRef& startRef,
                  std::vector<Mn::Vector3>& islandVerts) {
    islandsToPolys_[newIslandId].push_back(startRef);
    polyToIsland_.emplace(startRef, newIslandId);
    islandVerts.clear();

    // Force std::stack to be implemented via an std::vector as linked
    // lists are gross
    std::stack<dtPolyRef, std::vector<dtPolyRef>> stack;

    // Add the start ref to the stack
    stack.push(startRef);
    while (!stack.empty()) {
      dtPolyRef ref = stack.top();
      stack.pop();

      const dtMeshTile* tile = nullptr;
      const dtPoly* poly = nullptr;
      navMesh->getTileAndPolyByRefUnsafe(ref, &tile, &poly);

      for (int iVert = 0; iVert < poly->vertCount; ++iVert) {
        islandVerts.emplace_back(Mn::Vector3::from(
            &tile->verts[static_cast<size_t>(poly->verts[iVert]) * 3]));
      }

      // Iterate over all neighbours
      for (unsigned int iLink = poly->firstLink; iLink != DT_NULL_LINK;
           iLink = tile->links[iLink].next) {
        dtPolyRef neighbourRef = tile->links[iLink].ref;
        // If we've already visited this poly, skip it!
        if (polyToIsland_.find(neighbourRef) != polyToIsland_.end())
          continue;

        const dtMeshTile* neighbourTile = nullptr;
        const dtPoly* neighbourPoly = nullptr;
        navMesh->getTileAndPolyByRefUnsafe(neighbourRef, &neighbourTile,
                                           &neighbourPoly);

        // If a neighbour isn't walkable, don't add it
        if (!filter->passFilter(neighbourRef, neighbourTile, neighbourPoly))
          continue;

        polyToIsland_.emplace(neighbourRef, newIslandId);
        islandsToPolys_[newIslandId].push_back(neighbourRef);
        stack.push(neighbourRef);
      }
    }
  }
};
}  // namespace impl

struct PathFinder::Impl {
  Impl();
  ~Impl() = default;

  bool build(const NavMeshSettings& bs,
             const float* verts,
             int nverts,
             const int* tris,
             int ntris,
             const float* bmin,
             const float* bmax);
  bool build(const NavMeshSettings& bs, const esp::assets::MeshData& mesh);

  Mn::Vector3 getRandomNavigablePoint(int maxTries,
                                      int islandIndex /*= ID_UNDEFINED*/);
  Mn::Vector3 getRandomNavigablePointAroundSphere(
      const Mn::Vector3& circleCenter,
      float radius,
      int maxTries,
      int islandIndex /*= ID_UNDEFINED*/);
  Mn::Vector3 getRandomNavigablePointInCircle(
      const Mn::Vector3& circleCenter,
      float radius,
      int maxTries,
      int islandIndex /*= ID_UNDEFINED*/);

  bool findPath(ShortestPath& path);
  bool findPath(MultiGoalShortestPath& path);

  template <typename T>
  T tryStep(const T& start, const T& end, bool allowSliding);

  template <typename T>
  T snapPoint(const T& pt, int islandIndex = ID_UNDEFINED);

  template <typename T>
  int getIsland(const T& pt) const;

  bool loadNavMesh(const std::string& path);

  bool saveNavMesh(const std::string& path);

  bool isLoaded() const { return navMesh_ != nullptr; };

  float getNavigableArea(int islandIndex /*= ID_UNDEFINED*/) const {
    return islandSystem_->getNavigableArea(islandIndex);
  };

  int numIslands();

  void seed(uint32_t newSeed);

  float islandRadius(const Mn::Vector3& pt) const;

  float islandRadius(int islandIndex) const;

  float distanceToClosestObstacle(const Mn::Vector3& pt,
                                  float maxSearchRadius = 2.0f) const;
  HitRecord closestObstacleSurfacePoint(const Mn::Vector3& pt,
                                        float maxSearchRadius = 2.0f) const;

  bool isNavigable(const Mn::Vector3& pt, float maxYDelta = 0.5f) const;

  std::pair<Mn::Vector3, Mn::Vector3> bounds() const { return bounds_; };

  MatrixXb getTopDownView(float metersPerPixel, float height, float eps) const;

  MatrixXi getTopDownIslandView(float metersPerPixel,
                                float height,
                                float eps) const;

  assets::MeshData::ptr getNavMeshData(int islandIndex /*= ID_UNDEFINED*/);

  Cr::Containers::Optional<NavMeshSettings> getNavMeshSettings() const {
    return navMeshSettings_;
  }

 private:
  struct NavMeshDeleter {
    void operator()(dtNavMesh* mesh) { dtFreeNavMesh(mesh); }
  };
  struct NavQueryDeleter {
    void operator()(dtNavMeshQuery* query) { dtFreeNavMeshQuery(query); }
  };

  std::unique_ptr<dtNavMesh, NavMeshDeleter> navMesh_ = nullptr;
  std::unique_ptr<dtNavMeshQuery, NavQueryDeleter> navQuery_ = nullptr;
  std::unique_ptr<dtQueryFilter> filter_ = nullptr;
  std::unique_ptr<impl::IslandSystem> islandSystem_ = nullptr;

  //! Holds triangulated geom/topo. Generated when queried. Reset with
  //! navQuery_.
  std::unordered_map<int, assets::MeshData::ptr> islandMeshData_;
  Cr::Containers::Optional<NavMeshSettings> navMeshSettings_;

  std::pair<Mn::Vector3, Mn::Vector3> bounds_;

  bool initNavQuery();

  Cr::Containers::Optional<std::tuple<float, std::vector<Mn::Vector3>>>
  findPathInternal(const Mn::Vector3& start,
                   dtPolyRef startRef,
                   const Mn::Vector3& pathStart,
                   const Mn::Vector3& end,
                   dtPolyRef endRef,
                   const Mn::Vector3& pathEnd);

  bool findPathSetup(MultiGoalShortestPath& path,
                     dtPolyRef& startRef,
                     Mn::Vector3& pathStart);
};

namespace {
struct Workspace {
  rcHeightfield* solid = nullptr;
  unsigned char* triareas = nullptr;
  rcCompactHeightfield* chf = nullptr;
  rcContourSet* cset = nullptr;
  rcPolyMesh* pmesh = nullptr;
  rcPolyMeshDetail* dmesh = nullptr;

  ~Workspace() {
    rcFreeHeightField(solid);
    delete[] triareas;
    rcFreeCompactHeightfield(chf);
    rcFreeContourSet(cset);
    rcFreePolyMesh(pmesh);
    rcFreePolyMeshDetail(dmesh);
  }
};

enum PolyAreas { POLYAREA_GROUND, POLYAREA_DOOR };

enum PolyFlags {
  POLYFLAGS_WALK = 0x01,      // walkable
  POLYFLAGS_DOOR = 0x02,      // ability to move through doors
  POLYFLAGS_DISABLED = 0x04,  // disabled polygon
  POLYFLAGS_OFF_ISLAND =
      0x08,               // dynamically set to filter all but a specific island
  POLYFLAGS_ALL = 0xffff  // all abilities
};
}  // namespace

PathFinder::Impl::Impl() {
  filter_ = std::make_unique<dtQueryFilter>();
  filter_->setIncludeFlags(POLYFLAGS_WALK);
  filter_->setExcludeFlags(0);
}

bool PathFinder::Impl::build(const NavMeshSettings& bs,
                             const float* verts,
                             const int nverts,
                             const int* tris,
                             const int ntris,
                             const float* bmin,
                             const float* bmax) {
  Workspace ws;
  rcContext ctx;

  //
  // Step 1. Initialize build config.
  //

  // Init build configuration from GUI
  rcConfig cfg{};
  memset(&cfg, 0, sizeof(cfg));
  cfg.cs = bs.cellSize;
  cfg.ch = bs.cellHeight;
  cfg.walkableSlopeAngle = bs.agentMaxSlope;
  cfg.walkableHeight = static_cast<int>(ceilf(bs.agentHeight / cfg.ch));
  cfg.walkableClimb = static_cast<int>(floorf(bs.agentMaxClimb / cfg.ch));
  cfg.walkableRadius = static_cast<int>(ceilf(bs.agentRadius / cfg.cs));
  cfg.maxEdgeLen = static_cast<int>(bs.edgeMaxLen / bs.cellSize);
  cfg.maxSimplificationError = bs.edgeMaxError;
  cfg.minRegionArea =
      static_cast<int>(rcSqr(bs.regionMinSize));  // Note: area = size*size
  cfg.mergeRegionArea =
      static_cast<int>(rcSqr(bs.regionMergeSize));  // Note: area = size*size
  cfg.maxVertsPerPoly = static_cast<int>(bs.vertsPerPoly);
  cfg.detailSampleDist =
      bs.detailSampleDist < 0.9f ? 0 : bs.cellSize * bs.detailSampleDist;
  cfg.detailSampleMaxError = bs.cellHeight * bs.detailSampleMaxError;

  // Set the area where the navigation will be build.
  // Here the bounds of the input mesh are used, but the
  // area could be specified by an user defined box, etc.
  rcVcopy(cfg.bmin, bmin);
  rcVcopy(cfg.bmax, bmax);
  rcCalcGridSize(cfg.bmin, cfg.bmax, cfg.cs, &cfg.width, &cfg.height);
  ESP_DEBUG() << "Building navmesh with" << cfg.width << "x" << cfg.height
              << "cells";

  //
  // Step 2. Rasterize input polygon soup.
  //

  // Allocate voxel heightfield where we rasterize our input data to.
  ws.solid = rcAllocHeightfield();
  if (!ws.solid) {
    ESP_ERROR() << "Out of memory for heightfield allocation";
    return false;
  }
  if (!rcCreateHeightfield(&ctx, *ws.solid, cfg.width, cfg.height, cfg.bmin,
                           cfg.bmax, cfg.cs, cfg.ch)) {
    ESP_ERROR() << "Could not create solid heightfield";
    return false;
  }

  // Allocate array that can hold triangle area types.
  // If you have multiple meshes you need to process, allocate
  // and array which can hold the max number of triangles you need to process.
  ws.triareas = new unsigned char[ntris];
  if (!ws.triareas) {
    ESP_ERROR() << "Out of memory for triareas" << ntris;
    return false;
  }

  // Find triangles which are walkable based on their slope and rasterize them.
  // If your input data is multiple meshes, you can transform them here,
  // calculate the are type for each of the meshes and rasterize them.
  memset(ws.triareas, 0, ntris * sizeof(unsigned char));
  rcMarkWalkableTriangles(&ctx, cfg.walkableSlopeAngle, verts, nverts, tris,
                          ntris, ws.triareas);
  if (!rcRasterizeTriangles(&ctx, verts, nverts, tris, ws.triareas, ntris,
                            *ws.solid, cfg.walkableClimb)) {
    ESP_ERROR() << "Could not rasterize triangles.";
    return false;
  }

  //
  // Step 3. Filter walkables surfaces.
  //

  // Once all geoemtry is rasterized, we do initial pass of filtering to
  // remove unwanted overhangs caused by the conservative rasterization
  // as well as filter spans where the character cannot possibly stand.
  if (bs.filterLowHangingObstacles)
    rcFilterLowHangingWalkableObstacles(&ctx, cfg.walkableClimb, *ws.solid);
  if (bs.filterLedgeSpans)
    rcFilterLedgeSpans(&ctx, cfg.walkableHeight, cfg.walkableClimb, *ws.solid);
  if (bs.filterWalkableLowHeightSpans)
    rcFilterWalkableLowHeightSpans(&ctx, cfg.walkableHeight, *ws.solid);

  //
  // Step 4. Partition walkable surface to simple regions.
  //

  // Compact the heightfield so that it is faster to handle from now on.
  // This will result more cache coherent data as well as the neighbours
  // between walkable cells will be calculated.
  ws.chf = rcAllocCompactHeightfield();
  if (!ws.chf) {
    ESP_ERROR() << "Out of memory for compact heightfield";
    return false;
  }
  if (!rcBuildCompactHeightfield(&ctx, cfg.walkableHeight, cfg.walkableClimb,
                                 *ws.solid, *ws.chf)) {
    ESP_ERROR() << "Could not build compact heightfield";
    return false;
  }

  // Erode the walkable area by agent radius.
  if (!rcErodeWalkableArea(&ctx, cfg.walkableRadius, *ws.chf)) {
    ESP_ERROR() << "Could not erode walkable area";
    return false;
  }

  // // (Optional) Mark areas.
  // const ConvexVolume* vols = geom->getConvexVolumes();
  // for (int i  = 0; i < geom->getConvexVolumeCount(); ++i)
  //   rcMarkConvexPolyArea(ctx, vols[i].verts, vols[i].nverts, vols[i].hmin,
  //   vols[i].hmax, (unsigned char)vols[i].area, *ws.chf);

  // Partition the heightfield so that we can use simple algorithm later to
  // triangulate the walkable areas. There are 3 martitioning methods, each with
  // some pros and cons: 1) Watershed partitioning
  //   - the classic Recast partitioning
  //   - creates the nicest tessellation
  //   - usually slowest
  //   - partitions the heightfield into nice regions without holes or overlaps
  //   - the are some corner cases where this method creates produces holes and
  //   overlaps
  //      - holes may appear when a small obstacles is close to large open area
  //      (triangulation can handle this)
  //      - overlaps may occur if you have narrow spiral corridors (i.e stairs),
  //      this make triangulation to fail
  //   * generally the best choice if you precompute the nacmesh, use this if
  //   you have large open areas
  // 2) Monotone partioning
  //   - fastest
  //   - partitions the heightfield into regions without holes and overlaps
  //   (guaranteed)
  //   - creates long thin polygons, which sometimes causes paths with detours
  //   * use this if you want fast navmesh generation
  // 3) Layer partitoining
  //   - quite fast
  //   - partitions the heighfield into non-overlapping regions
  //   - relies on the triangulation code to cope with holes (thus slower than
  //   monotone partitioning)
  //   - produces better triangles than monotone partitioning
  //   - does not have the corner cases of watershed partitioning
  //   - can be slow and create a bit ugly tessellation (still better than
  //   monotone)
  //     if you have large open areas with small obstacles (not a problem if you
  //     use tiles)
  //   * good choice to use for tiled navmesh with medium and small sized tiles

  // Prepare for region partitioning, by calculating distance field along the
  // walkable surface.
  if (!rcBuildDistanceField(&ctx, *ws.chf)) {
    ESP_ERROR() << "Could not build distance field";
    return false;
  }
  // Partition the walkable surface into simple regions without holes.
  if (!rcBuildRegions(&ctx, *ws.chf, 0, cfg.minRegionArea,
                      cfg.mergeRegionArea)) {
    ESP_ERROR() << "Could not build watershed regions";
    return false;
  }
  // // Partition the walkable surface into simple regions without holes.
  // // Monotone partitioning does not need distancefield.
  // if (!rcBuildRegionsMonotone(ctx, *ws.chf, 0, cfg.minRegionArea,
  // cfg.mergeRegionArea))
  // // Partition the walkable surface into simple regions without holes.
  // if (!rcBuildLayerRegions(ctx, *ws.chf, 0, cfg.minRegionArea))

  //
  // Step 5. Trace and simplify region contours.
  //

  // Create contours.
  ws.cset = rcAllocContourSet();
  if (!ws.cset) {
    ESP_ERROR() << "Out of memory for contour set";
    return false;
  }
  if (!rcBuildContours(&ctx, *ws.chf, cfg.maxSimplificationError,
                       cfg.maxEdgeLen, *ws.cset)) {
    ESP_ERROR() << "Could not create contours";
    return false;
  }

  //
  // Step 6. Build polygons mesh from contours.
  //

  // Build polygon navmesh from the contours.
  ws.pmesh = rcAllocPolyMesh();
  if (!ws.pmesh) {
    ESP_ERROR() << "Out of memory for polymesh";
    return false;
  }
  if (!rcBuildPolyMesh(&ctx, *ws.cset, cfg.maxVertsPerPoly, *ws.pmesh)) {
    ESP_ERROR() << "Could not triangulate contours";
    return false;
  }

  //
  // Step 7. Create detail mesh which allows to access approximate height on
  // each polygon.
  //

  ws.dmesh = rcAllocPolyMeshDetail();
  if (!ws.dmesh) {
    ESP_ERROR() << "Out of memory for polymesh detail";
    return false;
  }

  if (!rcBuildPolyMeshDetail(&ctx, *ws.pmesh, *ws.chf, cfg.detailSampleDist,
                             cfg.detailSampleMaxError, *ws.dmesh)) {
    ESP_ERROR() << "Could not build detail mesh";
    return false;
  }

  // At this point the navigation mesh data is ready, you can access it from
  // ws.pmesh. See duDebugDrawPolyMesh or dtCreateNavMeshData as examples how to
  // access the data.

  //
  // (Optional) Step 8. Create Detour data from Recast poly mesh.
  //

  // The GUI may allow more max points per polygon than Detour can handle.
  // Only build the detour navmesh if we do not exceed the limit.
  if (cfg.maxVertsPerPoly <= DT_VERTS_PER_POLYGON) {
    unsigned char* navData = nullptr;
    int navDataSize = 0;

    // Update poly flags from areas.
    for (int i = 0; i < ws.pmesh->npolys; ++i) {
      if (ws.pmesh->areas[i] == RC_WALKABLE_AREA) {
        ws.pmesh->areas[i] = POLYAREA_GROUND;
      }
      if (ws.pmesh->areas[i] == POLYAREA_GROUND) {
        ws.pmesh->flags[i] = POLYFLAGS_WALK;
      } else if (ws.pmesh->areas[i] == POLYAREA_DOOR) {
        ws.pmesh->flags[i] = POLYFLAGS_WALK | POLYFLAGS_DOOR;
      }
    }

    dtNavMeshCreateParams params{};
    memset(&params, 0, sizeof(params));
    params.verts = ws.pmesh->verts;
    params.vertCount = ws.pmesh->nverts;
    params.polys = ws.pmesh->polys;
    params.polyAreas = ws.pmesh->areas;
    params.polyFlags = ws.pmesh->flags;
    params.polyCount = ws.pmesh->npolys;
    params.nvp = ws.pmesh->nvp;
    params.detailMeshes = ws.dmesh->meshes;
    params.detailVerts = ws.dmesh->verts;
    params.detailVertsCount = ws.dmesh->nverts;
    params.detailTris = ws.dmesh->tris;
    params.detailTriCount = ws.dmesh->ntris;
    // params.offMeshConVerts = geom->getOffMeshConnectionVerts();
    // params.offMeshConRad = geom->getOffMeshConnectionRads();
    // params.offMeshConDir = geom->getOffMeshConnectionDirs();
    // params.offMeshConAreas = geom->getOffMeshConnectionAreas();
    // params.offMeshConFlags = geom->getOffMeshConnectionFlags();
    // params.offMeshConUserID = geom->getOffMeshConnectionId();
    // params.offMeshConCount = geom->getOffMeshConnectionCount();
    params.walkableHeight = bs.agentHeight;
    params.walkableRadius = bs.agentRadius;
    params.walkableClimb = bs.agentMaxClimb;
    rcVcopy(params.bmin, ws.pmesh->bmin);
    rcVcopy(params.bmax, ws.pmesh->bmax);
    params.cs = cfg.cs;
    params.ch = cfg.ch;
    params.buildBvTree = true;

    if (!dtCreateNavMeshData(&params, &navData, &navDataSize)) {
      ESP_ERROR() << "Could not build Detour navmesh";
      return false;
    }

    navMesh_.reset(dtAllocNavMesh());
    if (!navMesh_) {
      dtFree(navData);
      ESP_ERROR() << "Could not allocate Detour navmesh";
      return false;
    }

    dtStatus status = 0;
    status = navMesh_->init(navData, navDataSize, DT_TILE_FREE_DATA);
    if (dtStatusFailed(status)) {
      dtFree(navData);
      ESP_ERROR() << "Could not init Detour navmesh";
      return false;
    }
    if (!initNavQuery()) {
      return false;
    }
    navMeshSettings_ = {bs};
  } else {
    ESP_ERROR() << "cfg.maxVertsPerPoly(" << cfg.maxVertsPerPoly
                << ") > DT_VERTS_PER_POLYGON(" << DT_VERTS_PER_POLYGON
                << "), so cannot build the Detour NavMesh. Aborting NavMesh "
                   "construction.";
    return false;
  }

  bounds_ = std::make_pair(Mn::Vector3::from(bmin), Mn::Vector3::from(bmax));

  ESP_DEBUG() << "Created navmesh with" << ws.pmesh->nverts << "vertices"
              << ws.pmesh->npolys << "polygons";

  return true;
}

bool PathFinder::Impl::initNavQuery() {
  // if we are reinitializing the NavQuery, then also reset the MeshData
  islandMeshData_.clear();

  navQuery_.reset(dtAllocNavMeshQuery());
  dtStatus status = navQuery_->init(navMesh_.get(), 2048);
  if (dtStatusFailed(status)) {
    ESP_ERROR() << "Could not init Detour navmesh query";
    return false;
  }

  islandSystem_ =
      std::make_unique<impl::IslandSystem>(navMesh_.get(), filter_.get());

  // Added as we also need to remove these on navmesh recomputation
  islandSystem_->removeZeroAreaPolys(navMesh_.get());

  return true;
}

bool PathFinder::Impl::build(const NavMeshSettings& bs,
                             const esp::assets::MeshData& mesh) {
  const int numVerts = mesh.vbo.size();
  const int numIndices = mesh.ibo.size();
  const float mf = std::numeric_limits<float>::max();
  Mn::Vector3 bmin(mf, mf, mf);
  Mn::Vector3 bmax(-mf, -mf, -mf);

  for (int i = 0; i < numVerts; ++i) {
    const Mn::Vector3& p = mesh.vbo[i];
    bmin = Mn::Math::min(bmin, p);
    bmax = Mn::Math::max(bmax, p);
  }

  int* indices = new int[numIndices];
  for (int i = 0; i < numIndices; ++i) {
    indices[i] = static_cast<int>(mesh.ibo[i]);
  }

  const bool success = build(bs, mesh.vbo[0].data(), numVerts, indices,
                             numIndices / 3, bmin.data(), bmax.data());
  delete[] indices;
  return success;
}

namespace {
const int NAVMESHSET_MAGIC = 'M' << 24 | 'S' << 16 | 'E' << 8 | 'T';  //'MSET';
const int NAVMESHSET_VERSION = 2;

struct NavMeshSetHeader {
  int magic;
  int version;
  int numTiles;
  dtNavMeshParams params;
};

struct NavMeshTileHeader {
  dtTileRef tileRef;
  int dataSize;
};

struct Triangle {
  std::vector<Mn::Vector3> v;
  Triangle() { v.resize(3); }
};

std::vector<Triangle> getPolygonTriangles(const dtPoly* poly,
                                          const dtMeshTile* tile) {
  // Code to iterate over triangles from here:
  // https://github.com/recastnavigation/recastnavigation/blob/57610fa6ef31b39020231906f8c5d40eaa8294ae/Detour/Source/DetourNavMesh.cpp#L684
  const std::ptrdiff_t ip = poly - tile->polys;
  const dtPolyDetail* pd = &tile->detailMeshes[ip];
  std::vector<Triangle> triangles(pd->triCount);

  for (int j = 0; j < pd->triCount; ++j) {
    const unsigned char* t =
        &tile->detailTris[static_cast<size_t>((pd->triBase + j)) * 4];
    // const float* v[3];
    for (int k = 0; k < 3; ++k) {
      if (t[k] < poly->vertCount)
        triangles[j].v[k] = Mn::Vector3::from(
            (&tile->verts[static_cast<size_t>(poly->verts[t[k]]) * 3]));
      else
        triangles[j].v[k] = Mn::Vector3::from(
            (&tile->detailVerts[static_cast<size_t>(
                                    (pd->vertBase + (t[k] - poly->vertCount))) *
                                3]));
    }
  }

  return triangles;
}

// Calculate the area of a polygon by iterating over the triangles in the detail
// mesh and computing their area
float polyArea(const dtPoly* poly, const dtMeshTile* tile) {
  std::vector<Triangle> triangles = getPolygonTriangles(poly, tile);

  float area = 0;
  for (auto& tri : triangles) {
    const Mn::Vector3 w1 = tri.v[1] - tri.v[0];
    const Mn::Vector3 w2 = tri.v[2] - tri.v[1];
    area += 0.5f * Mn::Math::cross(w1, w2).length();
  }

  return area;
}
}  // namespace

// Some polygons have zero area for some reason.  When we navigate into a zero
// area polygon, things crash.  So we find all zero area polygons and mark
// them as disabled/not navigable.
// Also compute the NavMesh areas for later query.
void impl::IslandSystem::removeZeroAreaPolys(dtNavMesh* navMesh) {
  islandsToArea_ = std::unordered_map<uint32_t, float>();
  islandsToArea_.reserve(islandsToPolys_.size());
  // initialize the area cache.
  for (auto& itr : islandsToPolys_) {
    islandsToArea_[itr.first] = 0.0;
  }
  // Iterate over all tiles
  for (int iTile = 0; iTile < navMesh->getMaxTiles(); ++iTile) {
    const dtMeshTile* tile =
        const_cast<const dtNavMesh*>(navMesh)->getTile(iTile);
    if (!tile)
      continue;

    // Iterate over all polygons in a tile
    for (int jPoly = 0; jPoly < tile->header->polyCount; ++jPoly) {
      // Get the polygon reference from the tile and polygon id
      dtPolyRef polyRef = navMesh->encodePolyId(tile->salt, iTile, jPoly);
      const dtPoly* poly = nullptr;
      const dtMeshTile* tmp = nullptr;
      navMesh->getTileAndPolyByRefUnsafe(polyRef, &tmp, &poly);

      CORRADE_INTERNAL_ASSERT(poly != nullptr);
      CORRADE_INTERNAL_ASSERT(tmp != nullptr);

      float polygonArea = polyArea(poly, tile);
      if (polygonArea < 1e-5f) {
        navMesh->setPolyFlags(polyRef, POLYFLAGS_DISABLED);
      } else if ((poly->flags & POLYFLAGS_WALK) != 0) {
        islandsToArea_[polyToIsland_[polyRef]] += polygonArea;
      }
    }
  }

  // total of all island areas
  float totalArea = 0;
  for (auto& itr : islandsToArea_) {
    totalArea += itr.second;
  }
  islandsToArea_[ID_UNDEFINED] = totalArea;
}

int PathFinder::Impl::numIslands() {
  return islandSystem_->numIslands();
}

bool PathFinder::Impl::loadNavMesh(const std::string& path) {
  FILE* fp = fopen(path.c_str(), "rb");
  if (!fp)
    return false;

  // Read header.
  NavMeshSetHeader header{};
  size_t readLen = fread(&header, sizeof(NavMeshSetHeader), 1, fp);
  if (readLen != 1) {
    fclose(fp);
    return false;
  }
  if (header.magic != NAVMESHSET_MAGIC) {
    fclose(fp);
    return false;
  }
  if (header.version < 1 || header.version > NAVMESHSET_VERSION) {
    fclose(fp);
    return false;
  }

  navMeshSettings_ = {NavMeshSettings{}};
  if (header.version >= 2) {
    fread(&(*navMeshSettings_), sizeof(NavMeshSettings), 1, fp);
  } else {
    ESP_DEBUG()
        << "NavMeshSettings aren't present, guessing that they are the default";
  }

  Mn::Vector3 bmin, bmax;

  dtNavMesh* mesh = dtAllocNavMesh();
  if (!mesh) {
    fclose(fp);
    return false;
  }
  dtStatus status = mesh->init(&header.params);
  if (dtStatusFailed(status)) {
    fclose(fp);
    return false;
  }

  // Read tiles.
  for (int i = 0; i < header.numTiles; ++i) {
    NavMeshTileHeader tileHeader{};
    readLen = fread(&tileHeader, sizeof(tileHeader), 1, fp);
    if (readLen != 1) {
      fclose(fp);
      return false;
    }

    if ((tileHeader.tileRef == 0u) || (tileHeader.dataSize == 0))
      break;

    unsigned char* data = static_cast<unsigned char*>(
        dtAlloc(tileHeader.dataSize, DT_ALLOC_PERM));
    if (!data)
      break;
    memset(data, 0, tileHeader.dataSize);
    readLen = fread(data, tileHeader.dataSize, 1, fp);
    if (readLen != 1) {
      dtFree(data);
      fclose(fp);
      return false;
    }

    mesh->addTile(data, tileHeader.dataSize, DT_TILE_FREE_DATA,
                  tileHeader.tileRef, nullptr);
    const dtMeshTile* tile = mesh->getTileByRef(tileHeader.tileRef);
    if (i == 0) {
      bmin = Mn::Vector3::from(tile->header->bmin);
      bmax = Mn::Vector3::from(tile->header->bmax);
    } else {
      bmin = Mn::Math::min(bmin, Mn::Vector3::from(tile->header->bmin));
      bmax = Mn::Math::max(bmax, Mn::Vector3::from(tile->header->bmax));
    }
  }

  fclose(fp);

  navMesh_.reset(mesh);
  bounds_ = std::make_pair(bmin, bmax);

  return initNavQuery();
}

bool PathFinder::Impl::saveNavMesh(const std::string& path) {
  const dtNavMesh* navMesh = navMesh_.get();
  if (!navMesh)
    return false;

  FILE* fp = fopen(path.c_str(), "wb");
  if (!fp)
    return false;

  // Store header.
  NavMeshSetHeader header{};
  header.magic = NAVMESHSET_MAGIC;
  header.version = NAVMESHSET_VERSION;
  header.numTiles = 0;
  for (int i = 0; i < navMesh->getMaxTiles(); ++i) {
    const dtMeshTile* tile = navMesh->getTile(i);
    if (!tile || !tile->header || (tile->dataSize == 0))
      continue;
    ++header.numTiles;
  }
  memcpy(&header.params, navMesh->getParams(), sizeof(dtNavMeshParams));
  fwrite(&header, sizeof(NavMeshSetHeader), 1, fp);
  if (!navMeshSettings_) {
    ESP_ERROR() << "NavMeshSettings weren't set. Either build or load a "
                   "navmesh before saving";
    return false;
  }
  fwrite(&(*navMeshSettings_), sizeof(NavMeshSettings), 1, fp);

  // Store tiles.
  for (int i = 0; i < navMesh->getMaxTiles(); ++i) {
    const dtMeshTile* tile = navMesh->getTile(i);
    if (!tile || !tile->header || (tile->dataSize == 0))
      continue;

    NavMeshTileHeader tileHeader{};
    tileHeader.tileRef = navMesh->getTileRef(tile);
    tileHeader.dataSize = tile->dataSize;
    fwrite(&tileHeader, sizeof(tileHeader), 1, fp);

    fwrite(tile->data, tile->dataSize, 1, fp);
  }

  fclose(fp);

  return true;
}

void PathFinder::Impl::seed(uint32_t newSeed) {
  // TODO: this should be using core::Random instead, but passing function
  // to navQuery_->findRandomPoint needs to be figured out first
  srand(newSeed);
}

// Returns a random number [0..1]
static float frand() {
  return static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
}

Mn::Vector3 PathFinder::Impl::getRandomNavigablePoint(
    const int maxTries /*= 10*/,
    int islandIndex /*= ID_UNDEFINED*/) {
  islandSystem_->assertValidIsland(islandIndex);
  if (getNavigableArea(islandIndex) <= 0.0f)
    throw std::runtime_error(
        "NavMesh has no navigable area, this indicates an issue with the "
        "NavMesh");

  // If this query should be island specific
  if (islandIndex != ID_UNDEFINED) {
    // set the poly flag to identify polys not on the target island
    islandSystem_->setPolyFlagForIsland(
        navMesh_.get(), PolyFlags::POLYFLAGS_OFF_ISLAND, islandIndex,
        /*setFlag=*/true, /*invert=*/true);
    filter_->setExcludeFlags(filter_->getExcludeFlags() |
                             PolyFlags::POLYFLAGS_OFF_ISLAND);
  }

  Mn::Vector3 pt;
  int i = 0;
  for (i = 0; i < maxTries; ++i) {
    dtPolyRef ref = 0;
    dtStatus status =
        navQuery_->findRandomPoint(filter_.get(), frand, &ref, pt.data());
    if (dtStatusSucceed(status))
      break;
  }

  // Clean up if this query was island specific
  if (islandIndex != ID_UNDEFINED) {
    // reset the poly flag identifing polys off the target island
    islandSystem_->setPolyFlagForIsland(
        navMesh_.get(), PolyFlags::POLYFLAGS_OFF_ISLAND, islandIndex,
        /*setFlag=*/false, /*invert=*/true);
    filter_->setExcludeFlags(filter_->getExcludeFlags() &
                             ~PolyFlags::POLYFLAGS_OFF_ISLAND);
  }

  if (i == maxTries) {
    ESP_ERROR() << "Failed to getRandomNavigablePoint.  Try increasing max "
                   "tries if the navmesh is fine but just hard to sample from";
    return Mn::Vector3(Mn::Constants::nan());
  }
  return pt;
}

Mn::Vector3 PathFinder::Impl::getRandomNavigablePointInCircle(
    const Mn::Vector3& circleCenter,
    const float radius,
    const int maxTries,
    int islandIndex) {
  float radSqr = radius * radius;

  islandSystem_->assertValidIsland(islandIndex);
  if (getNavigableArea(islandIndex) <= 0.0f)
    throw std::runtime_error(
        "NavMesh has no navigable area, this indicates an issue with the "
        "NavMesh");

  // set the poly flag to identify polys not on the target island
  islandSystem_->setPolyFlagForIslandCircle(navMesh_.get(),
                                            PolyFlags::POLYFLAGS_OFF_ISLAND,
                                            circleCenter, radius, islandIndex);
  filter_->setExcludeFlags(filter_->getExcludeFlags() |
                           PolyFlags::POLYFLAGS_OFF_ISLAND);

  Mn::Vector3 pt;
  int i = 0;
  for (i = 0; i < maxTries; ++i) {
    dtPolyRef ref = 0;
    dtStatus status =
        navQuery_->findRandomPoint(filter_.get(), frand, &ref, pt.data());
    if (dtStatusSucceed(status)) {
      float xd = circleCenter[0] - pt[0];
      float yd = circleCenter[2] - pt[2];
      float d2 = xd * xd + yd * yd;
      if (d2 < radSqr) {
        break;
      }
    }
  }

  // reset the poly flag identifying polys off the target island
  islandSystem_->setPolyFlagForIsland(
      navMesh_.get(), PolyFlags::POLYFLAGS_OFF_ISLAND, ID_UNDEFINED,
      /*setFlag=*/false, /*invert=*/true);
  filter_->setExcludeFlags(filter_->getExcludeFlags() &
                           ~PolyFlags::POLYFLAGS_OFF_ISLAND);

  if (i == maxTries) {
    ESP_ERROR() << "Failed to getRandomNavigablePoint.  Try increasing max "
                   "tries if the navmesh is fine but just hard to sample from";
    return Mn::Vector3(Mn::Constants::nan());
  }
  return pt;
}

Mn::Vector3 PathFinder::Impl::getRandomNavigablePointAroundSphere(
    const Mn::Vector3& circleCenter,
    const float radius,
    const int maxTries,
    int islandIndex) {
  islandSystem_->assertValidIsland(islandIndex);
  if (getNavigableArea(islandIndex) <= 0.0f)
    throw std::runtime_error(
        "NavMesh has no navigable area, this indicates an issue with the "
        "NavMesh");

  // If this query should be island specific
  if (islandIndex != ID_UNDEFINED) {
    // set the poly flag to identify polys not on the target island
    islandSystem_->setPolyFlagForIsland(
        navMesh_.get(), PolyFlags::POLYFLAGS_OFF_ISLAND, islandIndex,
        /*setFlag=*/true, /*invert=*/true);
    filter_->setExcludeFlags(filter_->getExcludeFlags() |
                             PolyFlags::POLYFLAGS_OFF_ISLAND);
  }

  Mn::Vector3 pt = Mn::Vector3(Mn::Constants::nan());
  dtPolyRef start_ref = 0;  // ID to start our search
  dtStatus status = navQuery_->findNearestPoly(
      circleCenter.data(), Mn::Vector3{radius, radius, radius}.data(),
      filter_.get(), &start_ref, pt.data());

  // cache and handle later to unify required clean-up
  bool failedAndAborting = (!dtStatusSucceed(status) || std::isnan(pt[0]));

  int i = 0;
  if (!failedAndAborting) {
    for (; i < maxTries; ++i) {
      dtPolyRef rand_ref = 0;
      status = navQuery_->findRandomPointAroundCircle(
          start_ref, circleCenter.data(), radius, filter_.get(), frand,
          &rand_ref, pt.data());
      if (dtStatusSucceed(status) &&
          (pt - circleCenter).dot() <= radius * radius) {
        break;
      }
    }
  }
  // Clean up if this query was island specific
  if (islandIndex != ID_UNDEFINED) {
    // reset the poly flag identifing polys off the target island
    islandSystem_->setPolyFlagForIsland(
        navMesh_.get(), PolyFlags::POLYFLAGS_OFF_ISLAND, islandIndex,
        /*setFlag=*/false, /*invert=*/true);
    filter_->setExcludeFlags(filter_->getExcludeFlags() &
                             ~PolyFlags::POLYFLAGS_OFF_ISLAND);
  }
  if (failedAndAborting) {
    ESP_ERROR()
        << "Failed to getRandomNavigablePoint. No polygon found within radius";
    return Mn::Vector3(Mn::Constants::nan());
  }
  if (i == maxTries) {
    ESP_ERROR() << "Failed to getRandomNavigablePoint.  Try increasing max "
                   "tries if the navmesh is fine but just hard to sample from";
    return Mn::Vector3(Mn::Constants::nan());
  }
  return pt;
}

namespace {
float pathLength(const std::vector<Mn::Vector3>& points) {
  CORRADE_INTERNAL_ASSERT(points.size() > 0);

  float length = 0;
  const Mn::Vector3* previousPoint = &points[0];
  for (const auto& pt : points) {
    length += (*previousPoint - pt).length();
    previousPoint = &pt;
  }

  return length;
}
}  // namespace

bool PathFinder::Impl::findPath(ShortestPath& path) {
  MultiGoalShortestPath tmp;
  tmp.requestedStart = path.requestedStart;
  tmp.setRequestedEnds({path.requestedEnd});

  bool status = findPath(tmp);

  path.geodesicDistance = tmp.geodesicDistance;
  path.points = std::move(tmp.points);
  return status;
}

Cr::Containers::Optional<std::tuple<float, std::vector<Mn::Vector3>>>
PathFinder::Impl::findPathInternal(const Mn::Vector3& start,
                                   dtPolyRef startRef,
                                   const Mn::Vector3& pathStart,
                                   const Mn::Vector3& end,
                                   dtPolyRef endRef,
                                   const Mn::Vector3& pathEnd) {
  // check if trivial path (start is same as end) and early return
  if (pathStart == pathEnd) {
    return std::make_tuple(0.0f, std::vector<Mn::Vector3>{pathStart, pathEnd});
  }

  // Check if there is a path between the start and any of the ends
  if (!islandSystem_->hasConnection(startRef, endRef)) {
    return Cr::Containers::NullOpt;
  }

  static const int MAX_POLYS = 256;
  dtPolyRef polys[MAX_POLYS];

  int numPolys = 0;
  dtStatus status =
      navQuery_->findPath(startRef, endRef, pathStart.data(), pathEnd.data(),
                          filter_.get(), polys, &numPolys, MAX_POLYS);
  if (status != DT_SUCCESS || numPolys == 0) {
    return Cr::Containers::NullOpt;
  }

  int numPoints = 0;
  std::vector<Mn::Vector3> points(MAX_POLYS);
  status = navQuery_->findStraightPath(start.data(), end.data(), polys,
                                       numPolys, points[0].data(), nullptr,
                                       nullptr, &numPoints, MAX_POLYS);
  if (status != DT_SUCCESS || numPoints == 0) {
    return Corrade::Containers::NullOpt;
  }

  points.resize(numPoints);

  const float length = pathLength(points);

  return std::make_tuple(length, std::move(points));
}

bool PathFinder::Impl::findPathSetup(MultiGoalShortestPath& path,
                                     dtPolyRef& startRef,
                                     Mn::Vector3& pathStart) {
  path.geodesicDistance = std::numeric_limits<float>::infinity();
  path.closestEndPointIndex = -1;
  path.points.clear();

  // find nearest polys and path
  dtStatus status = 0;
  std::tie(status, startRef, pathStart) =
      projectToPoly(path.requestedStart, navQuery_.get(), filter_.get());

  if (status != DT_SUCCESS || startRef == 0) {
    return false;
  }

  if (!path.pimpl_->endRefs.empty())
    return true;

  int numValidPoints = 0;
  for (const auto& rqEnd : path.getRequestedEnds()) {
    dtPolyRef endRef = 0;
    Mn::Vector3 pathEnd;
    std::tie(status, endRef, pathEnd) =
        projectToPoly(rqEnd, navQuery_.get(), filter_.get());

    if (status != DT_SUCCESS || endRef == 0) {
      path.pimpl_->endIsValid.emplace_back(false);
      ESP_DEBUG() << "Can't project end-point to navmesh, skipping: " << rqEnd;
    } else {
      path.pimpl_->endIsValid.emplace_back(true);
      numValidPoints++;
    }

    path.pimpl_->endRefs.emplace_back(endRef);
    path.pimpl_->pathEnds.emplace_back(pathEnd);
  }
  if (numValidPoints == 0) {
    ESP_DEBUG() << "Early abort, can't project any points to navmesh.";
    return false;
  }

  return true;
}

bool PathFinder::Impl::findPath(MultiGoalShortestPath& path) {
  dtPolyRef startRef = 0;
  Mn::Vector3 pathStart;
  if (!findPathSetup(path, startRef, pathStart))
    return false;

  if (path.pimpl_->requestedEnds.size() > 1) {
    // Bound the minimum distance any point could be from the start by either
    // how close it use to be minus how much we moved from the last search point
    // or just the L2 distance.

    ShortestPath prevPath;
    prevPath.requestedStart = path.requestedStart;
    prevPath.requestedEnd = path.pimpl_->prevRequestedStart;
    findPath(prevPath);
    const float movedAmount = prevPath.geodesicDistance;

    for (std::size_t i = 0; i < path.pimpl_->requestedEnds.size(); ++i) {
      path.pimpl_->minTheoreticalDist[i] = std::max(
          path.pimpl_->minTheoreticalDist[i] - movedAmount,
          (path.pimpl_->requestedEnds[i] - path.requestedStart).length());
    }

    path.pimpl_->prevRequestedStart = path.requestedStart;
  }

  // Explore possible goal points by their minimum theoretical distance.
  std::vector<size_t> ordering(path.pimpl_->requestedEnds.size());
  std::iota(ordering.begin(), ordering.end(), 0);
  std::sort(ordering.begin(), ordering.end(),
            [&path](const size_t a, const size_t b) -> bool {
              return path.pimpl_->minTheoreticalDist[a] <
                     path.pimpl_->minTheoreticalDist[b];
            });

  for (size_t i : ordering) {
    if (!path.pimpl_->endIsValid[i])
      continue;

    if (path.pimpl_->minTheoreticalDist[i] > path.geodesicDistance)
      continue;

    const Cr::Containers::Optional<std::tuple<float, std::vector<Mn::Vector3>>>
        findResult =
            findPathInternal(path.requestedStart, startRef, pathStart,
                             path.pimpl_->requestedEnds[i],
                             path.pimpl_->endRefs[i], path.pimpl_->pathEnds[i]);

    if (findResult && std::get<0>(*findResult) < path.geodesicDistance) {
      path.pimpl_->minTheoreticalDist[i] = std::get<0>(*findResult);
      path.geodesicDistance = std::get<0>(*findResult);
      path.points = std::get<1>(*findResult);
      path.closestEndPointIndex = i;
    }
  }

  return path.geodesicDistance < std::numeric_limits<float>::infinity();
}

template <typename T>
T PathFinder::Impl::tryStep(const T& start, const T& end, bool allowSliding) {
  static const int MAX_POLYS = 256;
  dtPolyRef polys[MAX_POLYS];

  dtStatus startStatus = 0, endStatus = 0;
  dtPolyRef startRef = 0, endRef = 0;
  Mn::Vector3 pathStart;
  std::tie(startStatus, startRef, pathStart) =
      projectToPoly(start, navQuery_.get(), filter_.get());
  std::tie(endStatus, endRef, std::ignore) =
      projectToPoly(end, navQuery_.get(), filter_.get());

  if (dtStatusFailed(startStatus) || dtStatusFailed(endStatus)) {
    return start;
  }

  if (not islandSystem_->hasConnection(startRef, endRef)) {
    return start;
  }

  Mn::Vector3 endPoint;
  int numPolys = 0;
  navQuery_->moveAlongSurface(startRef, pathStart.data(), end.data(),
                              filter_.get(), endPoint.data(), polys, &numPolys,
                              MAX_POLYS, allowSliding);
  // If there isn't any possible path between start and end, just return
  // start, that is cleanest
  if (numPolys == 0) {
    return start;
  }

  // According to recast's code
  // (https://github.com/recastnavigation/recastnavigation/blob/master/Detour/Source/DetourNavMeshQuery.cpp#L2006-L2007),
  // the endPoint is not guaranteed to be actually on the surface of the
  // navmesh, it seems to be in 99.9% of cases for us, but there are some
  // extreme edge cases where it won't be, so explicitly get the height of the
  // surface at the endPoint and set its height to that.
  // Note, this will never fail as endPoint is always within in the poly
  // polys[numPolys - 1]
  navQuery_->getPolyHeight(polys[numPolys - 1], endPoint.data(), &endPoint[1]);

  // Hack to deal with infinitely thin walls in recast allowing you to
  // transition between two different connected components
  // First check to see if the endPoint as returned by `moveAlongSurface`
  // is in the same connected component as the startRef according to
  // findNearestPoly
  std::tie(std::ignore, endRef, std::ignore) =
      projectToPoly(endPoint, navQuery_.get(), filter_.get());
  if (!this->islandSystem_->hasConnection(startRef, endRef)) {
    // There isn't a connection!  This happens when endPoint is on an edge
    // shared between two different connected components (aka infinitely thin
    // walls) The way to deal with this is to nudge the point into the polygon
    // we want it to be 'moveAlongSurface' tells us which polygon we want
    // endPoint to be in through the polys list
    const dtMeshTile* tile = nullptr;
    const dtPoly* poly = nullptr;
    navMesh_->getTileAndPolyByRefUnsafe(polys[numPolys - 1], &tile, &poly);

    // Calculate the center of the polygon we want the points to be in
    Mn::Vector3 polyCenter;
    for (int iVert = 0; iVert < poly->vertCount; ++iVert) {
      // auto idx = poly->verts[iVert];
      polyCenter += Mn::Vector3::from(
          &tile->verts[static_cast<size_t>(poly->verts[iVert]) * 3]);
    }
    polyCenter /= poly->vertCount;

    constexpr float nudgeDistance = 1e-4;  // 0.1mm
    const Mn::Vector3 nudgeDir = (polyCenter - endPoint).normalized();
    // And nudge the point towards the center by a little tiny bit :)
    endPoint = endPoint + nudgeDistance * nudgeDir;
  }

  return T{endPoint};
}

template <typename T>
T PathFinder::Impl::snapPoint(const T& pt, int islandIndex /*=ID_UNDEFINED*/) {
  islandSystem_->assertValidIsland(islandIndex);

  // If this query should be island specific
  if (islandIndex != ID_UNDEFINED) {
    // set the poly flag to identify polys not on the target island
    islandSystem_->setPolyFlagForIsland(
        navMesh_.get(), PolyFlags::POLYFLAGS_OFF_ISLAND, islandIndex,
        /*setFlag=*/true, /*invert=*/true);
    filter_->setExcludeFlags(filter_->getExcludeFlags() |
                             PolyFlags::POLYFLAGS_OFF_ISLAND);
  }

  dtStatus status = 0;
  Mn::Vector3 projectedPt;
  std::tie(status, std::ignore, projectedPt) =
      projectToPoly(pt, navQuery_.get(), filter_.get());

  // Clean up if this query was island specific
  if (islandIndex != ID_UNDEFINED) {
    // reset the poly flag identifing polys off the target island
    islandSystem_->setPolyFlagForIsland(
        navMesh_.get(), PolyFlags::POLYFLAGS_OFF_ISLAND, islandIndex,
        /*setFlag=*/false, /*invert=*/true);
    filter_->setExcludeFlags(filter_->getExcludeFlags() &
                             ~PolyFlags::POLYFLAGS_OFF_ISLAND);
  }

  if (dtStatusSucceed(status)) {
    return T{projectedPt};
  }
  return {Mn::Constants::nan(), Mn::Constants::nan(), Mn::Constants::nan()};
}

template <typename T>
int PathFinder::Impl::getIsland(const T& pt) const {
  dtStatus status = 0;
  Mn::Vector3 projectedPt;
  dtPolyRef polyRef = 0;
  std::tie(status, polyRef, projectedPt) =
      projectToPoly(pt, navQuery_.get(), filter_.get());

  if (dtStatusSucceed(status)) {
    return islandSystem_->getPolyIsland(polyRef);
  }
  return ID_UNDEFINED;
}

float PathFinder::Impl::islandRadius(int islandIndex) const {
  return islandSystem_->islandRadius(islandIndex);
}

float PathFinder::Impl::islandRadius(const Mn::Vector3& pt) const {
  dtPolyRef ptRef = 0;
  dtStatus status = 0;
  std::tie(status, ptRef, std::ignore) =
      projectToPoly(pt, navQuery_.get(), filter_.get());
  if (status != DT_SUCCESS || ptRef == 0) {
    return 0.0;
  }
  return islandSystem_->polyIslandRadius(ptRef);
}

float PathFinder::Impl::distanceToClosestObstacle(
    const Mn::Vector3& pt,
    const float maxSearchRadius /*= 2.0*/) const {
  return closestObstacleSurfacePoint(pt, maxSearchRadius).hitDist;
}

HitRecord PathFinder::Impl::closestObstacleSurfacePoint(
    const Mn::Vector3& pt,
    const float maxSearchRadius /*= 2.0*/) const {
  dtPolyRef ptRef = 0;
  dtStatus status = 0;
  Mn::Vector3 polyPt;
  std::tie(status, ptRef, polyPt) =
      projectToPoly(pt, navQuery_.get(), filter_.get());
  if (status != DT_SUCCESS || ptRef == 0) {
    return {Mn::Vector3(0, 0, 0), Mn::Vector3(0, 0, 0),
            std::numeric_limits<float>::infinity()};
  }
  Mn::Vector3 hitPos, hitNormal;
  float hitDist = Mn::Constants::nan();
  navQuery_->findDistanceToWall(ptRef, polyPt.data(), maxSearchRadius,
                                filter_.get(), &hitDist, hitPos.data(),
                                hitNormal.data());
  return {hitPos, hitNormal, hitDist};
}

bool PathFinder::Impl::isNavigable(const Mn::Vector3& pt,
                                   const float maxYDelta /*= 0.5*/) const {
  dtPolyRef ptRef = 0;
  dtStatus status = 0;
  Mn::Vector3 polyPt;
  std::tie(status, ptRef, polyPt) =
      projectToPoly(pt, navQuery_.get(), filter_.get());

  if (status != DT_SUCCESS || ptRef == 0)
    return false;

  if (std::abs(polyPt[1] - pt[1]) > maxYDelta ||
      (Mn::Vector2(pt[0], pt[2]) - Mn::Vector2(polyPt[0], polyPt[2])).length() >
          1e-2f)
    return false;

  return true;
}

MatrixXb PathFinder::Impl::getTopDownView(const float metersPerPixel,
                                          const float height,
                                          const float eps) const {
  std::pair<Mn::Vector3, Mn::Vector3> mapBounds = bounds();
  Mn::Vector3 bound1 = mapBounds.first;
  Mn::Vector3 bound2 = mapBounds.second;

  float xspan = std::abs(bound1[0] - bound2[0]);
  float zspan = std::abs(bound1[2] - bound2[2]);
  int xResolution = xspan / metersPerPixel;
  int zResolution = zspan / metersPerPixel;
  float startx = fmin(bound1[0], bound2[0]);
  float startz = fmin(bound1[2], bound2[2]);
  MatrixXb topdownMap(zResolution, xResolution);

  float curz = startz;
  float curx = startx;
  for (int h = 0; h < zResolution; ++h) {
    for (int w = 0; w < xResolution; ++w) {
      Mn::Vector3 point = Mn::Vector3(curx, height, curz);
      topdownMap(h, w) = isNavigable(point, eps);
      curx = curx + metersPerPixel;
    }
    curz = curz + metersPerPixel;
    curx = startx;
  }

  return topdownMap;
}

MatrixXi PathFinder::Impl::getTopDownIslandView(const float metersPerPixel,
                                                const float height,
                                                const float eps) const {
  std::pair<Mn::Vector3, Mn::Vector3> mapBounds = bounds();
  Mn::Vector3 bound1 = mapBounds.first;
  Mn::Vector3 bound2 = mapBounds.second;

  float xspan = std::abs(bound1[0] - bound2[0]);
  float zspan = std::abs(bound1[2] - bound2[2]);
  int xResolution = xspan / metersPerPixel;
  int zResolution = zspan / metersPerPixel;
  float startx = fmin(bound1[0], bound2[0]);
  float startz = fmin(bound1[2], bound2[2]);
  MatrixXi topdownMap(zResolution, xResolution);

  float curz = startz;
  float curx = startx;
  for (int h = 0; h < zResolution; ++h) {
    for (int w = 0; w < xResolution; ++w) {
      Mn::Vector3 point = Mn::Vector3(curx, height, curz);
      if (isNavigable(point, eps)) {
        // get the island
        topdownMap(h, w) = getIsland(point);
      } else {
        topdownMap(h, w) = -1;
      }
      curx = curx + metersPerPixel;
    }
    curz = curz + metersPerPixel;
    curx = startx;
  }

  return topdownMap;
}

assets::MeshData::ptr PathFinder::Impl::getNavMeshData(
    int islandIndex /*= ID_UNDEFINED*/) {
  islandSystem_->assertValidIsland(islandIndex);

  if (islandMeshData_.find(islandIndex) == islandMeshData_.end() &&
      isLoaded()) {
    assets::MeshData::ptr curIslandMeshData = assets::MeshData::create();
    std::vector<Mn::Vector3>& vbo = curIslandMeshData->vbo;
    std::vector<uint32_t>& ibo = curIslandMeshData->ibo;

    // Iterate over all tiles
    for (int iTile = 0; iTile < navMesh_->getMaxTiles(); ++iTile) {
      const dtMeshTile* tile =
          const_cast<const dtNavMesh*>(navMesh_.get())->getTile(iTile);
      if (!tile)
        continue;

      // Iterate over all polygons in a tile
      for (int jPoly = 0; jPoly < tile->header->polyCount; ++jPoly) {
        // Get the polygon reference from the tile and polygon id
        dtPolyRef polyRef = navMesh_->encodePolyId(tile->salt, iTile, jPoly);
        if (islandIndex != ID_UNDEFINED &&
            islandSystem_->getPolyIsland(polyRef) != islandIndex) {
          // skip polys not in the island.
          continue;
        }
        const dtPoly* poly = nullptr;
        const dtMeshTile* tmp = nullptr;
        navMesh_->getTileAndPolyByRefUnsafe(polyRef, &tmp, &poly);

        CORRADE_INTERNAL_ASSERT(poly != nullptr);
        CORRADE_INTERNAL_ASSERT(tmp != nullptr);

        std::vector<Triangle> triangles = getPolygonTriangles(poly, tile);

        for (auto& tri : triangles) {
          for (int k = 0; k < 3; ++k) {
            vbo.push_back(tri.v[k]);
            ibo.push_back(vbo.size() - 1);
          }
        }
      }
    }
    // return newly added meshdata
    return islandMeshData_.emplace(islandIndex, std::move(curIslandMeshData))
        .first->second;
  }
  // meshdata already exists, so lookup and return
  return islandMeshData_[islandIndex];
}

PathFinder::PathFinder() : pimpl_{spimpl::make_unique_impl<Impl>()} {}

bool PathFinder::build(const NavMeshSettings& bs,
                       const float* verts,
                       const int nverts,
                       const int* tris,
                       const int ntris,
                       const float* bmin,
                       const float* bmax) {
  return pimpl_->build(bs, verts, nverts, tris, ntris, bmin, bmax);
}
bool PathFinder::build(const NavMeshSettings& bs,
                       const esp::assets::MeshData& mesh) {
  return pimpl_->build(bs, mesh);
}

Mn::Vector3 PathFinder::getRandomNavigablePoint(
    const int maxTries /*= 10*/,
    int islandIndex /*= ID_UNDEFINED*/) {
  return pimpl_->getRandomNavigablePoint(maxTries, islandIndex);
}

Mn::Vector3 PathFinder::getRandomNavigablePointAroundSphere(
    const Mn::Vector3& circleCenter,
    const float radius,
    const int maxTries,
    int islandIndex /*= ID_UNDEFINED*/) {
  return pimpl_->getRandomNavigablePointInCircle(circleCenter, radius, maxTries,
                                                 islandIndex);
}

bool PathFinder::findPath(ShortestPath& path) {
  return pimpl_->findPath(path);
}

bool PathFinder::findPath(MultiGoalShortestPath& path) {
  return pimpl_->findPath(path);
}

template Mn::Vector3 PathFinder::tryStep<Mn::Vector3>(const Mn::Vector3&,
                                                      const Mn::Vector3&);

template <typename T>
T PathFinder::tryStep(const T& start, const T& end) {
  return pimpl_->tryStep(start, end, /*allowSliding=*/true);
}

template Mn::Vector3 PathFinder::tryStepNoSliding<Mn::Vector3>(
    const Mn::Vector3&,
    const Mn::Vector3&);

template <typename T>
T PathFinder::tryStepNoSliding(const T& start, const T& end) {
  return pimpl_->tryStep(start, end, /*allowSliding=*/false);
}

template Mn::Vector3 PathFinder::snapPoint<Mn::Vector3>(const Mn::Vector3& pt,
                                                        int islandIndex);

template int PathFinder::getIsland<Mn::Vector3>(const Mn::Vector3& pt);

template <typename T>
T PathFinder::snapPoint(const T& pt, int islandIndex) {
  return pimpl_->snapPoint(pt, islandIndex);
}

template <typename T>
int PathFinder::getIsland(const T& pt) {
  return pimpl_->getIsland(pt);
}

bool PathFinder::loadNavMesh(const std::string& path) {
  return pimpl_->loadNavMesh(path);
}

bool PathFinder::saveNavMesh(const std::string& path) {
  return pimpl_->saveNavMesh(path);
}

bool PathFinder::isLoaded() const {
  return pimpl_->isLoaded();
}

void PathFinder::seed(uint32_t newSeed) {
  return pimpl_->seed(newSeed);
}

float PathFinder::islandRadius(const Mn::Vector3& pt) const {
  return pimpl_->islandRadius(pt);
}

float PathFinder::islandRadius(int islandIndex) const {
  return pimpl_->islandRadius(islandIndex);
}

int PathFinder::numIslands() const {
  return pimpl_->numIslands();
}

float PathFinder::distanceToClosestObstacle(const Mn::Vector3& pt,
                                            const float maxSearchRadius) const {
  return pimpl_->distanceToClosestObstacle(pt, maxSearchRadius);
}

HitRecord PathFinder::closestObstacleSurfacePoint(
    const Mn::Vector3& pt,
    const float maxSearchRadius) const {
  return pimpl_->closestObstacleSurfacePoint(pt, maxSearchRadius);
}

bool PathFinder::isNavigable(const Mn::Vector3& pt,
                             const float maxYDelta) const {
  return pimpl_->isNavigable(pt, maxYDelta);
}

float PathFinder::getNavigableArea(int islandIndex /*= ID_UNDEFINED*/) const {
  return pimpl_->getNavigableArea(islandIndex);
}

std::pair<Mn::Vector3, Mn::Vector3> PathFinder::bounds() const {
  return pimpl_->bounds();
}

MatrixXb PathFinder::getTopDownView(const float metersPerPixel,
                                    const float height,
                                    const float eps) {
  return pimpl_->getTopDownView(metersPerPixel, height, eps);
}

MatrixXi PathFinder::getTopDownIslandView(const float metersPerPixel,
                                          const float height,
                                          const float eps) {
  return pimpl_->getTopDownIslandView(metersPerPixel, height, eps);
}

assets::MeshData::ptr PathFinder::getNavMeshData(
    int islandIndex /*= ID_UNDEFINED*/) {
  return pimpl_->getNavMeshData(islandIndex);
}

Cr::Containers::Optional<NavMeshSettings> PathFinder::getNavMeshSettings()
    const {
  return pimpl_->getNavMeshSettings();
}

}  // namespace nav
}  // namespace esp
