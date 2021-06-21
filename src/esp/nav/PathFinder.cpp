// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "PathFinder.h"
#include <numeric>
#include <stack>
#include <unordered_map>

#include <Magnum/Magnum.h>
#include <Magnum/Math/Vector3.h>

#include <Magnum/EigenIntegration/GeometryIntegration.h>
#include <Magnum/EigenIntegration/Integration.h>

#include <Corrade/Containers/Optional.h>

#include <cstdio>
// NOLINTNEXTLINE
#define _USE_MATH_DEFINES
#include <cmath>
#include <limits>

#include "esp/assets/MeshData.h"
#include "esp/core/esp.h"

#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourNavMeshQuery.h"
#include "DetourNode.h"
#include "Recast.h"

namespace Mn = Magnum;
namespace Cr = Corrade;

namespace esp {
namespace nav {

struct MultiGoalShortestPath::Impl {
  std::vector<vec3f> requestedEnds;

  std::vector<dtPolyRef> endRefs;
  std::vector<vec3f> pathEnds;

  std::vector<float> minTheoreticalDist;
  vec3f prevRequestedStart = vec3f::Zero();
};

MultiGoalShortestPath::MultiGoalShortestPath()
    : pimpl_{spimpl::make_unique_impl<Impl>()} {};

void MultiGoalShortestPath::setRequestedEnds(
    const std::vector<vec3f>& newEnds) {
  pimpl_->endRefs.clear();
  pimpl_->pathEnds.clear();
  pimpl_->requestedEnds = newEnds;

  pimpl_->minTheoreticalDist.assign(newEnds.size(), 0);
}

const std::vector<vec3f>& MultiGoalShortestPath::getRequestedEnds() const {
  return pimpl_->requestedEnds;
}

namespace {
template <typename T>
std::tuple<dtStatus, dtPolyRef, vec3f> projectToPoly(
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
  vec3f polyXYZ = vec3f::Constant(Mn::Constants::nan());
  dtStatus status = navQuery->findNearestPoly(pt.data(), polyPickExt, filter,
                                              &polyRef, polyXYZ.data());

  // So let's call it a failure if it didn't actually find a point....
  if (std::isnan(polyXYZ[0]))
    status = DT_FAILURE;

  return std::make_tuple(status, polyRef, polyXYZ);
}
}  // namespace

namespace impl {

// Runs connected component analysis on the navmesh to figure out which polygons
// are connected This gives O(1) lookup for if a path between two polygons
// exists or not
// Takes O(npolys) to construct
class IslandSystem {
 public:
  IslandSystem(const dtNavMesh* navMesh, const dtQueryFilter* filter) {
    std::vector<vec3f> islandVerts;

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
          vec3f centroid = vec3f::Zero();
          for (auto& v : islandVerts) {
            centroid += v;
          }
          centroid /= islandVerts.size();

          float maxRadius = 0.0;
          for (auto& v : islandVerts) {
            maxRadius = std::max(maxRadius, (v - centroid).norm());
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

  inline float islandRadius(dtPolyRef ref) const {
    auto itRef = polyToIsland_.find(ref);
    if (itRef == polyToIsland_.end())
      return 0.0;

    return islandRadius_[itRef->second];
  }

 private:
  std::unordered_map<dtPolyRef, uint32_t> polyToIsland_;
  std::vector<float> islandRadius_;

  void expandFrom(const dtNavMesh* navMesh,
                  const dtQueryFilter* filter,
                  const uint32_t newIslandId,
                  const dtPolyRef& startRef,
                  std::vector<vec3f>& islandVerts) {
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
        islandVerts.emplace_back(
            Eigen::Map<vec3f>(&tile->verts[poly->verts[iVert] * 3]));
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
             const int nverts,
             const int* tris,
             const int ntris,
             const float* bmin,
             const float* bmax);
  bool build(const NavMeshSettings& bs, const esp::assets::MeshData& mesh);

  vec3f getRandomNavigablePoint(int maxTries);

  bool findPath(ShortestPath& path);
  bool findPath(MultiGoalShortestPath& path);

  template <typename T>
  T tryStep(const T& start, const T& end, bool allowSliding);

  template <typename T>
  T snapPoint(const T& pt);

  bool loadNavMesh(const std::string& path);

  bool saveNavMesh(const std::string& path);

  bool isLoaded() const { return navMesh_ != nullptr; };

  float getNavigableArea() const { return navMeshArea_; };

  void seed(uint32_t newSeed);

  float islandRadius(const vec3f& pt) const;

  float distanceToClosestObstacle(const vec3f& pt,
                                  const float maxSearchRadius = 2.0) const;
  HitRecord closestObstacleSurfacePoint(
      const vec3f& pt,
      const float maxSearchRadius = 2.0) const;

  bool isNavigable(const vec3f& pt, const float maxYDelta = 0.5) const;

  std::pair<vec3f, vec3f> bounds() const { return bounds_; };

  Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> getTopDownView(
      const float metersPerPixel,
      const float height) const;

  assets::MeshData::ptr getNavMeshData();

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
  assets::MeshData::ptr meshData_ = nullptr;

  //! Sum of all NavMesh polygons. Computed on NavMesh load/recompute. See
  //! removeZeroAreaPolys.
  float navMeshArea_ = 0;

  std::pair<vec3f, vec3f> bounds_;

  void removeZeroAreaPolys();

  bool initNavQuery();

  Cr::Containers::Optional<std::tuple<float, std::vector<vec3f>>>
  findPathInternal(const vec3f& start,
                   dtPolyRef startRef,
                   const vec3f& pathStart,
                   const vec3f& end,
                   dtPolyRef endRef,
                   const vec3f& pathEnd);

  bool findPathSetup(MultiGoalShortestPath& path,
                     dtPolyRef& startRef,
                     vec3f& pathStart);
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
  POLYFLAGS_ALL = 0xffff      // all abilities
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
  LOG(INFO) << "Building navmesh with " << cfg.width << "x" << cfg.height
            << " cells";

  //
  // Step 2. Rasterize input polygon soup.
  //

  // Allocate voxel heightfield where we rasterize our input data to.
  ws.solid = rcAllocHeightfield();
  if (!ws.solid) {
    LOG(ERROR) << "Out of memory for heightfield allocation";
    return false;
  }
  if (!rcCreateHeightfield(&ctx, *ws.solid, cfg.width, cfg.height, cfg.bmin,
                           cfg.bmax, cfg.cs, cfg.ch)) {
    LOG(ERROR) << "Could not create solid heightfield";
    return false;
  }

  // Allocate array that can hold triangle area types.
  // If you have multiple meshes you need to process, allocate
  // and array which can hold the max number of triangles you need to process.
  ws.triareas = new unsigned char[ntris];
  if (!ws.triareas) {
    LOG(ERROR) << "Out of memory for triareas" << ntris;
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
    LOG(ERROR) << "Could not rasterize triangles.";
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
    LOG(ERROR) << "Out of memory for compact heightfield";
    return false;
  }
  if (!rcBuildCompactHeightfield(&ctx, cfg.walkableHeight, cfg.walkableClimb,
                                 *ws.solid, *ws.chf)) {
    LOG(ERROR) << "Could not build compact heightfield";
    return false;
  }

  // Erode the walkable area by agent radius.
  if (!rcErodeWalkableArea(&ctx, cfg.walkableRadius, *ws.chf)) {
    LOG(ERROR) << "Could not erode walkable area";
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
    LOG(ERROR) << "Could not build distance field";
    return false;
  }
  // Partition the walkable surface into simple regions without holes.
  if (!rcBuildRegions(&ctx, *ws.chf, 0, cfg.minRegionArea,
                      cfg.mergeRegionArea)) {
    LOG(ERROR) << "Could not build watershed regions";
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
    LOG(ERROR) << "Out of memory for contour set";
    return false;
  }
  if (!rcBuildContours(&ctx, *ws.chf, cfg.maxSimplificationError,
                       cfg.maxEdgeLen, *ws.cset)) {
    LOG(ERROR) << "Could not create contours";
    return false;
  }

  //
  // Step 6. Build polygons mesh from contours.
  //

  // Build polygon navmesh from the contours.
  ws.pmesh = rcAllocPolyMesh();
  if (!ws.pmesh) {
    LOG(ERROR) << "Out of memory for polymesh";
    return false;
  }
  if (!rcBuildPolyMesh(&ctx, *ws.cset, cfg.maxVertsPerPoly, *ws.pmesh)) {
    LOG(ERROR) << "Could not triangulate contours";
    return false;
  }

  //
  // Step 7. Create detail mesh which allows to access approximate height on
  // each polygon.
  //

  ws.dmesh = rcAllocPolyMeshDetail();
  if (!ws.dmesh) {
    LOG(ERROR) << "Out of memory for polymesh detail";
    return false;
  }

  if (!rcBuildPolyMeshDetail(&ctx, *ws.pmesh, *ws.chf, cfg.detailSampleDist,
                             cfg.detailSampleMaxError, *ws.dmesh)) {
    LOG(ERROR) << "Could not build detail mesh";
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
      LOG(ERROR) << "Could not build Detour navmesh";
      return false;
    }

    navMesh_.reset(dtAllocNavMesh());
    if (!navMesh_) {
      dtFree(navData);
      LOG(ERROR) << "Could not allocate Detour navmesh";
      return false;
    }

    dtStatus status = 0;
    status = navMesh_->init(navData, navDataSize, DT_TILE_FREE_DATA);
    if (dtStatusFailed(status)) {
      dtFree(navData);
      LOG(ERROR) << "Could not init Detour navmesh";
      return false;
    }
    if (!initNavQuery()) {
      return false;
    }
  }

  bounds_ = std::make_pair(vec3f(bmin), vec3f(bmax));

  // Added as we also need to remove these on navmesh recomputation
  removeZeroAreaPolys();

  LOG(INFO) << "Created navmesh with " << ws.pmesh->nverts << " vertices "
            << ws.pmesh->npolys << " polygons";

  return true;
}

bool PathFinder::Impl::initNavQuery() {
  // if we are reinitializing the NavQuery, then also reset the MeshData
  meshData_.reset();

  navQuery_.reset(dtAllocNavMeshQuery());
  dtStatus status = navQuery_->init(navMesh_.get(), 2048);
  if (dtStatusFailed(status)) {
    LOG(ERROR) << "Could not init Detour navmesh query";
    return false;
  }

  islandSystem_ =
      std::make_unique<impl::IslandSystem>(navMesh_.get(), filter_.get());

  return true;
}

bool PathFinder::Impl::build(const NavMeshSettings& bs,
                             const esp::assets::MeshData& mesh) {
  const int numVerts = mesh.vbo.size();
  const int numIndices = mesh.ibo.size();
  const float mf = std::numeric_limits<float>::max();
  vec3f bmin(mf, mf, mf);
  vec3f bmax(-mf, -mf, -mf);

  for (int i = 0; i < numVerts; i++) {
    const vec3f& p = mesh.vbo[i];
    bmin = bmin.cwiseMin(p);
    bmax = bmax.cwiseMax(p);
  }

  int* indices = new int[numIndices];
  for (int i = 0; i < numIndices; i++) {
    indices[i] = static_cast<int>(mesh.ibo[i]);
  }

  const bool success = build(bs, mesh.vbo[0].data(), numVerts, indices,
                             numIndices / 3, bmin.data(), bmax.data());
  delete[] indices;
  return success;
}

namespace {
const int NAVMESHSET_MAGIC = 'M' << 24 | 'S' << 16 | 'E' << 8 | 'T';  //'MSET';
const int NAVMESHSET_VERSION = 1;

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
  std::vector<vec3f> v;
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
    const unsigned char* t = &tile->detailTris[(pd->triBase + j) * 4];
    const float* v[3];
    for (int k = 0; k < 3; ++k) {
      if (t[k] < poly->vertCount)
        triangles[j].v[k] =
            Eigen::Map<const vec3f>(&tile->verts[poly->verts[t[k]] * 3]);
      else
        triangles[j].v[k] = Eigen::Map<const vec3f>(
            &tile->detailVerts[(pd->vertBase + (t[k] - poly->vertCount)) * 3]);
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
    const vec3f w1 = tri.v[1] - tri.v[0];
    const vec3f w2 = tri.v[2] - tri.v[1];
    area += 0.5 * w1.cross(w2).norm();
  }

  return area;
}
}  // namespace

// Some polygons have zero area for some reason.  When we navigate into a zero
// area polygon, things crash.  So we find all zero area polygons and mark
// them as disabled/not navigable.
// Also compute the total NavMesh area for later query.
void PathFinder::Impl::removeZeroAreaPolys() {
  navMeshArea_ = 0;
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
      const dtPoly* poly = nullptr;
      const dtMeshTile* tmp = nullptr;
      navMesh_->getTileAndPolyByRefUnsafe(polyRef, &tmp, &poly);

      CORRADE_INTERNAL_ASSERT(poly != nullptr);
      CORRADE_INTERNAL_ASSERT(tmp != nullptr);

      float polygonArea = polyArea(poly, tile);
      if (polygonArea < 1e-5) {
        navMesh_->setPolyFlags(polyRef, POLYFLAGS_DISABLED);
      } else if ((poly->flags & POLYFLAGS_WALK) != 0) {
        navMeshArea_ += polygonArea;
      }
    }
  }
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
  if (header.version != NAVMESHSET_VERSION) {
    fclose(fp);
    return false;
  }

  vec3f bmin, bmax;

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
      bmin = vec3f(tile->header->bmin);
      bmax = vec3f(tile->header->bmax);
    } else {
      bmin = bmin.array().min(Eigen::Array3f{tile->header->bmin});
      bmax = bmax.array().max(Eigen::Array3f{tile->header->bmax});
    }
  }

  fclose(fp);

  navMesh_.reset(mesh);
  bounds_ = std::make_pair(bmin, bmax);

  removeZeroAreaPolys();

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
    header.numTiles++;
  }
  memcpy(&header.params, navMesh->getParams(), sizeof(dtNavMeshParams));
  fwrite(&header, sizeof(NavMeshSetHeader), 1, fp);

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

vec3f PathFinder::Impl::getRandomNavigablePoint(const int maxTries /*= 10*/) {
  if (getNavigableArea() <= 0.0)
    throw std::runtime_error(
        "NavMesh has no navigable area, this indicates an issue with the "
        "NavMesh");

  vec3f pt;

  int i = 0;
  for (i = 0; i < maxTries; ++i) {
    dtPolyRef ref = 0;
    dtStatus status =
        navQuery_->findRandomPoint(filter_.get(), frand, &ref, pt.data());
    if (dtStatusSucceed(status))
      break;
  }

  if (i == maxTries) {
    LOG(ERROR) << "Failed to getRandomNavigablePoint.  Try increasing max "
                  "tries if the navmesh is fine but just hard to sample from";
    return vec3f::Constant(Mn::Constants::nan());
  } else {
    return pt;
  }
}

namespace {
float pathLength(const std::vector<vec3f>& points) {
  CORRADE_INTERNAL_ASSERT(points.size() > 0);

  float length = 0;
  const vec3f* previousPoint = &points[0];
  for (const auto& pt : points) {
    length += (*previousPoint - pt).norm();
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

Cr::Containers::Optional<std::tuple<float, std::vector<vec3f>>>
PathFinder::Impl::findPathInternal(const vec3f& start,
                                   dtPolyRef startRef,
                                   const vec3f& pathStart,
                                   const vec3f& end,
                                   dtPolyRef endRef,
                                   const vec3f& pathEnd) {
  // check if trivial path (start is same as end) and early return
  if (pathStart.isApprox(pathEnd)) {
    return std::make_tuple(0.0f, std::vector<vec3f>{pathStart, pathEnd});
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
  std::vector<vec3f> points(MAX_POLYS);
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
                                     vec3f& pathStart) {
  path.geodesicDistance = std::numeric_limits<float>::infinity();
  path.points.clear();

  // find nearest polys and path
  dtStatus status = 0;
  std::tie(status, startRef, pathStart) =
      projectToPoly(path.requestedStart, navQuery_.get(), filter_.get());

  if (status != DT_SUCCESS || startRef == 0) {
    return false;
  }

  if (path.pimpl_->endRefs.size() != 0)
    return true;

  for (const auto& rqEnd : path.getRequestedEnds()) {
    dtPolyRef endRef = 0;
    vec3f pathEnd;
    std::tie(status, endRef, pathEnd) =
        projectToPoly(rqEnd, navQuery_.get(), filter_.get());

    if (status != DT_SUCCESS || endRef == 0) {
      return false;
    }

    path.pimpl_->endRefs.emplace_back(endRef);
    path.pimpl_->pathEnds.emplace_back(pathEnd);
  }

  return true;
}

bool PathFinder::Impl::findPath(MultiGoalShortestPath& path) {
  dtPolyRef startRef = 0;
  vec3f pathStart;
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

    for (int i = 0; i < path.pimpl_->requestedEnds.size(); ++i) {
      path.pimpl_->minTheoreticalDist[i] = std::max(
          path.pimpl_->minTheoreticalDist[i] - movedAmount,
          (path.pimpl_->requestedEnds[i] - path.requestedStart).norm());
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
    if (path.pimpl_->minTheoreticalDist[i] > path.geodesicDistance)
      continue;

    const Cr::Containers::Optional<std::tuple<float, std::vector<vec3f>>>
        findResult =
            findPathInternal(path.requestedStart, startRef, pathStart,
                             path.pimpl_->requestedEnds[i],
                             path.pimpl_->endRefs[i], path.pimpl_->pathEnds[i]);

    if (findResult && std::get<0>(*findResult) < path.geodesicDistance) {
      path.pimpl_->minTheoreticalDist[i] = std::get<0>(*findResult);
      path.geodesicDistance = std::get<0>(*findResult);
      path.points = std::get<1>(*findResult);
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
  vec3f pathStart;
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

  vec3f endPoint;
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
    vec3f polyCenter = vec3f::Zero();
    for (int iVert = 0; iVert < poly->vertCount; ++iVert) {
      polyCenter += Eigen::Map<vec3f>(&tile->verts[poly->verts[iVert] * 3]);
    }
    polyCenter /= poly->vertCount;

    constexpr float nudgeDistance = 1e-4;  // 0.1mm
    const vec3f nudgeDir = (polyCenter - endPoint).normalized();
    // And nudge the point towards the center by a little tiny bit :)
    endPoint = endPoint + nudgeDistance * nudgeDir;
  }

  return T{endPoint};
}

template <typename T>
T PathFinder::Impl::snapPoint(const T& pt) {
  dtStatus status = 0;
  vec3f projectedPt;
  std::tie(status, std::ignore, projectedPt) =
      projectToPoly(pt, navQuery_.get(), filter_.get());

  if (dtStatusSucceed(status)) {
    return T{projectedPt};
  } else {
    return {Mn::Constants::nan(), Mn::Constants::nan(), Mn::Constants::nan()};
  }
}

float PathFinder::Impl::islandRadius(const vec3f& pt) const {
  dtPolyRef ptRef = 0;
  dtStatus status = 0;
  std::tie(status, ptRef, std::ignore) =
      projectToPoly(pt, navQuery_.get(), filter_.get());
  if (status != DT_SUCCESS || ptRef == 0) {
    return 0.0;
  } else {
    return islandSystem_->islandRadius(ptRef);
  }
}

float PathFinder::Impl::distanceToClosestObstacle(
    const vec3f& pt,
    const float maxSearchRadius /*= 2.0*/) const {
  return closestObstacleSurfacePoint(pt, maxSearchRadius).hitDist;
}

HitRecord PathFinder::Impl::closestObstacleSurfacePoint(
    const vec3f& pt,
    const float maxSearchRadius /*= 2.0*/) const {
  dtPolyRef ptRef = 0;
  dtStatus status = 0;
  vec3f polyPt;
  std::tie(status, ptRef, polyPt) =
      projectToPoly(pt, navQuery_.get(), filter_.get());
  if (status != DT_SUCCESS || ptRef == 0) {
    return {vec3f(0, 0, 0), vec3f(0, 0, 0),
            std::numeric_limits<float>::infinity()};
  } else {
    vec3f hitPos, hitNormal;
    float hitDist = NAN;
    navQuery_->findDistanceToWall(ptRef, polyPt.data(), maxSearchRadius,
                                  filter_.get(), &hitDist, hitPos.data(),
                                  hitNormal.data());
    return {hitPos, hitNormal, hitDist};
  }
}

bool PathFinder::Impl::isNavigable(const vec3f& pt,
                                   const float maxYDelta /*= 0.5*/) const {
  dtPolyRef ptRef = 0;
  dtStatus status = 0;
  vec3f polyPt;
  std::tie(status, ptRef, polyPt) =
      projectToPoly(pt, navQuery_.get(), filter_.get());

  if (status != DT_SUCCESS || ptRef == 0)
    return false;

  if (std::abs(polyPt[1] - pt[1]) > maxYDelta ||
      (Eigen::Vector2f(pt[0], pt[2]) - Eigen::Vector2f(polyPt[0], polyPt[2]))
              .norm() > 1e-2)
    return false;

  return true;
}

typedef Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> MatrixXb;

Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>
PathFinder::Impl::getTopDownView(const float metersPerPixel,
                                 const float height) const {
  std::pair<vec3f, vec3f> mapBounds = bounds();
  vec3f bound1 = mapBounds.first;
  vec3f bound2 = mapBounds.second;

  float xspan = std::abs(bound1[0] - bound2[0]);
  float zspan = std::abs(bound1[2] - bound2[2]);
  int xResolution = xspan / metersPerPixel;
  int zResolution = zspan / metersPerPixel;
  float startx = fmin(bound1[0], bound2[0]);
  float startz = fmin(bound1[2], bound2[2]);
  MatrixXb topdownMap(zResolution, xResolution);

  float curz = startz;
  float curx = startx;
  for (int h = 0; h < zResolution; h++) {
    for (int w = 0; w < xResolution; w++) {
      vec3f point = vec3f(curx, height, curz);
      topdownMap(h, w) = isNavigable(point, 0.5);
      curx = curx + metersPerPixel;
    }
    curz = curz + metersPerPixel;
    curx = startx;
  }

  return topdownMap;
}

assets::MeshData::ptr PathFinder::Impl::getNavMeshData() {
  if (meshData_ == nullptr && isLoaded()) {
    meshData_ = assets::MeshData::create();
    std::vector<esp::vec3f>& vbo = meshData_->vbo;
    std::vector<uint32_t>& ibo = meshData_->ibo;

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
  }
  return meshData_;
}

PathFinder::PathFinder() : pimpl_{spimpl::make_unique_impl<Impl>()} {};

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

vec3f PathFinder::getRandomNavigablePoint(const int maxTries /*= 10*/) {
  return pimpl_->getRandomNavigablePoint(maxTries);
}

bool PathFinder::findPath(ShortestPath& path) {
  return pimpl_->findPath(path);
}

bool PathFinder::findPath(MultiGoalShortestPath& path) {
  return pimpl_->findPath(path);
}

template vec3f PathFinder::tryStep<vec3f>(const vec3f&, const vec3f&);
template Mn::Vector3 PathFinder::tryStep<Mn::Vector3>(const Mn::Vector3&,
                                                      const Mn::Vector3&);

template <typename T>
T PathFinder::tryStep(const T& start, const T& end) {
  return pimpl_->tryStep(start, end, /*allowSliding=*/true);
}

template vec3f PathFinder::tryStepNoSliding<vec3f>(const vec3f&, const vec3f&);
template Mn::Vector3 PathFinder::tryStepNoSliding<Mn::Vector3>(
    const Mn::Vector3&,
    const Mn::Vector3&);

template <typename T>
T PathFinder::tryStepNoSliding(const T& start, const T& end) {
  return pimpl_->tryStep(start, end, /*allowSliding=*/false);
}

template vec3f PathFinder::snapPoint<vec3f>(const vec3f& pt);
template Mn::Vector3 PathFinder::snapPoint<Mn::Vector3>(const Mn::Vector3& pt);

template <typename T>
T PathFinder::snapPoint(const T& pt) {
  return pimpl_->snapPoint(pt);
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

float PathFinder::islandRadius(const vec3f& pt) const {
  return pimpl_->islandRadius(pt);
}

float PathFinder::distanceToClosestObstacle(const vec3f& pt,
                                            const float maxSearchRadius) const {
  return pimpl_->distanceToClosestObstacle(pt, maxSearchRadius);
}

HitRecord PathFinder::closestObstacleSurfacePoint(
    const vec3f& pt,
    const float maxSearchRadius) const {
  return pimpl_->closestObstacleSurfacePoint(pt, maxSearchRadius);
}

bool PathFinder::isNavigable(const vec3f& pt, const float maxYDelta) const {
  return pimpl_->isNavigable(pt, maxYDelta);
}

float PathFinder::getNavigableArea() const {
  return pimpl_->getNavigableArea();
}

std::pair<vec3f, vec3f> PathFinder::bounds() const {
  return pimpl_->bounds();
}

Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> PathFinder::getTopDownView(
    const float metersPerPixel,
    const float height) {
  return pimpl_->getTopDownView(metersPerPixel, height);
}

assets::MeshData::ptr PathFinder::getNavMeshData() {
  return pimpl_->getNavMeshData();
}

}  // namespace nav
}  // namespace esp
