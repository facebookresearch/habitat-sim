// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "PathFinder.h"
#include <stack>
#include <unordered_map>

#include <cstdio>
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

using namespace esp;

namespace esp {
namespace nav {
namespace {

std::tuple<dtStatus, dtPolyRef, vec3f> projectToPoly(
    const vec3f& pt,
    const dtNavMeshQuery* navQuery,
    const dtQueryFilter* filter) {
  // Defines size of the bounding box to search in for the nearest polygon.  If
  // there is no polygon inside the bounding box, the status is set to failure
  // and polyRef == 0
  constexpr float polyPickExt[3] = {2, 4, 2};  // [2 * dx, 2 * dy, 2 * dz]
  dtPolyRef polyRef;
  vec3f polyXYZ;
  dtStatus status = navQuery->findNearestPoly(pt.data(), polyPickExt, filter,
                                              &polyRef, polyXYZ.data());

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
        dtPolyRef startRef = navMesh->encodePolyId(iTile, tile->salt, jPoly);

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

      const dtMeshTile* tile = 0;
      const dtPoly* poly = 0;
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

        const dtMeshTile* neighbourTile = 0;
        const dtPoly* neighbourPoly = 0;
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
}  // namespace nav
}  // namespace esp

struct Workspace {
  rcHeightfield* solid = 0;
  unsigned char* triareas = 0;
  rcCompactHeightfield* chf = 0;
  rcContourSet* cset = 0;
  rcPolyMesh* pmesh = 0;
  rcPolyMeshDetail* dmesh = 0;

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

esp::nav::PathFinder::PathFinder() : navMesh_(0), navQuery_(0), filter_(0) {
  filter_ = new dtQueryFilter();
  filter_->setIncludeFlags(POLYFLAGS_WALK);
  filter_->setExcludeFlags(0);
}

void esp::nav::PathFinder::free() {
  if (navMesh_) {
    dtFreeNavMesh(navMesh_);
    navMesh_ = 0;
  }
  if (navQuery_) {
    dtFreeNavMeshQuery(navQuery_);
    navQuery_ = 0;
  }
  if (filter_) {
    delete filter_;
  }

  if (islandSystem_) {
    delete islandSystem_;
  }
}

bool esp::nav::PathFinder::build(const NavMeshSettings& bs,
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
  rcConfig cfg;
  memset(&cfg, 0, sizeof(cfg));
  cfg.cs = bs.cellSize;
  cfg.ch = bs.cellHeight;
  cfg.walkableSlopeAngle = bs.agentMaxSlope;
  cfg.walkableHeight = (int)ceilf(bs.agentHeight / cfg.ch);
  cfg.walkableClimb = (int)floorf(bs.agentMaxClimb / cfg.ch);
  cfg.walkableRadius = (int)ceilf(bs.agentRadius / cfg.cs);
  cfg.maxEdgeLen = (int)(bs.edgeMaxLen / bs.cellSize);
  cfg.maxSimplificationError = bs.edgeMaxError;
  cfg.minRegionArea = (int)rcSqr(bs.regionMinSize);  // Note: area = size*size
  cfg.mergeRegionArea =
      (int)rcSqr(bs.regionMergeSize);  // Note: area = size*size
  cfg.maxVertsPerPoly = (int)bs.vertsPerPoly;
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
    unsigned char* navData = 0;
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

    dtNavMeshCreateParams params;
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

    navMesh_ = 0;
    navMesh_ = dtAllocNavMesh();
    if (!navMesh_) {
      dtFree(navData);
      LOG(ERROR) << "Could not allocate Detour navmesh";
      return false;
    }

    dtStatus status;
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

  LOG(INFO) << "Created navmesh with " << ws.pmesh->nverts << " vertices "
            << ws.pmesh->npolys << " polygons";

  return true;
}

bool esp::nav::PathFinder::initNavQuery() {
  navQuery_ = dtAllocNavMeshQuery();
  dtStatus status = navQuery_->init(navMesh_, 2048);
  if (dtStatusFailed(status)) {
    LOG(ERROR) << "Could not init Detour navmesh query";
    return false;
  }

  islandSystem_ = new impl::IslandSystem(navMesh_, filter_);

  return true;
}

bool esp::nav::PathFinder::build(const NavMeshSettings& bs,
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

static const int NAVMESHSET_MAGIC =
    'M' << 24 | 'S' << 16 | 'E' << 8 | 'T';  //'MSET';
static const int NAVMESHSET_VERSION = 1;

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

bool esp::nav::PathFinder::loadNavMesh(const std::string& path) {
  FILE* fp = fopen(path.c_str(), "rb");
  if (!fp)
    return false;

  // Read header.
  NavMeshSetHeader header;
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
    NavMeshTileHeader tileHeader;
    readLen = fread(&tileHeader, sizeof(tileHeader), 1, fp);
    if (readLen != 1) {
      fclose(fp);
      return false;
    }

    if (!tileHeader.tileRef || !tileHeader.dataSize)
      break;

    unsigned char* data =
        (unsigned char*)dtAlloc(tileHeader.dataSize, DT_ALLOC_PERM);
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
                  tileHeader.tileRef, 0);
  }

  fclose(fp);

  navMesh_ = mesh;
  return initNavQuery();
}

bool esp::nav::PathFinder::saveNavMesh(const std::string& path) {
  if (!navMesh_)
    return false;

  FILE* fp = fopen(path.c_str(), "wb");
  if (!fp)
    return false;

  // Store header.
  NavMeshSetHeader header;
  header.magic = NAVMESHSET_MAGIC;
  header.version = NAVMESHSET_VERSION;
  header.numTiles = 0;
  for (int i = 0; i < navMesh_->getMaxTiles(); ++i) {
    const dtMeshTile* tile = ((const dtNavMesh*)navMesh_)->getTile(i);
    if (!tile || !tile->header || !tile->dataSize)
      continue;
    header.numTiles++;
  }
  memcpy(&header.params, navMesh_->getParams(), sizeof(dtNavMeshParams));
  fwrite(&header, sizeof(NavMeshSetHeader), 1, fp);

  // Store tiles.
  for (int i = 0; i < navMesh_->getMaxTiles(); ++i) {
    const dtMeshTile* tile = ((const dtNavMesh*)navMesh_)->getTile(i);
    if (!tile || !tile->header || !tile->dataSize)
      continue;

    NavMeshTileHeader tileHeader;
    tileHeader.tileRef = navMesh_->getTileRef(tile);
    tileHeader.dataSize = tile->dataSize;
    fwrite(&tileHeader, sizeof(tileHeader), 1, fp);

    fwrite(tile->data, tile->dataSize, 1, fp);
  }

  fclose(fp);

  return true;
}

void esp::nav::PathFinder::seed(uint32_t newSeed) {
  // TODO: this should be using core::Random instead, but passing function
  // to navQuery_->findRandomPoint needs to be figured out first
  srand(newSeed);
}

// Returns a random number [0..1]
static float frand() {
  return (float)rand() / (float)RAND_MAX;
}

vec3f esp::nav::PathFinder::getRandomNavigablePoint() {
  dtPolyRef ref;
  vec3f pt;
  dtStatus status = navQuery_->findRandomPoint(filter_, frand, &ref, pt.data());
  if (!dtStatusSucceed(status)) {
    LOG(ERROR) << "Failed to getRandomNavigablePoint";
  }
  return pt;
}

bool esp::nav::PathFinder::findPath(ShortestPath& path) {
  MultiGoalShortestPath tmp;
  tmp.requestedStart = path.requestedStart;
  tmp.requestedEnds.assign({path.requestedEnd});

  bool status = findPath(tmp);

  path.points.assign(tmp.points.begin(), tmp.points.end());
  path.geodesicDistance = tmp.geodesicDistance;

  return status;
}

bool esp::nav::PathFinder::findPath(MultiGoalShortestPath& path) {
  // initialize
  static const int MAX_POLYS = 256;
  dtPolyRef polys[MAX_POLYS];
  path.geodesicDistance = std::numeric_limits<float>::infinity();
  dtPolyRef startRef;

  // find nearest polys and path
  vec3f pathStart;
  int numPoints = 0;
  int numPolys = 0;
  dtStatus status;
  std::tie(status, startRef, pathStart) =
      projectToPoly(path.requestedStart, navQuery_, filter_);

  if (status != DT_SUCCESS || startRef == 0) {
    return false;
  }

  std::vector<vec3f> pathEnds;
  std::vector<float> pathEndsCoords;
  std::vector<dtPolyRef> endRefs;
  for (const auto& rqEnd : path.requestedEnds) {
    pathEnds.emplace_back();
    endRefs.emplace_back();
    std::tie(status, endRefs.back(), pathEnds.back()) =
        projectToPoly(rqEnd, navQuery_, filter_);

    pathEndsCoords.emplace_back(pathEnds.back()[0]);
    pathEndsCoords.emplace_back(pathEnds.back()[1]);
    pathEndsCoords.emplace_back(pathEnds.back()[2]);

    if (status != DT_SUCCESS || endRefs.back() == 0) {
      return false;
    }
  }

  // check if trivial path (start is same as end) and early return
  if (std::find_if(path.requestedEnds.begin(), path.requestedEnds.end(),
                   [&path](const vec3f& rqEnd) -> bool {
                     return rqEnd.isApprox(path.requestedStart);
                   }) != path.requestedEnds.end()) {
    path.geodesicDistance = 0;
    return true;
  }

  // Check if there is a path between the start and any of the ends
  if (std::find_if(endRefs.begin(), endRefs.end(),
                   [this, &startRef](const dtPolyRef& end) -> bool {
                     return this->islandSystem_->hasConnection(startRef, end);
                   }) == endRefs.end()) {
    return false;
  }

  int goalFoundIdx;
  status = navQuery_->findBidirPathToAny(
      endRefs.size(), startRef, endRefs.data(), path.requestedStart.data(),
      pathEndsCoords.data(), filter_, polys, &numPolys, MAX_POLYS,
      &goalFoundIdx);
  if (status != DT_SUCCESS) {
    return false;
  }

  if (numPolys) {
    const vec3f& closestRequestedEnd = path.requestedEnds[goalFoundIdx];

    path.points.resize(MAX_POLYS);
    status = navQuery_->findStraightPath(
        path.requestedStart.data(), closestRequestedEnd.data(), polys, numPolys,
        path.points[0].data(), 0, 0, &numPoints, MAX_POLYS);

    if (status != DT_SUCCESS) {
      return false;
    }
  }

  // resize down to number of waypoints and compute distance
  if (numPoints > 0) {
    path.points.resize(numPoints);
    path.geodesicDistance = 0;
    if (numPoints > 1) {
      vec3f previousPoint = path.points[0];
      for (int i = 1; i < path.points.size(); i++) {
        const vec3f& currentPoint = path.points[i];
        path.geodesicDistance += (currentPoint - previousPoint).norm();
        previousPoint = currentPoint;
      }
    }
    return true;
  }

  return false;
}

vec3f esp::nav::PathFinder::tryStep(const Eigen::Ref<const vec3f> start,
                                    const Eigen::Ref<const vec3f> end) {
  static const int MAX_POLYS = 256;
  dtPolyRef polys[MAX_POLYS];

  dtPolyRef startRef, endRef;
  vec3f pathStart, pathEnd;
  std::tie(std::ignore, startRef, pathStart) =
      projectToPoly(start, navQuery_, filter_);
  std::tie(std::ignore, endRef, pathEnd) =
      projectToPoly(end, navQuery_, filter_);
  vec3f endPoint;
  int numPolys;
  navQuery_->moveAlongSurface(startRef, pathStart.data(), pathEnd.data(),
                              filter_, endPoint.data(), polys, &numPolys,
                              MAX_POLYS);

  // Hack to deal with infinitely thin walls in recast allowing you to
  // transition between two different connected components
  // First check to see if the endPoint as returned by `moveAlongSurface`
  // is in the same connected component as the startRef according to
  // findNearestPoly
  std::tie(std::ignore, endRef, std::ignore) =
      projectToPoly(endPoint, navQuery_, filter_);
  if (!this->islandSystem_->hasConnection(startRef, endRef)) {
    // There isn't a connection!  This happens when endPoint is on an edge
    // shared between two different connected components (aka infinitely thin
    // walls) The way to deal with this is to nudge the point into the polygon
    // we want it to be 'moveAlongSurface' tells us which polygon we want
    // endPoint to be in through the polys list
    const dtMeshTile* tile = 0;
    const dtPoly* poly = 0;
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

  return endPoint;
}

float esp::nav::PathFinder::islandRadius(const vec3f& pt) const {
  dtPolyRef ptRef;
  dtStatus status;
  std::tie(status, ptRef, std::ignore) = projectToPoly(pt, navQuery_, filter_);
  if (status != DT_SUCCESS || ptRef == 0) {
    return 0.0;
  } else {
    return islandSystem_->islandRadius(ptRef);
  }
}

float esp::nav::PathFinder::distanceToClosestObstacle(
    const vec3f& pt,
    const float maxSearchRadius /*= 2.0*/) const {
  return closestObstacleSurfacePoint(pt, maxSearchRadius).hitDist;
}

esp::nav::HitRecord esp::nav::PathFinder::closestObstacleSurfacePoint(
    const vec3f& pt,
    const float maxSearchRadius /*= 2.0*/) const {
  dtPolyRef ptRef;
  dtStatus status;
  vec3f polyPt;
  std::tie(status, ptRef, polyPt) = projectToPoly(pt, navQuery_, filter_);
  if (status != DT_SUCCESS || ptRef == 0) {
    return {vec3f(0, 0, 0), vec3f(0, 0, 0),
            std::numeric_limits<float>::infinity()};
  } else {
    vec3f hitPos, hitNormal;
    float hitDist;
    navQuery_->findDistanceToWall(ptRef, polyPt.data(), maxSearchRadius,
                                  filter_, &hitDist, hitPos.data(),
                                  hitNormal.data());
    return {hitPos, hitNormal, hitDist};
  }
}

bool esp::nav::PathFinder::isNavigable(const vec3f& pt,
                                       const float maxYDelta /*= 0.5*/) const {
  dtPolyRef ptRef;
  dtStatus status;
  vec3f polyPt;
  std::tie(status, ptRef, polyPt) = projectToPoly(pt, navQuery_, filter_);

  if (status != DT_SUCCESS || ptRef == 0)
    return false;

  if (std::abs(polyPt[1] - pt[1]) > maxYDelta ||
      (Eigen::Vector2f(pt[0], pt[2]) - Eigen::Vector2f(polyPt[0], polyPt[2]))
              .norm() > 1e-2)
    return false;

  return true;
}
