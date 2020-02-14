/*
 *  Progressive Mesh type Polygon Reduction Algorithm
 *  by Stan Melax (c) 1998
 *  Permission to use any of this code wherever you want is granted..
 *  Although, please do acknowledge authorship if appropriate.
 *
 *  See the header file progmesh.h for a description of this module
 */

#include <cstring>
#include <fstream>
#include <iostream>

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#define NOMINMAX
#include <assert.h>

#include <tinyply.h>

#include "linalg.h"  // typical 3D math routines following hlsl style for the most part
using namespace linalg::aliases;
#include "geometric.h"
#include "progmesh.h"

template <typename T>
void copyTo(std::shared_ptr<tinyply::PlyData> data, std::vector<T>& dst) {
  dst.resize(data->count);
  assert(data->buffer.size_bytes() == sizeof(T) * dst.size());
  std::memcpy(dst.data(), data->buffer.get(), data->buffer.size_bytes());
}

template <class T>
int Contains(const std::vector<T>& c, const T& t) {
  return std::count(begin(c), end(c), t);
}
template <class T>
int IndexOf(const std::vector<T>& c, const T& v) {
  return std::find(begin(c), end(c), v) - begin(c);
}  // Note: Not presently called
template <class T>
T& Add(std::vector<T>& c, T t) {
  c.push_back(t);
  return c.back();
}
template <class T>
T Pop(std::vector<T>& c) {
  auto val = std::move(c.back());
  c.pop_back();
  return val;
}
template <class T>
void AddUnique(std::vector<T>& c, T t) {
  if (!Contains(c, t))
    c.push_back(t);
}
template <class T>
void Remove(std::vector<T>& c, T t) {
  auto it = std::find(begin(c), end(c), t);
  assert(it != end(c));
  c.erase(it);
  assert(!Contains(c, t));
}

/*
 *  For the polygon reduction algorithm we use data structures
 *  that contain a little bit more information than the usual
 *  indexed face set type of data structure.
 *  From a vertex we wish to be able to quickly get the
 *  neighboring faces and vertices.
 */
class Triangle;
class Vertex;

class Triangle {
 public:
  Vertex* vertex[3];  // the 3 points that make this tri
  float3 normal;      // unit vector othogonal to this face
  int objid;
  Triangle(Vertex* v0, Vertex* v1, Vertex* v2, int id = 0);
  ~Triangle();
  void ComputeNormal();
  void ReplaceVertex(Vertex* vold, Vertex* vnew);
  int HasVertex(Vertex* v);
};
class Vertex {
 public:
  float3 position;                // location of point in euclidean space
  int id;                         // place of vertex in original Array
  std::vector<Vertex*> neighbor;  // adjacent vertices
  std::vector<Triangle*> face;    // adjacent triangles
  float objdist;                  // cached cost of collapsing edge
  bool dirty = false;
  Vertex* collapse;  // candidate vertex for collapse
  Vertex(float3 v, int _id);
  ~Vertex();
  void RemoveIfNonNeighbor(Vertex* n);
};
std::vector<Vertex*> vertices;
std::vector<Triangle*> triangles;

struct CmpVertices {
  bool operator()(const Vertex* lhs, const Vertex* rhs) {
    return lhs->objdist > rhs->objdist;
  }
};

Triangle::Triangle(Vertex* v0, Vertex* v1, Vertex* v2, int id) {
  assert(v0 != v1 && v1 != v2 && v2 != v0);
  vertex[0] = v0;
  vertex[1] = v1;
  vertex[2] = v2;
  objid = id;
  ComputeNormal();
  triangles.push_back(this);
  for (int i = 0; i < 3; i++) {
    vertex[i]->face.push_back(this);
    for (int j = 0; j < 3; j++)
      if (i != j) {
        AddUnique(vertex[i]->neighbor, vertex[j]);
      }
  }
}
Triangle::~Triangle() {
  // Remove(triangles, this);
  for (int i = 0; i < 3; i++) {
    if (vertex[i])
      Remove(vertex[i]->face, this);
  }
  for (int i = 0; i < 3; i++) {
    int i2 = (i + 1) % 3;
    if (!vertex[i] || !vertex[i2])
      continue;
    vertex[i]->RemoveIfNonNeighbor(vertex[i2]);
    vertex[i2]->RemoveIfNonNeighbor(vertex[i]);
  }
}
int Triangle::HasVertex(Vertex* v) {
  return (v == vertex[0] || v == vertex[1] || v == vertex[2]);
}
void Triangle::ComputeNormal() {
  float3 v0 = vertex[0]->position;
  float3 v1 = vertex[1]->position;
  float3 v2 = vertex[2]->position;
  normal = cross(v1 - v0, v2 - v1);
  if (length(normal) == 0)
    return;
  normal = normalize(normal);
}

void Triangle::ReplaceVertex(Vertex* vold, Vertex* vnew) {
  assert(vold && vnew);
  assert(vold == vertex[0] || vold == vertex[1] || vold == vertex[2]);
  assert(vnew != vertex[0] && vnew != vertex[1] && vnew != vertex[2]);
  if (vold == vertex[0]) {
    vertex[0] = vnew;
  } else if (vold == vertex[1]) {
    vertex[1] = vnew;
  } else {
    assert(vold == vertex[2]);
    vertex[2] = vnew;
  }
  Remove(vold->face, this);
  assert(!Contains(vnew->face, this));
  vnew->face.push_back(this);
  for (int i = 0; i < 3; i++) {
    vold->RemoveIfNonNeighbor(vertex[i]);
    vertex[i]->RemoveIfNonNeighbor(vold);
  }
  for (int i = 0; i < 3; i++) {
    assert(Contains(vertex[i]->face, this) == 1);
    for (int j = 0; j < 3; j++)
      if (i != j) {
        AddUnique(vertex[i]->neighbor, vertex[j]);
      }
  }
  ComputeNormal();
}

Vertex::Vertex(float3 v, int _id) {
  position = v;
  id = _id;
  vertices.push_back(this);
}

Vertex::~Vertex() {
  assert(face.size() == 0);
  while (neighbor.size()) {
    Remove(neighbor[0]->neighbor, this);
    Remove(neighbor, neighbor[0]);
  }
  // Remove(vertices, this);
}
void Vertex::RemoveIfNonNeighbor(Vertex* n) {
  // removes n from neighbor Array if n isn't a neighbor.
  if (!Contains(neighbor, n))
    return;
  for (unsigned int i = 0; i < face.size(); i++) {
    if (face[i]->HasVertex(n))
      return;
  }
  Remove(neighbor, n);
}

float ComputeEdgeCollapseCost(Vertex* u, Vertex* v) {
  // if we collapse edge uv by moving u to v then how
  // much different will the model change, i.e. how much "error".
  // Texture, vertex normal, and border vertex code was removed
  // to keep this demo as simple as possible.
  // The method of determining cost was designed in order
  // to exploit small and coplanar regions for
  // effective polygon reduction.
  // Is is possible to add some checks here to see if "folds"
  // would be generated.  i.e. normal of a remaining face gets
  // flipped.  I never seemed to run into this problem and
  // therefore never added code to detect this case.
  float edgelength = length(v->position - u->position);
  float curvature = 0;

  // find the "sides" triangles that are on the edge uv
  std::vector<Triangle*> sides;
  for (unsigned int i = 0; i < u->face.size(); i++) {
    if (u->face[i]->HasVertex(v)) {
      sides.push_back(u->face[i]);
    }
  }
  // use the triangle facing most away from the sides
  // to determine our curvature term
  for (unsigned int i = 0; i < u->face.size(); i++) {
    float mincurv = 1;  // curve for face i and closer side to it
    for (unsigned int j = 0; j < sides.size(); j++) {
      float dotprod =
          dot(u->face[i]->normal,
              sides[j]->normal);  // use dot product of face normals.
      mincurv = std::min(mincurv, (1 - dotprod) / 2.0f);
    }
    curvature = std::max(curvature, mincurv);
  }
  int semcostfactor = 1;
  if (u->face.size() > 1) {
    int objid = u->face[0]->objid;
    for (unsigned int i = 1; i < u->face.size(); i++) {
      if (objid != u->face[i]->objid) {
        semcostfactor++;
      }
    }
  } else {
    assert(u->face.size() == 1);
    semcostfactor++;
  }
  float cost =
      edgelength * curvature * semcostfactor + edgelength * (semcostfactor - 1);
#if 0
  std::cout << "cost of " << v->position << " is " << cost  << std::endl;
#endif
  // the more coplanar the lower the curvature term
  return cost;
}

void ComputeEdgeCostAtVertex(Vertex* v) {
  // compute the edge collapse cost for all edges that start
  // from vertex v.  Since we are only interested in reducing
  // the object by selecting the min cost edge at each step, we
  // only cache the cost of the least cost edge at this vertex
  // (in member variable collapse) as well as the value of the
  // cost (in member variable objdist).
  if (v->neighbor.size() == 0) {
    // v doesn't have neighbors so it costs nothing to collapse
    v->collapse = NULL;
    v->objdist = -0.01f;
    return;
  }
  v->objdist = 1000000;
  v->collapse = NULL;
  // search all neighboring edges for "least cost" edge
  for (unsigned int i = 0; i < v->neighbor.size(); i++) {
    float dist;
    dist = ComputeEdgeCollapseCost(v, v->neighbor[i]);
    if (dist < v->objdist) {
      v->collapse = v->neighbor[i];  // candidate for edge collapse
      v->objdist = dist;             // cost of the collapse
    }
  }
}
void ComputeAllEdgeCollapseCosts() {
  // For all the edges, compute the difference it would make
  // to the model if it was collapsed.  The least of these
  // per vertex is cached in each vertex object.
  for (unsigned int i = 0; i < vertices.size(); i++) {
    ComputeEdgeCostAtVertex(vertices[i]);
  }
}

void Collapse(Vertex* u, Vertex* v) {
  // Collapse the edge uv by moving vertex u onto v
  // Actually remove tris on uv, then update tris that
  // have u to have v, and then remove u.
  if (!v) {
    // u is a vertex all by itself so just delete it
#ifdef DEBUG
    std::cout << "Collapsing " << u->position << std::endl;
#endif
    delete u;
    return;
  }
#ifdef DEBUG
  std::cout << "Collapsing " << u->position << " to " << v->position
            << std::endl;
#endif
  std::vector<Vertex*> tmp;
  // make tmp a Array of all the neighbors of u
  for (unsigned int i = 0; i < u->neighbor.size(); i++) {
    tmp.push_back(u->neighbor[i]);
  }
  // delete triangles on edge uv:
  {
    auto i = u->face.size();
    while (i--) {
      if (u->face[i]->HasVertex(v)) {
        delete (u->face[i]);
      }
    }
  }
  // update remaining triangles to have v instead of u
  {
    auto i = u->face.size();
    while (i--) {
      u->face[i]->ReplaceVertex(u, v);
    }
  }
  delete u;
  // recompute the edge collapse costs for neighboring vertices
  for (unsigned int i = 0; i < tmp.size(); i++) {
    ComputeEdgeCostAtVertex(tmp[i]);
    tmp[i]->dirty = true;
  }
}

void AddVertex(std::vector<float3>& vert) {
  for (unsigned int i = 0; i < vert.size(); i++) {
    Vertex* v = new Vertex(vert[i], i);
  }
}
void AddFaces(std::vector<tridata>& tri, std::vector<int>& objid) {
  for (unsigned int i = 0; i < tri.size(); i++) {
    Vertex *v0 = vertices[tri[i].v[0]];
    Vertex *v1 = vertices[tri[i].v[1]];
    Vertex *v2 = vertices[tri[i].v[2]];
    // Skip zero-size triangles
    if (v0 == v1 || v1 == v2 || v2 == v0)
      continue;
    Triangle* t = new Triangle(v0, v1, v2, objid[i]);
  }
}

Vertex* MinimumCostEdge() {
  // Find the edge that when collapsed will affect model the least.
  // This funtion actually returns a Vertex, the second vertex
  // of the edge (collapse candidate) is stored in the vertex data.
  // Serious optimization opportunity here: this function currently
  // does a sequential search through an unsorted Array :-(
  // Our algorithm could be O(n*lg(n)) instead of O(n*n)
  Vertex* mn = vertices[0];
  for (unsigned int i = 0; i < vertices.size(); i++) {
    if (vertices[i]->objdist < mn->objdist) {
      mn = vertices[i];
    }
  }
  return mn;
}

void ProgressiveMesh(std::vector<float3>& vert,
                     std::vector<tridata>& tri,
                     std::vector<int>& objid,
                     std::vector<int>& map,
                     std::vector<int>& permutation) {
  AddVertex(vert);  // put input data into our data structures
  AddFaces(tri, objid);
  std::cout << "Start computing costs" << std::endl;
  ComputeAllEdgeCollapseCosts();  // cache all edge collapse costs
  std::cout << "Done computing costs" << std::endl;
  permutation.resize(vertices.size());  // allocate space
  map.resize(vertices.size());          // allocate space
  std::make_heap(vertices.begin(), vertices.end(), CmpVertices{});
  int i = 0;
  // reduce the object down to nothing:
  while (vertices.size() > 0) {
    // get the next vertex to collapse
    // Vertex* mn = MinimumCostEdge();
    Vertex* mn = nullptr;
  popagain:
    std::pop_heap(vertices.begin(), vertices.end(), CmpVertices{});
    mn = vertices.back();
    if (mn->dirty) {
      mn->dirty = false;
      std::push_heap(vertices.begin(), vertices.end(), CmpVertices{});
      goto popagain;
    }
    vertices.pop_back();
    // keep track of this vertex, i.e. the collapse ordering
    permutation[mn->id] = vertices.size();
    // keep track of vertex to which we collapse to
    map[vertices.size()] = (mn->collapse) ? mn->collapse->id : -1;
    // Collapse this edge
    Collapse(mn, mn->collapse);
    if (++i % 10000 == 0)
      std::cout << "." << std::flush;
  }
  std::cout << std::endl;
  // reorder the map Array based on the collapse ordering
#ifdef DEBUG
  std::cout << "map:";
#endif
  for (unsigned int i = 0; i < map.size(); i++) {
    map[i] = (map[i] == -1) ? 0 : permutation[map[i]];
#ifdef DEBUG
    std::cout << " " << map[i];
#endif
  }
#ifdef DEBUG
  std::cout << std::endl;
#endif
  // The caller of this function should reorder their vertices
  // according to the returned "permutation".
}

void PermuteVertices(std::vector<float3>& vert,
                     std::vector<tridata>& tri,
                     std::vector<int>& permutation) {
  // rearrange the vertex Array
  std::vector<float3> temp_Array;
  unsigned int i;
  assert(permutation.size() == vert.size());
  for (i = 0; i < vert.size(); i++) {
    temp_Array.push_back(vert[i]);
  }
  for (i = 0; i < vert.size(); i++) {
    vert[permutation[i]] = temp_Array[i];
  }
#ifdef DEBUG
  std::cout << "vert:" << std::endl;
  for (i = 0; i < vert.size(); i++) {
    std::cout << vert[i] << std::endl;
  }
  std::cout << std::endl;
#endif
  // update the changes in the entries in the triangle Array
  for (i = 0; i < tri.size(); i++) {
    for (int j = 0; j < 3; j++) {
      tri[i].v[j] = permutation[tri[i].v[j]];
    }
  }
}

void LoadData(std::vector<float3>& vert,
              std::vector<tridata>& tri,
              std::vector<int>& objid) {
  float verts[6][3] = {{0, 0, 0}, {1, 0, 0}, {2, 0, 0},
                       {0, 1, 0}, {1, 1, 0}, {0, 2, 0}};
  float tris[4][3] = {{0, 1, 4}, {0, 4, 3}, {1, 2, 4}, {3, 4, 5}};
  float objids[4] = {0, 1, 0, 1};
  int i;
  for (i = 0; i < 6; i++) {
    float* vp = verts[i];
    vert.push_back(float3(vp[0], vp[1], vp[2]));
  }
  for (i = 0; i < 4; i++) {
    tridata td;
    td.v[0] = tris[i][0];
    td.v[1] = tris[i][1];
    td.v[2] = tris[i][2];
    tri.push_back(td);
  }
  objid = std::vector<int>(objids, objids + sizeof(objids) / sizeof(int));
}

int Map(std::vector<int>& collapse_map, int a, int mx) {
  if (mx <= 0)
    return 0;
  while (a >= mx) {
    a = collapse_map[a];
  }
  return a;
}

#if 0
void DrawModelTriangles(std::vector<float3>& vert,
                        std::vector<tridata>& tri,
                        std::vector<int>& objid,
                        std::vector<int>& collapse_map,
                        int render_num) {
  assert(collapse_map.size());
  int renderpolycount = 0;
  for (unsigned int i = 0; i < tri.size(); i++) {
    int p0 = Map(collapse_map, tri[i].v[0], render_num);
    int p1 = Map(collapse_map, tri[i].v[1], render_num);
    int p2 = Map(collapse_map, tri[i].v[2], render_num);
    // note:  serious optimization opportunity here,
    //  by sorting the triangles the following "continue"
    //  could have been made into a "break" statement.
    if (p0 == p1 || p1 == p2 || p2 == p0)
      continue;
    renderpolycount++;
    float3 v0, v1, v2;
    v0 = vert[p0];
    v1 = vert[p1];
    v2 = vert[p2];
    std::cout << v0 << "," << v1 << "," << v2 << " " << objid[i] << std::endl;
  }
  std::cout << renderpolycount++ << std::endl;
}
#endif

void LoadDataFromFile(const std::string plyFile,
                      std::vector<float3>& vert,
                      std::vector<tridata>& tri,
                      std::vector<int>& objid) {
  std::ifstream ifs(plyFile, std::ios::binary);
  if (!ifs.good()) {
    std::cerr << "Cannot open file at " << plyFile;
    return;
  }

  tinyply::PlyFile file;
  if (!file.parse_header(ifs)) {
    std::cerr << "Could not read header";
    return;
  }

  std::shared_ptr<tinyply::PlyData> vertices, colors, face_inds, object_ids;

  vertices = file.request_properties_from_element("vertex", {"x", "y", "z"});

  face_inds =
      file.request_properties_from_element("face", {"vertex_indices"}, 0);

  object_ids = file.request_properties_from_element("face", {"object_id"});

  file.read(ifs);

  assert(vertices->t == tinyply::Type::FLOAT32);
  copyTo(vertices, vert);

  // We can load int32 index buffers as uint32 index buffers as they are simply
  // indices into an array and thus can't be negative. int32 is really uint31
  // and can be safely reinterpret_casted to uint32
  assert(face_inds->t == tinyply::Type::INT32 ||
         face_inds->t == tinyply::Type::UINT32);
  // We can figure out the number of vertices per face by dividing the number
  // of bytes needed to store the faces by the number of faces and the size of
  // each vertex
  int vertexPerFace =
      face_inds->buffer.size_bytes() / (face_inds->count * sizeof(uint32_t));
  assert(vertexPerFace == 3);
  copyTo(face_inds, tri);

  // If the input mesh is a quad mesh, then we had to double the number of
  // primitives, so we also need to double the size of the object IDs buffer
  int indicesPerFace = 3 * (vertexPerFace == 4 ? 2 : 1);
  if (object_ids->t == tinyply::Type::INT32 ||
      object_ids->t == tinyply::Type::UINT32) {
    copyTo(object_ids, objid);
  } else {
    std::cerr << "Cannot load object_id of type "
              << tinyply::PropertyTable[object_ids->t].str;
    return;
  }

  std::cout << vert.size() << std::endl;
  std::cout << tri.size() << std::endl;
  std::cout << objid.size() << std::endl;
}

int CalcNumFaces(std::vector<float3>& vert,
                 std::vector<tridata>& tri,
                 std::vector<int>& collapse_map,
                 int numVerts) {
  int renderpolycount = 0;
  for (unsigned int i = 0; i < tri.size(); i++) {
    int p0 = Map(collapse_map, tri[i].v[0], numVerts);
    int p1 = Map(collapse_map, tri[i].v[1], numVerts);
    int p2 = Map(collapse_map, tri[i].v[2], numVerts);
    // note:  serious optimization opportunity here,
    //  by sorting the triangles the following "continue"
    //  could have been made into a "break" statement.
    if (p0 == p1 || p1 == p2 || p2 == p0)
      continue;
    renderpolycount++;
  }
  return renderpolycount;
}

void WriteDataToFile(const std::string plyFile,
                     std::vector<float3>& vert,
                     std::vector<tridata>& tri,
                     std::vector<int>& objid,
                     std::vector<int>& collapse_map,
                     int numVerts) {
  int numFaces = CalcNumFaces(vert, tri, collapse_map, numVerts);
  std::ofstream f(plyFile, std::ios::out | std::ios::binary);
  f << "ply" << std::endl;
  f << "format binary_little_endian 1.0" << std::endl;
  f << "element vertex " << numVerts << std::endl;
  f << "property float x" << std::endl;
  f << "property float y" << std::endl;
  f << "property float z" << std::endl;
  f << "property uchar red" << std::endl;
  f << "property uchar green" << std::endl;
  f << "property uchar blue" << std::endl;
  f << "element face " << numFaces << std::endl;
  f << "property list uchar int vertex_indices" << std::endl;
  f << "property ushort object_id" << std::endl;
  f << "end_header" << std::endl;

  // We need to rotate to match .glb where -Z is gravity
  for (size_t i = 0; i < numVerts; i++) {
    unsigned char gray[] = {0x80, 0x80, 0x80};
    float3 vertex = vert[i];
    f.write(reinterpret_cast<char*>(&vertex), sizeof(vertex));
    f.write(reinterpret_cast<char*>(gray), sizeof(gray));
  }

  for (unsigned int i = 0; i < tri.size(); i++) {
    int p0 = Map(collapse_map, tri[i].v[0], numVerts);
    int p1 = Map(collapse_map, tri[i].v[1], numVerts);
    int p2 = Map(collapse_map, tri[i].v[2], numVerts);
    // note:  serious optimization opportunity here,
    //  by sorting the triangles the following "continue"
    //  could have been made into a "break" statement.
    if (p0 == p1 || p1 == p2 || p2 == p0)
      continue;
    f.put(3);
    f.write(reinterpret_cast<char*>(&p0), sizeof(p0));
    f.write(reinterpret_cast<char*>(&p1), sizeof(p1));
    f.write(reinterpret_cast<char*>(&p2), sizeof(p2));
    unsigned short id = objid[i];
    f.write(reinterpret_cast<char*>(&id), sizeof(id));
  }

  f.close();
}

int main(int argc, char* argv[]) {
  std::vector<float3> vert;  // global Array of vertices
  std::vector<tridata> tri;  // global Array of triangles
  std::vector<int> objid;
  std::vector<int> collapse_map;  // to which neighbor each vertex collapses
  std::vector<int> permutation;
  if (argc != 4) {
    std::cerr << "Usage: " << argv[0] << " in.ply out.ply numvertices"
              << std::endl;
    exit(1);
  }

  LoadDataFromFile(argv[1], vert, tri, objid);
  std::cout << "loaded" << std::endl;
  if (vert.size() < atoi(argv[3])) {
    std::cerr << "Nothing to do. Mesh has less than " << argv[3] << " vertices" << std::endl;
    exit(1);
  }

  // LoadData(vert, tri, objid);
  ProgressiveMesh(vert, tri, objid, collapse_map, permutation);
  PermuteVertices(vert, tri, permutation);
  WriteDataToFile(argv[2], vert, tri, objid, collapse_map, atoi(argv[3]));
  // DrawModelTriangles(vert, tri, objid, collapse_map, 4);
  return 0;
}
