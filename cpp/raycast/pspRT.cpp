#include <algorithm>
#include <chrono>

//#define DBG
#include "utils/pspDbg.h"
#include "utils/pspRT.h"
#include "utils/pspTimer.h"

using namespace std;

// static BVH members
size_t rt::BVH::leafNodes=0;
size_t rt::BVH::interiorNodes=0;
size_t rt::BVH::totalPrimitives=0;

using V3f = Imath::V3f;
using Box3f = Imath::Box3f;

////////////////////////////////// Hit ///////////////////////////////////

rt::Hit::Hit(void) : pos(0),nrm(0),t(std::numeric_limits<float>::max()),
		     u(0),v(0),w(0),prim(nullptr) {}
rt::Hit::Hit(const Hit& that) { *this = that; }
rt::Hit& rt::Hit::operator=(const Hit& that)
{
  this->geomID=that.geomID;
  this->primID=that.primID;
  this->pos=that.pos;
  this->nrm=that.nrm;
  this->t=that.t;
  this->u=that.u;
  this->v=that.v;
  this->w=that.w;
  this->prim=that.prim;
  return *this;
}

////////////////////////////////// Ray ///////////////////////////////////

#define ABS(dir) V3f( fabs(dir[0]),fabs(dir[1]),fabs(dir[2]) )
#define MAX_DIM(adir) (adir[0]>adir[1]) ? ( adir[0]>adir[2] ? 0 : 2 ) : ( adir[1]>adir[2] ? 1 : 2 )

rt::Ray::Ray(void): o(0),d(0),i(0),tmax(std::numeric_limits<float>::max()),kx(0),ky(0),kz(0),Sx(0),Sy(0),Sz(0){}
rt::Ray::Ray(const Ray& that) { *this = that; }
rt::Ray::Ray(const V3f& orig, const V3f& dir) :
  o(orig), d(dir), tmax(std::numeric_limits<float>::max())
{
  // blute: 2019-May-28 watertight fixes

  V3f ad = ABS(d);
  int md = MAX_DIM(ad);

  // calculate dimension where the raydirection is maximal
  kz = MAX_DIM( ABS(d) );
  kx = kz+1; if( kx==3 ) kx=0;
  ky = kx+1; if( ky==3 ) ky=0;
  // swap kx and ky dim to preserve winding dir of triangles
  if( d[kz] < 0.f ) std::swap(kx,ky);

  // calculate shear constants, to make ray unit length aligned with Z
  Sx = d[kx] / d[kz];
  Sy = d[ky] / d[kz];
  Sz = 1.f / d[kz];

  // old
  i[0] = 1.0f / d[0];
  i[1] = 1.0f / d[1];
  i[2] = 1.0f / d[2];
}

rt::Ray& rt::Ray::operator=(const Ray& that)
{
  this->o=that.o;
  this->d=that.d;
  this->i=that.i;
  this->tmax=that.tmax;

  this->kx=that.kx;
  this->ky=that.ky;
  this->kz=that.kz;

  this->Sx=that.Sx;
  this->Sy=that.Sy;
  this->Sz=that.Sz;

  return *this;
}

//////////////////////////////// Triangle ////////////////////////////////

rt::Triangle::Triangle(const std::shared_ptr<TriangleMesh> &m, int triNumber) : mesh(m), primID(triNumber)
{
  vi = &mesh->vertexIndices[3 * triNumber];
}

Box3f
rt::Triangle::bounds(void) const
{
  const V3f &p0 = mesh->p[vi[0]];
  const V3f &p1 = mesh->p[vi[1]];
  const V3f &p2 = mesh->p[vi[2]];
  Box3f res;
  res.extendBy(p0);
  res.extendBy(p1);
  res.extendBy(p2);
  return res;
}

bool
rt::Triangle::intersect(const Ray &ray, Hit *hit) const
{
  const Imath::V3f &origA = mesh->p[vi[0]];
  const Imath::V3f &origB = mesh->p[vi[1]];
  const Imath::V3f &origC = mesh->p[vi[2]];
  const Imath::V3f A = origA - ray.o;
  const Imath::V3f B = origB - ray.o;
  const Imath::V3f C = origC - ray.o;

  // shear/scale verts
  const float Ax = A[ray.kx] - ray.Sx * A[ray.kz];
  const float Ay = A[ray.ky] - ray.Sy * A[ray.kz];
  const float Bx = B[ray.kx] - ray.Sx * B[ray.kz];
  const float By = B[ray.ky] - ray.Sy * B[ray.kz];
  const float Cx = C[ray.kx] - ray.Sx * C[ray.kz];
  const float Cy = C[ray.ky] - ray.Sy * C[ray.kz];

  // calc scaled bary coords
  float U = Cx*By - Cy*Bx;
  float V = Ax*Cy - Ay*Cx;
  float W = Bx*Ay - By*Ax;

  // fall back to dbl prec test for edges
  if( U==0.f or V==0.f or W==0.f )
  {
    double CxBy = static_cast<double>(Cx) * static_cast<double>(By);
    double CyBx = static_cast<double>(Cy) * static_cast<double>(Bx);
    U = static_cast<float>(CxBy - CyBx);

    double AxCy = static_cast<double>(Ax) * static_cast<double>(Cy);
    double AyCx = static_cast<double>(Ay) * static_cast<double>(Cx);
    V = static_cast<float>(AxCy - AyCx);

    double BxAy = static_cast<double>(Bx) * static_cast<double>(Ay);
    double ByAx = static_cast<double>(By) * static_cast<double>(Ax);
    W = static_cast<float>(BxAy - ByAx);
  }

  // no backface culling
  if(  (U<0.f or V<0.f or W<0.f) and (U>0.f or V>0.f or W>0.f) ) return false;

  // calculate determinant
  float det = U+V+W;

  // ray is co-planar with the triangle, hit something else
  if( det==0.f ) return false;

  // calc scaled z-coords of verts, use to calc the hit dist
  const float Az = ray.Sz*A[ray.kz];
  const float Bz = ray.Sz*B[ray.kz];
  const float Cz = ray.Sz*C[ray.kz];
  const float T  = U*Az + V*Bz + W*Cz;

  float xorf_T = std::abs(T);
  if(std::signbit(T) != std::signbit(det))
    xorf_T = -xorf_T;

  float ray_near = 0.0f;
  float hit_t = std::numeric_limits<float_t>::infinity();
  float abs_det = std::abs(det);
  if(xorf_T < ray_near * abs_det or hit_t * abs_det < xorf_T)
    return false;

  const float rcpDet = 1.f / det;
  hit->u = U*rcpDet;
  hit->v = V*rcpDet;
  hit->w = W*rcpDet;
  hit->t = T*rcpDet;

  hit->pos = ray.o + hit->t * ray.d;
  hit->primID = primID;

  const Imath::V3f edge1 = origB - origA;
  const Imath::V3f edge2 = origC - origA;
  const float EPS=1e-3;
  if( edge1.length() < EPS or edge2.length() < EPS ){
    hit->nrm = Imath::V3f(0,0,0);
  } else {
    hit->nrm = edge1.cross(edge2);
    if( hit->nrm.dot(ray.d) > 0.f  ) hit->nrm = -hit->nrm;
  }

  return true;
}

////////////////////////////// TriangleMesh //////////////////////////////

rt::TriangleMesh::TriangleMesh(const std::vector<Imath::V3f>& t, const size_t stride) :
  nVertices(t.size()),nTriangles(t.size()/XYZ)
{

  p = t;
  for(int tidx=0;tidx<nVertices;++tidx)
    vertexIndices.push_back(tidx);
}

std::vector<std::shared_ptr<rt::Primitive>>
rt::CreateTriangleMesh(const std::vector<float>& raw, size_t stride)
{

  size_t nv = raw.size()/stride;
  std::vector<Imath::V3f> inTris( nv );
  for( size_t vidx=0; vidx<nv ; ++vidx ){
    const float* v = raw.data() + vidx*stride;
    inTris[vidx] = Imath::V3f(v[0],v[1],v[2]);
  }
  std::shared_ptr<TriangleMesh> mesh = std::make_shared<TriangleMesh>(inTris, stride);
  std::vector<std::shared_ptr<Primitive>> outTris;
  outTris.reserve(mesh->nTriangles);
  for (int i = 0; i < mesh->nTriangles; ++i)
    outTris.push_back(std::make_shared<Triangle>(mesh, i));
  return outTris;
}

//////////////////////////////////////////////////////////////////////////
////////////////////////////// BVH support ///////////////////////////////
//////////////////////////////////////////////////////////////////////////

///////////////////////////// BVH::BuildNode /////////////////////////////

void rt::BVH::BuildNode::InitLeaf(int first, int n, const Box3f &b)
{
  firstPrimOffset = first;
  nPrimitives = n;
  bounds = b;
  children[0] = children[1] = nullptr;
  ++leafNodes;
  totalPrimitives += n;
}

void rt::BVH::BuildNode::InitInterior(int axis, BuildNode *c0, BuildNode *c1)
{
  children[0] = c0;
  children[1] = c1;

  bounds = c0->bounds;
  bounds.extendBy(c1->bounds);

  splitAxis = axis;
  nPrimitives = 0;
  ++interiorNodes;
}

/////////////////////////// BVH::PrimitiveInfo ///////////////////////////

rt::BVH::PrimitiveInfo::PrimitiveInfo(void)
{}

rt::BVH::PrimitiveInfo::PrimitiveInfo(size_t primitiveNumber, const Box3f &bounds) :
  primitiveNumber(primitiveNumber),
  bounds(bounds),
  centroid(.5f * bounds.min + .5f * bounds.max)
{}

////////////////////////////////// BVH ///////////////////////////////////

namespace rt {

  V3f Offset(const Box3f& b, const V3f &p)
  {
    V3f o = p - b.min;
    if (b.max[0] > b.min[0]) o[0] /= b.max[0] - b.min[0];
    if (b.max[1] > b.min[1]) o[1] /= b.max[1] - b.min[1];
    if (b.max[2] > b.min[2]) o[2] /= b.max[2] - b.min[2];
    return o;
  }

  float SurfaceArea(const Box3f& b)
  {
    V3f d = b.size();
    return 2 * (d[0] * d[1] + d[0] * d[2] + d[1] * d[2]);
  }

} // namespace rt

Box3f
rt::BVH::checkbuild(rt::BVH::BuildNode* node)
{
  Box3f subs;
  if( node->nPrimitives ){
    for (int i = 0; i < node->nPrimitives; ++i)
      subs.extendBy(primitives[node->firstPrimOffset + i]->bounds());

    std::cerr<<"LEAF "
	     <<subs.min[0]<<" "
	     <<subs.min[1]<<" "
	     <<subs.min[2]<<" "
	     <<subs.max[0]<<" "
	     <<subs.max[1]<<" "
	     <<subs.max[2]<<"\n";
  } else {
    if( node->children[0] ) subs.extendBy( checkbuild( node->children[0] ) );
    if( node->children[1] ) subs.extendBy( checkbuild( node->children[1] ) );
  }

  for(int d=0;d<3;++d){
    ASSERT( node->bounds.min[d] <= subs.min[d] and subs.max[d] <= node->bounds.max[d] );
  }

  return subs;

} // check tree

rt::BVH::BVH(const std::vector<std::shared_ptr<Primitive>> &p, int maxPrimsInNode) :
  primitives(p), nodes(nullptr)
{
    //psp::BlockTimer bt("BVH build");

  FANCYMESG(__PRETTY_FUNCTION__);

  if ( primitives.empty() ) {
    std::cerr<<"BVH::BVH() : no primitives!\n";
    return;
  }

  //auto abgn = std::chrono::system_clock::now();

  std::vector<PrimitiveInfo> primitiveInfo( primitives.size() );

  for (size_t i = 0; i < primitives.size(); ++i)
    primitiveInfo[i] = {i, primitives[i]->bounds()};

  std::vector<std::shared_ptr<Primitive>> orderedPrims( primitives.size() );

  int totalNodes = 0;
  MemoryArena arena(1024 * 1024);
  BuildNode *root = recursiveBuild(arena, primitiveInfo, 0, primitives.size(), &totalNodes, orderedPrims);
  primitives.swap(orderedPrims);

  nodes = AllocAligned<LinearNode>(totalNodes);

  int offset = 0;
  flattenTree(root, &offset);
  ASSERT(offset == totalNodes);
}

rt::BVH::~BVH(void)
{
  FANCYMESG(__PRETTY_FUNCTION__);

  FreeAligned(nodes);
}

Box3f
rt::BVH::bounds(void) const
{
  return nodes ? nodes[0].bounds : Box3f();
}

bool
rt::BVH::intersect(const Ray &ray, Hit *isect) const
{
  if (!nodes) {
    DIE("rt::BVH::intersect(): no nodes!");
    return false;
  }

  bool anyhit = false;

  int dirIsNeg[3] = {ray.i[0] < 0, ray.i[1] < 0, ray.i[2] < 0};

  // Follow ray through BVH nodes to find primitive intersections

  int toVisitOffset = 0, currentNodeIndex = 0;
  int nodesToVisit[64];

  while (true) {

    const LinearNode *node = &nodes[currentNodeIndex];

    // Check ray against BVH node
    //if (node->bounds.IntersectP(ray, ray.i, dirIsNeg))
    Imath::Line3f line( ray.o, ray.o + ray.d );
    Imath::V3f hitpos;

    if( intersects(node->bounds, line, hitpos) )
    {
      if (node->nPrimitives > 0)
      {
	// Intersect ray with primitives in leaf BVH node
	Hit hitrec;
	for (int i = 0; i < node->nPrimitives; ++i){
	  if (primitives[node->primitivesOffset + i]->intersect(ray, &hitrec))
	  {
	    anyhit = true;
	    if( hitrec.t < isect->t ){
	      hitrec.prim = primitives[node->primitivesOffset + i].get();
	      *isect = hitrec;

	    }
	  }
	}
	if (toVisitOffset == 0) {
	  break;
	}
	currentNodeIndex = nodesToVisit[--toVisitOffset];
      }
      else
      {
	// Put far BVH node on _nodesToVisit_ stack, advance to near node
	if (dirIsNeg[node->axis])
	{
	  nodesToVisit[toVisitOffset++] = currentNodeIndex + 1; // save lo child for later
	  currentNodeIndex = node->secondChildOffset;           //  use hi child now
	}
	else
	{
	  nodesToVisit[toVisitOffset++] = node->secondChildOffset; // save hi child for later
	  currentNodeIndex = currentNodeIndex + 1;                 //  use lo child now
	}
      }
    }
    else
    {
      if (toVisitOffset == 0) {
	break;
      }
      currentNodeIndex = nodesToVisit[--toVisitOffset];
    }
  }
  return anyhit;
}

int
rt::BVH::flattenTree(BuildNode *node, int *offset)
{
  LinearNode *linearNode = &nodes[*offset]; // dst
  linearNode->bounds = node->bounds;

  int myOffset = (*offset)++; // 0 => 1

  if (node->nPrimitives > 0) {
    ASSERT(!node->children[0] && !node->children[1]);
    ASSERT(node->nPrimitives < 65536);
    linearNode->primitivesOffset = node->firstPrimOffset;
    linearNode->nPrimitives = node->nPrimitives;
  } else {
    // Create interior flattened BVH node
    linearNode->axis = node->splitAxis;
    linearNode->nPrimitives = 0;
    flattenTree(node->children[0], offset);
    linearNode->secondChildOffset = flattenTree(node->children[1], offset);
  }
  return myOffset;
}

rt::BVH::BuildNode*
rt::BVH::recursiveBuild(MemoryArena &arena,
			std::vector<PrimitiveInfo> &primitiveInfo,
			int bgn, int end, int* totalNodes,
			std::vector<std::shared_ptr<Primitive>> &orderedPrims)
{
  const int PRIMS_PER_LEAF = 4;
  BuildNode *node = arena.Alloc<BuildNode>();
  (*totalNodes)++;

  Imath::Box3f bounds;
  for (int i = bgn; i < end; ++i)
    bounds.extendBy(primitiveInfo[i].bounds);

  int nPrimitives = end - bgn;
  if (nPrimitives <= PRIMS_PER_LEAF )
  {
    int firstPrimOffset = orderedPrims.size();
    for (int i = bgn; i < end; ++i) {
      int primNum = primitiveInfo[i].primitiveNumber;
      orderedPrims.push_back(primitives[primNum]);
    }
    node->InitLeaf(firstPrimOffset, nPrimitives, bounds);
    return node;
  }

  Imath::Box3f centroidBounds;
  for (int i = bgn; i < end; ++i)
    centroidBounds.extendBy( primitiveInfo[i].centroid );
  int dim = centroidBounds.majorAxis();

  // flat in the split dim => LEAF
  if (centroidBounds.max[dim] == centroidBounds.min[dim])
  {
    int firstPrimOffset = orderedPrims.size();
    for (int i = bgn; i < end; ++i) {
      int primNum = primitiveInfo[i].primitiveNumber;
      orderedPrims.push_back(primitives[primNum]);
    }
    node->InitLeaf(firstPrimOffset, nPrimitives, bounds);
    return node;
  }

  // ok use SAH algorithm to possibly split the primitives
  constexpr int nBuckets = 12;
  BucketInfo buckets[nBuckets];

  // Initialize _BucketInfo_ for SAH partition buckets
  for (int i = bgn; i < end; ++i) {
    int b = nBuckets * rt::Offset(centroidBounds,primitiveInfo[i].centroid)[dim];
    if (b == nBuckets) b = nBuckets - 1;
    ASSERT(b >= 0 && b < nBuckets);
    buckets[b].count++;
    buckets[b].bounds.extendBy( primitiveInfo[i].bounds );
  } // for i

  // Compute costs for splitting after each bucket
  float cost[nBuckets - 1];
  for (int i = 0; i < nBuckets - 1; ++i) {
    Imath::Box3f b0, b1;
    int count0 = 0, count1 = 0;
    for (int j = 0; j <= i; ++j) {
      b0.extendBy( buckets[j].bounds );
      count0 += buckets[j].count;
    }
    for (int j = i + 1; j < nBuckets; ++j) {
      b1.extendBy( buckets[j].bounds );
      count1 += buckets[j].count;
    }
    cost[i] = 1.f + ((float)count0 * rt::SurfaceArea(b0) +
		     (float)count1 * rt::SurfaceArea(b1)) / rt::SurfaceArea(bounds);
  } // for i

  // Find bucket to split at that minimizes SAH metric
  float minCost = cost[0];
  int minCostSplitBucket = 0;
  for (int i = 1; i < nBuckets - 1; ++i) {
    if (cost[i] < minCost) {
      minCost = cost[i];
      minCostSplitBucket = i;
    }
  } // for i

  float leafCost = (float)nPrimitives;
  bool shouldSplit = nPrimitives > PRIMS_PER_LEAF || minCost < leafCost;
  int mid = (bgn + end) / 2;

  if( shouldSplit )
  {
    PrimitiveInfo *pmid = std::partition
      (&primitiveInfo[bgn], &primitiveInfo[end - 1] + 1,[=](const PrimitiveInfo &pi)
       {
	 int b = nBuckets * rt::Offset(centroidBounds,pi.centroid)[dim];
	 if (b == nBuckets) b = nBuckets - 1;
	 ASSERT(b >= 0 && b < nBuckets);
	 return b <= minCostSplitBucket;
       });
    mid = pmid - &primitiveInfo[0];
  }
  else // not shouldSplit => LEAF
  {
    int firstPrimOffset = orderedPrims.size();
    for (int i = bgn; i < end; ++i) {
      int primNum = primitiveInfo[i].primitiveNumber;
      orderedPrims.push_back(primitives[primNum]);
    }
    node->InitLeaf(firstPrimOffset, nPrimitives, bounds);
    return node;
  }

  node->InitInterior(dim,
		     recursiveBuild(arena, primitiveInfo, bgn, mid, totalNodes, orderedPrims),
		     recursiveBuild(arena, primitiveInfo, mid, end, totalNodes, orderedPrims));

  return node;

} // rt::BVH::recursiveBuild
