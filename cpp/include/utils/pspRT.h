#ifndef _pspRT_h_
#define _pspRT_h_

#include "ImathVec.h"
#include "ImathQuat.h"
#include "ImathMatrix.h"
#include "ImathLine.h"
#include "ImathBox.h"
#include "ImathBoxAlgo.h"

#include <vector>
#include <memory>
#include "pspRTmem.h"

namespace rt
{
	using V3f = Imath::V3f;
	using Box3f = Imath::Box3f;

	const size_t XYZ = 3;
	const size_t TRI = 9;
	const int maxPrimsInNode = 4;

	struct Primitive;

	struct Hit
	{
		Hit(void);
		Hit(const Hit &);
		Hit &operator=(const Hit &);
		V3f pos, nrm;
		float t, u, v, w;
		int geomID, primID;
		const Primitive *prim;
	}; // rt::Hit

	struct Ray
	{
		Ray(void);
		Ray(const Ray &);
		Ray(const V3f &orig, const V3f &dir);
		Ray &operator=(const Ray &);
		inline V3f at(float t) const { return o + t * d; }
		V3f o, d, i;
		float tmax;

		// watertight support
		int kx, ky, kz; // ray dir indices, possible swizzled
		float Sx, Sy, Sz;
	}; // rt::Ray

	// something a ray can hit
	struct Primitive
	{
		virtual Box3f bounds() const = 0;
		virtual bool intersect(const Ray &ray, Hit *hit) const = 0;
	}; // rt::Primitive

	struct BVH : public Primitive
	{

		struct BuildNode
		{
			void InitLeaf(int first, int n, const Box3f &b);
			void InitInterior(int axis, BuildNode *c0, BuildNode *c1);
			Box3f bounds;
			BuildNode *children[2];
			int splitAxis, firstPrimOffset, nPrimitives;
		}; // BVH::BuildNode

		struct LinearNode
		{
			Box3f bounds;
			union
			{
				int primitivesOffset;  // leaf
				int secondChildOffset; // interior
			};
			uint16_t nPrimitives; // 0 -> interior node
			uint8_t axis;		  // interior node: xyz
			uint8_t pad[1];		  // ensure 32 byte total size
		};						  // BVH::LinearNode

		struct PrimitiveInfo
		{
			PrimitiveInfo(void);
			PrimitiveInfo(size_t primitiveNumber, const Box3f &bounds);
			size_t primitiveNumber;
			Box3f bounds;
			V3f centroid;
		}; // BVH::PrimitiveInfo

		struct BucketInfo
		{
			int count = 0;
			Box3f bounds;
		}; // BVH::BucketInfo

		BVH(const std::vector<std::shared_ptr<Primitive>> &p, int maxPrimsInNode = 1);
		~BVH(void);

		Box3f bounds(void) const;
		bool intersect(const Ray &ray, Hit *hit) const;

		Box3f checkbuild(rt::BVH::BuildNode *);

		int flattenTree(BuildNode *node, int *offset);
		BuildNode *recursiveBuild(MemoryArena &arena,
								  std::vector<PrimitiveInfo> &primitiveInfo,
								  int bgn, int end, int *totalNodes,
								  std::vector<std::shared_ptr<Primitive>> &orderedPrims);

		static size_t leafNodes, interiorNodes, totalPrimitives;
		std::vector<std::shared_ptr<Primitive>> primitives; // copy
		LinearNode *nodes = nullptr;

	}; // rt::BVH

	// mass storage for triangles
	struct TriangleMesh
	{
		TriangleMesh(const std::vector<Imath::V3f> &t, const size_t stride = XYZ);
		const int nVertices, nTriangles;
		std::vector<int> vertexIndices;
		std::vector<Imath::V3f> p;
	}; // rt::TriangleMesh

	std::vector<std::shared_ptr<rt::Primitive>>
	CreateTriangleMesh(const std::vector<float> &raw, size_t stride);

	std::unique_ptr<BVH>
	CreateBVH(const std::vector<float> &raw, size_t stride)
	{
		return std::make_unique<BVH>(CreateTriangleMesh(raw, stride), 4);
	}

	// hitable triangles
	struct Triangle : public Primitive
	{
		Triangle(const std::shared_ptr<TriangleMesh> &m, int triNumber);

		Box3f bounds(void) const;
		bool intersect(const Ray &ray, Hit *hit) const;

		std::shared_ptr<TriangleMesh> mesh;
		const int *vi;
		int primID;
	}; // rt::Triangle

} // namespace rt

#endif
