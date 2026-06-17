#pragma once

#include "MeshSurfaceReconstructionInternal.h"

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace geoalgo
{
namespace meshrecon
{

struct PartitionVec3d
{
	double x = 0.0;
	double y = 0.0;
	double z = 0.0;
	PartitionVec3d operator+(const PartitionVec3d& o) const { return {x + o.x, y + o.y, z + o.z}; }
	PartitionVec3d operator-(const PartitionVec3d& o) const { return {x - o.x, y - o.y, z - o.z}; }
	PartitionVec3d operator*(double s) const { return {x * s, y * s, z * s}; }
	double dot(const PartitionVec3d& o) const { return x * o.x + y * o.y + z * o.z; }
	double length() const;
	PartitionVec3d normalized() const;
};

PartitionVec3d crossPartitionVec(const PartitionVec3d& a, const PartitionVec3d& b);
PartitionVec3d readPartitionV(const std::vector<float>& v, int i);
int64_t partitionEdgeKey(int v0, int v1);

struct MeshAdjacency
{
	std::vector<std::vector<int>> fullAdj;
	std::unordered_map<int64_t, std::pair<int, int>> edgeToFaces;
};

MeshAdjacency buildMeshAdjacency(const IndexedMeshLite& mesh, int faceCount);
void computeFaceGeometry(
	const IndexedMeshLite& mesh,
	int faceCount,
	std::vector<PartitionVec3d>& faceNormals,
	std::vector<PartitionVec3d>& faceCentroids,
	std::vector<double>& faceAreas);

void chartToPatches(const std::vector<int>& chart, std::vector<QuadPatch>& patches);
void rebuildPatchAdjacency(
	const std::vector<std::vector<int>>& adj,
	int faceCount,
	std::vector<QuadPatch>& patches);
int computeJunctionCount(const std::vector<QuadPatch>& patches);

void computePatchFaceStats(
	const std::vector<QuadPatch>& patches,
	int minFacesThreshold,
	int& outMin,
	int& outMax,
	int& outSmallCount);

bool partitionQuadDomainsHybrid(
	const IndexedMeshLite& mesh,
	const MeshSurfaceReconstructParams& params,
	std::vector<QuadPatch>& patches,
	int& outJunctionCount,
	MeshSurfaceReconstructReport* partitionStats,
	std::string* errMsg);

} // namespace meshrecon
} // namespace geoalgo
