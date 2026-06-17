#pragma once

#include "TubularGrinding.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace geoalgo
{
namespace tg
{

struct Vec3
{
	double x = 0.0;
	double y = 0.0;
	double z = 0.0;
};

struct IndexedMeshLite
{
	std::vector<float> soup;
	int faceCount = 0;
	std::vector<Vec3> faceCentroids;
	std::vector<Vec3> faceNormals;
	std::vector<std::array<int, 3>> faceVerts;
	std::vector<std::vector<int>> faceNeighbors;
	std::array<double, 3> bboxMin{{0.0, 0.0, 0.0}};
	std::array<double, 3> bboxMax{{0.0, 0.0, 0.0}};
};

bool buildIndexedMeshLite(const std::vector<float>& soup, IndexedMeshLite& out, std::string* errMsg);

void orientMeshFaceNormals(IndexedMeshLite& mesh);

bool isPlanarStencilFace(const IndexedMeshLite& mesh, const int faceIndex, const double minNormalSpreadDeg);

bool rayBundleCenterPoint(
	const std::vector<Vec3>& origins,
	const std::vector<Vec3>& inwardDirs,
	Vec3& outCenter);

void segmentDisplayRgb(int segmentIndex, int segmentCount, float& outR, float& outG, float& outB);

Vec3 normalizeVec3(const Vec3& v);
double dot(const Vec3& a, const Vec3& b);
Vec3 cross(const Vec3& a, const Vec3& b);
Vec3 add(const Vec3& a, const Vec3& b);
Vec3 sub(const Vec3& a, const Vec3& b);
Vec3 scale(const Vec3& v, const double s);
double length(const Vec3& v);
double clamp01(const double v);

/// 对称 3×3 协方差矩阵最小特征值对应特征向量
bool smallestEigenvector3(
	const double cov[3][3],
	Vec3& outEigenvector);

Vec3 computeLocalAxisFromFaceNormals(const IndexedMeshLite& mesh, const int faceIndex);

int runDbscan(
	const std::vector<Vec3>& featurePoints,
	double eps,
	int minPts,
	std::vector<int>& outLabels);

double axisAngleDeg(const Vec3& a, const Vec3& b);

bool rayBundleCenterPoint(
	const std::vector<Vec3>& origins,
	const std::vector<Vec3>& inwardDirs,
	Vec3& outCenter);

/// 射线束近似汇聚点；maxMeanDistanceMm≤0 时按局部尺度自动放宽
bool approximateRayBundleCenter(
	const std::vector<Vec3>& origins,
	const std::vector<Vec3>& inwardDirs,
	double maxMeanDistanceMm,
	Vec3& outCenter);

bool fitCircle2d(
	const std::vector<std::array<double, 2>>& pts,
	double& outCx,
	double& outCy,
	double& outRadius);

void buildFrenetFrames(
	const std::vector<TubularCenterlineSample>& samples,
	std::vector<TubularCenterlineSample>& outSamples);

TubularGrindingTemplateKind selectTemplateKind(
	const TubularPipeSegment& segment,
	const std::vector<TubularCenterlineSample>& samples);

} // namespace tg
} // namespace geoalgo
