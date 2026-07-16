#pragma once

#include "geometry_algorithm_global.h"

#include <cstddef>
#include <string>
#include <vector>

namespace geoalgo
{

struct Point3d
{
	double x = 0.0;
	double y = 0.0;
	double z = 0.0;
};

struct Polyline3d
{
	std::vector<float> xyz;
};

struct TessellateParams
{
	double linearDeflectionMm = 0.01;
	bool linearDeflectionRelative = true;
	double angularDeflectionDeg = 0.5;
	bool flipReversedFaces = true;
};

enum class MeshDiscretizeMode
{
	AdaptiveTriangulation,
	UniformRelative,
	UVStructuredGrid,
	WireTubeMesh,
	WireRibbonMesh,
	ProfileSweepMesh,
	RemeshSoup,
	PointCloudSurface
};

enum class MeshQualityPreset
{
	Coarse,
	Medium,
	Fine,
	Custom
};

enum class MeshDensityControl
{
	QualityPreset,
	TargetEdgeLength,
	TargetTriangleCount
};

struct MeshDiscretizeParams
{
	MeshDiscretizeMode mode = MeshDiscretizeMode::AdaptiveTriangulation;
	MeshQualityPreset quality = MeshQualityPreset::Medium;
	MeshDensityControl densityControl = MeshDensityControl::QualityPreset;
	double targetEdgeLengthMm = 0.0;
	std::size_t targetTriangleCount = 0;
	TessellateParams tessellate;
	int uvGridCountU = 32;
	int uvGridCountV = 32;
	double tubeRadiusMm = 1.0;
	int tubeSides = 12;
	double ribbonWidthMm = 2.0;
	bool mergeCoplanarTriangles = false;
};

struct MeshDiscretizeReport
{
	std::size_t triangleCount = 0;
	double bboxDiagonalMm = 0.0;
	double avgEdgeLengthMm = 0.0;
	MeshDiscretizeMode modeUsed = MeshDiscretizeMode::AdaptiveTriangulation;
};

struct MeshHierarchyPart
{
	std::string partPath;
	std::string parentPartPath;
	std::string displayName;
	std::vector<float> triangleSoup;
};

struct PolylineParams
{
	bool closedPreserveEndpoint = false;
};

struct IntersectionHit
{
	Point3d positionMm;
	double paramOnEdge1 = 0.0;
	double paramOnEdge2 = 0.0;
	double paramU = 0.0;
	double paramV = 0.0;
};

struct IntersectionResult
{
	std::vector<IntersectionHit> points;
	std::vector<Polyline3d> curves;
	double maxResidualMm = 0.0;
};

struct IntersectionParams
{
	double toleranceMm = 1e-3;
	bool discretizeCurves = true;
	TessellateParams curveDisc;
};

enum class MeshBooleanOp
{
	Difference,
	Union,
	Intersection
};

enum class BrepBooleanOp
{
	Fuse,
	Common,
	Cut
};

} // namespace geoalgo
