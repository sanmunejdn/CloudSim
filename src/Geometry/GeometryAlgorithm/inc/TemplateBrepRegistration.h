#pragma once

#include "geometry_algorithm_global.h"
#include "ShapeHandle.h"

#include <Eigen/Geometry>

#include <cstddef>
#include <string>
#include <vector>

namespace geoalgo
{

struct TemplateBrepRegistrationParams
{
	double maxPairMm = 0.0;
	int maxIterations = 30;
	std::size_t icpMaxPoints = 8000U;
	double normalGateDeg = 0.0;
	double convergenceTransMm = 0.005;
};

/// 反向点-面 ICP：固定扫描点，迭代变换模板 shape 贴齐扫描
GEOMETRY_ALGORITHM_API bool rigidRegisterTemplateToScanPointToPlane(
	const std::vector<float>& scanXyz,
	const std::vector<float>& scanNormals,
	const ShapeHandle& originalTemplateShape,
	ShapeHandle& outAlignedTemplateShape,
	Eigen::Isometry3d& outTemplateToScan,
	double& outRmseMm,
	const TemplateBrepRegistrationParams& params,
	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool applyIsometryToShapeHandle(
	const ShapeHandle& shape,
	const Eigen::Isometry3d& transform,
	ShapeHandle& outShape,
	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API double measureScanToShapeMaxDistanceMm(
	const std::vector<float>& scanXyz,
	const ShapeHandle& shape,
	double& outAvgDistMm);

} // namespace geoalgo
