#ifndef GEOMETRYALGORITHM_TEMPLATEBREPREGISTRATION_H
#define GEOMETRYALGORITHM_TEMPLATEBREPREGISTRATION_H

/// @file TemplateBrepRegistration.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 反向点-面 ICP：固定扫描点，迭代变换模板 shape 贴齐扫描

#include "geometry_algorithm_global.h"

#include "ShapeHandle.h"

#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Geometry>

namespace geoalgo
{
struct TemplateBrepRegistrationParams
{
	double maxPairMm = 0.0;              ///< 配对距离上限（mm）；0=自动
	int maxIterations = 30;
	std::size_t icpMaxPoints = 8000U;
	double normalGateDeg = 0.0;          ///< 法向门限（°）；0=禁用
	double convergenceTransMm = 0.005;
};

/**
 * 反向点-面 ICP：固定扫描，变换模板 B-rep 投影对应点
 * @param outTemplateToScan 模板→扫描刚体（世界系增量由 Host 写入 worldMatrix）
 * @return false：扫描 buffer 无效、模板访问失败、对应点过少或 ICP 步失败
 */
GEOMETRY_ALGORITHM_API bool
rigidRegisterTemplateToScanPointToPlane(const std::vector<float>& scanXyz, const std::vector<float>& scanNormals,
										const ShapeHandle& originalTemplateShape, ShapeHandle& outAlignedTemplateShape,
										Eigen::Isometry3d& outTemplateToScan, double& outRmseMm,
										const TemplateBrepRegistrationParams& params, std::string* errMsg = nullptr);

/**
 * 对 ShapeHandle 应用刚体变换（内部 ICP 临时体与单测）
 * @return false：shape 访问失败
 */
GEOMETRY_ALGORITHM_API bool applyIsometryToShapeHandle(const ShapeHandle& shape, const Eigen::Isometry3d& transform,
													   ShapeHandle& outShape, std::string* errMsg = nullptr);

/** 扫描点到 shape 表面最大/平均距离（mm），用于预览门控 */
GEOMETRY_ALGORITHM_API double measureScanToShapeMaxDistanceMm(const std::vector<float>& scanXyz,
															  const ShapeHandle& shape, double& outAvgDistMm);

} // namespace geoalgo

#endif // GEOMETRYALGORITHM_TEMPLATEBREPREGISTRATION_H
