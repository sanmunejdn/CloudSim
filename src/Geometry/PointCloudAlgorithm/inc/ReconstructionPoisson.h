#ifndef POINTCLOUDALGORITHM_RECONSTRUCTIONPOISSON_H
#define POINTCLOUDALGORITHM_RECONSTRUCTIONPOISSON_H

/// @file ReconstructionPoisson.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief Poisson 隐式重建：定向点云 → 水密倾向三角 soup（mm）

#include "point_cloud_algorithm_global.h"

#include <string>
#include <vector>

namespace pclalgo
{
/**
 * CGAL poisson_surface_reconstruction_delaunay：将定向点云作为泊松约束求指示函数并提取等值面
 * 强依赖法线方向；开口扫描或法线错误易空洞/翻面
 * @param xyz 3*N float，mm
 * @param normalsNxNyNz 3*N，与 xyz 同序
 * @param triangleSoupOut 9*T float，每三角 3 顶点 xyz
 * @param spacingMm 八叉树/体素尺度；≤0 时用平均点距（k=6），仍≤0 则兜底 1.0
 * @param smAngleDeg 表面平滑角度阈值，默认 20°
 * @param smRadiusRel 平滑半径（相对 spacing），默认 30
 * @param smDistanceRel 平滑距离（相对 spacing），默认 0.375
 * @return false：xyz/法线长度不符、点数<3、CGAL 失败或输出空（errMsg）
 */
POINT_CLOUD_ALGORITHM_API bool reconstructPoisson(const std::vector<float>& xyz,
												  const std::vector<float>& normalsNxNyNz,
												  std::vector<float>& triangleSoupOut, double spacingMm = 0.0,
												  double smAngleDeg = 20.0, double smRadiusRel = 30.0,
												  double smDistanceRel = 0.375, std::string* errMsg = nullptr);

/**
 * 一键 Poisson：preprocessForReconstruction → reconstructPoisson（默认平滑参数）
 * @param xyz 按值传入，内部可改写（体素/离群）
 * @param voxelPrefilterMm 体素预滤波边长 mm；≤0 跳过
 * @param outlierRemovalPercent 离群剔除百分比，默认 5
 * @return false：预处理失败或 Poisson 失败
 */
POINT_CLOUD_ALGORITHM_API bool reconstructPoissonAuto(std::vector<float> xyz, std::vector<float>& triangleSoupOut,
													  double voxelPrefilterMm, double outlierRemovalPercent = 5.0,
													  std::string* errMsg = nullptr);

} // namespace pclalgo

#endif // POINTCLOUDALGORITHM_RECONSTRUCTIONPOISSON_H
