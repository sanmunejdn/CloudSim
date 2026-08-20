#ifndef VCGALGORITHMS_MESHDEFECTDETECT_H
#define VCGALGORITHMS_MESHDEFECTDETECT_H

/// @file MeshDefectDetect.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 网格缺陷检测：针状三角 / 高曲率突起 / 边界尖刺（只读报告）

#include "vcg_algorithms_global.h"

#include <cstdint>
#include <string>
#include <vector>

namespace vcgalgo
{
enum class MeshDefectKind : std::uint8_t
{
	NeedleTriangle = 0,
	HighCurvatureProtrusion = 1,
	BoundarySpike = 2,
	Unknown = 3
};

struct MeshDefectFace
{
	int faceIndex = -1;
	MeshDefectKind kind = MeshDefectKind::Unknown;
	double score = 0.0; ///< 越大越可疑
};

struct DefectDetectParams
{
	double sensitivity = 0.08; ///< 灵敏度；越大检出越多
	int minClusterFaces = 3;   ///< 缺陷簇最少面数
	bool detectNeedle = true;
	bool detectProtrusion = true;
	bool detectBoundarySpike = true;
};

struct DefectDetectReport
{
	int totalFaces = 0;
	int defectFaceCount = 0;
	double defectAreaRatio = 0.0;
	int needleCount = 0;
	int protrusionCount = 0;
	int boundarySpikeCount = 0;
	std::vector<MeshDefectFace> defects;
};

/**
 * 多信号缺陷检测，不修改输入
 * @return false：soup 非法（非 9 倍数或空）
 */
VCg_ALGORITHMS_API bool detectMeshDefects(const std::vector<float>& triangleSoup, DefectDetectReport& report,
										  const DefectDetectParams& params = {}, std::string* errMsg = nullptr);

} // namespace vcgalgo

#endif // VCGALGORITHMS_MESHDEFECTDETECT_H
