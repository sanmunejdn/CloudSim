#ifndef VCGALGORITHMS_MESHDEFECTDETECT_H
#define VCGALGORITHMS_MESHDEFECTDETECT_H

/// @file MeshDefectDetect.h
/// @brief MeshDefectDetect 接口

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

	double score = 0.0;
};

struct DefectDetectParams

{
	double sensitivity = 0.08;

	int minClusterFaces = 3;

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

VCg_ALGORITHMS_API bool detectMeshDefects(

	const std::vector<float>& triangleSoup,

	DefectDetectReport& report,

	const DefectDetectParams& params = {},

	std::string* errMsg = nullptr);

} // namespace vcgalgo

#endif // VCGALGORITHMS_MESHDEFECTDETECT_H
