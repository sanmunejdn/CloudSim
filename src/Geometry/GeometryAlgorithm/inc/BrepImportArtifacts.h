#pragma once

#include "geometry_algorithm_global.h"
#include "ShapeHandle.h"

#include <memory>
#include <string>
#include <vector>

namespace geoalgo
{

/// BREP 导入预处理：显示/拾取/线框共用，避免重复 OCCT 离散
struct GEOMETRY_ALGORITHM_API BrepImportArtifacts
{
	std::vector<float> displaySoup;
	std::vector<int> triangleFaceIndex;
	std::vector<std::vector<float>> faceSoups;
	std::vector<std::vector<float>> edgePolylines;
	std::vector<std::vector<int>> faceEdgeIndices;
};

GEOMETRY_ALGORITHM_API bool buildBrepImportArtifacts(
	const ShapeHandle& shape,
	BrepImportArtifacts& out,
	std::string* errMsg = nullptr);

/// 按 ShapeHandle 共享 identity 缓存；命中则零成本返回
GEOMETRY_ALGORITHM_API std::shared_ptr<const BrepImportArtifacts> getOrBuildBrepImportArtifacts(
	const ShapeHandle& shape,
	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API void clearBrepImportArtifactsCache();

} // namespace geoalgo
