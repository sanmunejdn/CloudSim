#pragma once

#include "vcg_algorithms_global.h"

#include <string>
#include <vector>

namespace vcgalgo
{

// 索引化 mesh 表示，vcglib 模板类型不暴露到头文件
struct VCg_ALGORITHMS_API IndexedMesh
{
	std::vector<float> vertices;  // 3*N，xyz
	std::vector<int> faces;       // 3*F，顶点索引
};

// triangleSoup → IndexedMesh（去重顶点）
VCg_ALGORITHMS_API bool triangleSoupToIndexedMesh(
	const std::vector<float>& soup,
	IndexedMesh& out,
	std::string* errMsg = nullptr);

// IndexedMesh → triangleSoup
VCg_ALGORITHMS_API bool indexedMeshToTriangleSoup(
	const IndexedMesh& mesh,
	std::vector<float>& outSoup);

} // namespace vcgalgo
