#ifndef VCGALGORITHMS_VCGMESHADAPTER_H
#define VCGALGORITHMS_VCGMESHADAPTER_H

/// @file VcgMeshAdapter.h
/// @brief soup ↔ 索引化 mesh 转换（不对外暴露 vcglib 类型）

#include "vcg_algorithms_global.h"

#include <string>
#include <vector>

namespace vcgalgo
{
struct VCg_ALGORITHMS_API IndexedMesh
{
	std::vector<float> vertices; ///< 3*N，xyz mm
	std::vector<int> faces;		 ///< 3*F，顶点索引
};

/**
 * triangleSoup（9*T）→ 焊点 IndexedMesh
 * @return false：soup 非法（非 9 倍数或空）
 */
VCg_ALGORITHMS_API bool triangleSoupToIndexedMesh(const std::vector<float>& soup, IndexedMesh& out,
												  std::string* errMsg = nullptr);

/** IndexedMesh → 展开 triangleSoup；空 mesh 得空 soup */
VCg_ALGORITHMS_API bool indexedMeshToTriangleSoup(const IndexedMesh& mesh, std::vector<float>& outSoup);

} // namespace vcgalgo

#endif // VCGALGORITHMS_VCGMESHADAPTER_H
