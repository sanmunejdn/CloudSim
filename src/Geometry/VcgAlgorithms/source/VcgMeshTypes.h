#pragma once

// 内部头文件：vcglib mesh 类型定义
// 仅在 VcgAlgorithms 模块内部 .cpp 中 include

#include <string>
#include <vector>

#include <vcg/complex/complex.h>
#include <vcg/math/quadric.h>

namespace vcgalgo
{

class VcgVertex;
class VcgFace;

struct VcgUsedTypes : public vcg::UsedTypes<
	vcg::Use<VcgVertex>::AsVertexType,
	vcg::Use<VcgFace>::AsFaceType>
{};

// 顶点：含 VFAdj（简化）、Qualityd（重网格，与 Coord3d 标量一致）、内置 Quadric（QEM 简化）
class VcgVertex : public vcg::Vertex<VcgUsedTypes,
	vcg::vertex::VFAdj,
	vcg::vertex::Coord3d,
	vcg::vertex::Normal3d,
	vcg::vertex::Mark,
	vcg::vertex::Qualityd,
	vcg::vertex::BitFlags>
{
public:
	vcg::math::Quadric<double>& Qd() { return q_; }
	const vcg::math::Quadric<double>& Qd() const { return q_; }

private:
	vcg::math::Quadric<double> q_;
};

class VcgFace : public vcg::Face<VcgUsedTypes,
	vcg::face::VFAdj,
	vcg::face::FFAdj,
	vcg::face::VertexRef,
	vcg::face::Normal3d,
	vcg::face::Qualityd,
	vcg::face::BitFlags>
{};

using VcgMesh = vcg::tri::TriMesh<std::vector<VcgVertex>, std::vector<VcgFace>>;

namespace internal
{

bool soupToVcgMesh(const std::vector<float>& soup, VcgMesh& mesh, std::string* errMsg);
void vcgMeshToSoup(const VcgMesh& mesh, std::vector<float>& soup);
VcgMesh createEmptyVcgMesh();

} // namespace internal

} // namespace vcgalgo
