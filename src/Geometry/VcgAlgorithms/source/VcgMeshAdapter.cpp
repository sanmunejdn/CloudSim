#include "VcgMeshAdapter.h"
#include "VcgMeshTypes.h"

#include <unordered_map>
#include <cmath>
#include <cstdint>

namespace vcgalgo
{

// 量化 key 用于顶点去重
struct QuantizedKey
{
	int64_t x, y, z;

	bool operator==(const QuantizedKey& o) const
	{
		return x == o.x && y == o.y && z == o.z;
	}
};

struct QuantizedKeyHash
{
	std::size_t operator()(const QuantizedKey& k) const
	{
		// FNV-1a
		std::size_t h = 14695981039346656037ULL;
		auto mix = [&](int64_t v) {
			const auto* p = reinterpret_cast<const uint8_t*>(&v);
			for (int i = 0; i < 8; ++i)
			{
				h ^= static_cast<std::size_t>(p[i]);
				h *= 1099511628211ULL;
			}
		};
		mix(k.x);
		mix(k.y);
		mix(k.z);
		return h;
	}
};

// 量化精度：0.001mm（微米级）
static constexpr double kQuantizeScale = 1000.0;

static QuantizedKey quantize(double x, double y, double z)
{
	return {
		static_cast<int64_t>(std::round(x * kQuantizeScale)),
		static_cast<int64_t>(std::round(y * kQuantizeScale)),
		static_cast<int64_t>(std::round(z * kQuantizeScale))
	};
}

bool triangleSoupToIndexedMesh(
	const std::vector<float>& soup,
	IndexedMesh& out,
	std::string* errMsg)
{
	if (soup.empty() || soup.size() % 9 != 0)
	{
		if (errMsg) *errMsg = "invalid triangle soup size";
		return false;
	}

	const std::size_t triCount = soup.size() / 9;
	out.vertices.clear();
	out.faces.clear();
	out.vertices.reserve(triCount * 3); // 最坏情况无去重
	out.faces.reserve(triCount * 3);

	std::unordered_map<QuantizedKey, int, QuantizedKeyHash> vertexMap;
	vertexMap.reserve(triCount * 3);

	for (std::size_t t = 0; t < triCount; ++t)
	{
		const std::size_t base = t * 9;
		for (int v = 0; v < 3; ++v)
		{
			const std::size_t off = base + v * 3;
			const float x = soup[off];
			const float y = soup[off + 1];
			const float z = soup[off + 2];

			const auto key = quantize(x, y, z);
			auto it = vertexMap.find(key);
			if (it != vertexMap.end())
			{
				out.faces.push_back(it->second);
			}
			else
			{
				const int idx = static_cast<int>(out.vertices.size() / 3);
				out.vertices.push_back(x);
				out.vertices.push_back(y);
				out.vertices.push_back(z);
				out.faces.push_back(idx);
				vertexMap.emplace(key, idx);
			}
		}
	}

	return true;
}

bool indexedMeshToTriangleSoup(
	const IndexedMesh& mesh,
	std::vector<float>& outSoup)
{
	if (mesh.vertices.empty() || mesh.faces.empty())
	{
		outSoup.clear();
		return false;
	}

	const std::size_t faceCount = mesh.faces.size() / 3;
	outSoup.clear();
	outSoup.reserve(faceCount * 9);

	for (std::size_t f = 0; f < faceCount; ++f)
	{
		for (int v = 0; v < 3; ++v)
		{
			const int vi = mesh.faces[f * 3 + v];
			const std::size_t off = static_cast<std::size_t>(vi) * 3;
			outSoup.push_back(mesh.vertices[off]);
			outSoup.push_back(mesh.vertices[off + 1]);
			outSoup.push_back(mesh.vertices[off + 2]);
		}
	}

	return true;
}

// 内部：IndexedMesh → VcgMesh
static bool indexedMeshToVcgMesh(const IndexedMesh& in, VcgMesh& mesh)
{
	mesh.Clear();
	if (in.vertices.empty() || in.faces.empty()) return false;

	const std::size_t vertCount = in.vertices.size() / 3;
	const std::size_t faceCount = in.faces.size() / 3;

	mesh.vert.reserve(vertCount);
	mesh.face.reserve(faceCount);

	// 添加顶点
	for (std::size_t i = 0; i < vertCount; ++i)
	{
		auto vi = vcg::tri::Allocator<VcgMesh>::AddVertex(mesh,
			VcgMesh::CoordType(in.vertices[i * 3], in.vertices[i * 3 + 1], in.vertices[i * 3 + 2]));
		(void)vi;
	}

	// 添加面
	for (std::size_t f = 0; f < faceCount; ++f)
	{
		const int i0 = in.faces[f * 3];
		const int i1 = in.faces[f * 3 + 1];
		const int i2 = in.faces[f * 3 + 2];
		if (i0 < 0 || i1 < 0 || i2 < 0 ||
			static_cast<std::size_t>(i0) >= vertCount ||
			static_cast<std::size_t>(i1) >= vertCount ||
			static_cast<std::size_t>(i2) >= vertCount)
		{
			continue;
		}
		vcg::tri::Allocator<VcgMesh>::AddFace(mesh,
			&mesh.vert[i0], &mesh.vert[i1], &mesh.vert[i2]);
	}

	mesh.vert.shrink_to_fit();
	mesh.face.shrink_to_fit();
	return mesh.face.size() > 0;
}

// 内部：VcgMesh → IndexedMesh
static void vcgMeshToIndexedMesh(const VcgMesh& mesh, IndexedMesh& out)
{
	out.vertices.clear();
	out.faces.clear();

	if (mesh.vert.empty()) return;

	out.vertices.reserve(mesh.vert.size() * 3);
	for (const auto& v : mesh.vert)
	{
		out.vertices.push_back(static_cast<float>(v.P()[0]));
		out.vertices.push_back(static_cast<float>(v.P()[1]));
		out.vertices.push_back(static_cast<float>(v.P()[2]));
	}

	out.faces.reserve(mesh.face.size() * 3);
	for (const auto& f : mesh.face)
	{
		if (f.IsD()) continue; // 跳过已删除面
		for (int j = 0; j < 3; ++j)
		{
			out.faces.push_back(static_cast<int>(vcg::tri::Index(mesh, f.V(j))));
		}
	}
}

// 声明为内部可见，供其他 .cpp 使用
// 通过链接单元内部头文件暴露（不写到公开 .h）
namespace internal
{

bool soupToVcgMesh(const std::vector<float>& soup, VcgMesh& mesh, std::string* errMsg)
{
	IndexedMesh indexed;
	if (!triangleSoupToIndexedMesh(soup, indexed, errMsg)) return false;
	return indexedMeshToVcgMesh(indexed, mesh);
}

void vcgMeshToSoup(const VcgMesh& mesh, std::vector<float>& soup)
{
	IndexedMesh indexed;
	vcgMeshToIndexedMesh(mesh, indexed);
	indexedMeshToTriangleSoup(indexed, soup);
}

VcgMesh createEmptyVcgMesh()
{
	return VcgMesh{};
}

} // namespace internal

} // namespace vcgalgo
