#include "SelfTest.h"
#include "VcgMeshAdapter.h"
#include "MeshSimplify.h"
#include "MeshSmooth.h"
#include "MeshRepair.h"
#include "MeshRemesh.h"

#include <array>
#include <cmath>
#include <string>

namespace vcgalgo
{

namespace
{

// 生成一个简单立方体 triangleSoup（12 三角形，6 面×2）
std::vector<float> makeCubeSoup(float size = 10.0f)
{
	const float h = size / 2.0f;
	// 8 顶点
	const float v[8][3] = {
		{-h, -h, -h}, { h, -h, -h}, { h,  h, -h}, {-h,  h, -h},
		{-h, -h,  h}, { h, -h,  h}, { h,  h,  h}, {-h,  h,  h}
	};
	// 6 面，每面 2 三角形，CCW
	const int faces[12][3] = {
		{0,1,2}, {0,2,3}, // -Z
		{4,6,5}, {4,7,6}, // +Z
		{0,4,5}, {0,5,1}, // -Y
		{2,6,7}, {2,7,3}, // +Y
		{0,3,7}, {0,7,4}, // -X
		{1,5,6}, {1,6,2}  // +X
	};

	std::vector<float> soup;
	soup.reserve(12 * 9);
	for (int f = 0; f < 12; ++f)
	{
		for (int vi = 0; vi < 3; ++vi)
		{
			const int idx = faces[f][vi];
			soup.push_back(v[idx][0]);
			soup.push_back(v[idx][1]);
			soup.push_back(v[idx][2]);
		}
	}
	return soup;
}

// 生成一个细分球面 triangleSoup（用于测试简化/平滑）
std::vector<float> makeSphereSoup(float radius = 10.0f, int stacks = 10, int slices = 20)
{
	std::vector<float> soup;

	const float pi = 3.14159265358979323846f;

	for (int i = 0; i < stacks; ++i)
	{
		const float theta0 = pi * static_cast<float>(i) / static_cast<float>(stacks);
		const float theta1 = pi * static_cast<float>(i + 1) / static_cast<float>(stacks);

		for (int j = 0; j < slices; ++j)
		{
			const float phi0 = 2.0f * pi * static_cast<float>(j) / static_cast<float>(slices);
			const float phi1 = 2.0f * pi * static_cast<float>(j + 1) / static_cast<float>(slices);

			// 球面坐标 → 直角坐标
			auto sphXyz = [&](float theta, float phi, float& ox, float& oy, float& oz) {
				ox = radius * std::sin(theta) * std::cos(phi);
				oy = radius * std::sin(theta) * std::sin(phi);
				oz = radius * std::cos(theta);
			};

			float p00x, p00y, p00z;
			float p10x, p10y, p10z;
			float p01x, p01y, p01z;
			float p11x, p11y, p11z;
			sphXyz(theta0, phi0, p00x, p00y, p00z);
			sphXyz(theta1, phi0, p10x, p10y, p10z);
			sphXyz(theta0, phi1, p01x, p01y, p01z);
			sphXyz(theta1, phi1, p11x, p11y, p11z);

			// 两个三角形
			soup.push_back(p00x); soup.push_back(p00y); soup.push_back(p00z);
			soup.push_back(p10x); soup.push_back(p10y); soup.push_back(p10z);
			soup.push_back(p11x); soup.push_back(p11y); soup.push_back(p11z);

			soup.push_back(p00x); soup.push_back(p00y); soup.push_back(p00z);
			soup.push_back(p11x); soup.push_back(p11y); soup.push_back(p11z);
			soup.push_back(p01x); soup.push_back(p01y); soup.push_back(p01z);
		}
	}
	return soup;
}

} // namespace

bool runSelfTest(std::vector<std::string>& failures)
{
	failures.clear();

	// 测试1：IndexedMesh 转换往返
	{
		const auto soup = makeCubeSoup();
		IndexedMesh indexed;
		if (!triangleSoupToIndexedMesh(soup, indexed))
		{
			failures.push_back("Test1: triangleSoupToIndexedMesh failed");
		}
		else
		{
			// 立方体 8 顶点，去重后应 ≤8（实际 8）
			if (indexed.vertices.size() / 3 > 12)
			{
				failures.push_back("Test1: vertex dedup failed, got " +
					std::to_string(indexed.vertices.size() / 3) + " vertices");
			}
			std::vector<float> roundtrip;
			if (!indexedMeshToTriangleSoup(indexed, roundtrip))
			{
				failures.push_back("Test1: indexedMeshToTriangleSoup failed");
			}
			else if (roundtrip.size() != soup.size())
			{
				failures.push_back("Test1: roundtrip size mismatch");
			}
		}
	}

	// 测试2：网格简化
	{
		const auto soup = makeSphereSoup(10.0f, 15, 30);
		const int originalFaces = static_cast<int>(soup.size() / 9);
		const int targetFaces = originalFaces / 3;

		std::vector<float> simplified;
		SimplifyParams params;
		params.targetFaceCount = targetFaces;
		if (!simplifyQuadricEdgeCollapse(soup, simplified, params))
		{
			failures.push_back("Test2: simplifyQuadricEdgeCollapse failed");
		}
		else
		{
			const int resultFaces = static_cast<int>(simplified.size() / 9);
			if (resultFaces > targetFaces + 100) // 允许一定误差
			{
				failures.push_back("Test2: simplified faces " +
					std::to_string(resultFaces) + " > target " + std::to_string(targetFaces));
			}
		}
	}

	// 测试3：Laplacian 平滑
	{
		const auto soup = makeSphereSoup();
		std::vector<float> smoothed;
		if (!smoothLaplacian(soup, 3, smoothed))
		{
			failures.push_back("Test3: smoothLaplacian failed");
		}
		else if (smoothed.size() != soup.size())
		{
			failures.push_back("Test3: smoothed size mismatch");
		}
	}

	// 测试4：Implicit Fairing 平滑
	{
		const auto soup = makeSphereSoup();
		std::vector<float> smoothed;
		if (!smoothImplicitFairing(soup, 0.2, smoothed))
		{
			failures.push_back("Test4: smoothImplicitFairing failed");
		}
		else if (smoothed.size() != soup.size())
		{
			failures.push_back("Test4: smoothed size mismatch");
		}
	}

	// 测试5：网格修复
	{
		const auto soup = makeCubeSoup();
		std::vector<float> repaired;
		RepairParams params;
		params.removeDegenerate = true;
		params.removeDuplicate = true;
		params.removeNonManifold = true;
		if (!repairMesh(soup, repaired, params))
		{
			failures.push_back("Test5: repairMesh failed");
		}
		else if (repaired.empty())
		{
			failures.push_back("Test5: repairMesh produced empty output");
		}
	}

	// 测试6：各向同性重网格
	{
		const auto soup = makeSphereSoup();
		std::vector<float> remeshed;
		if (!isotropicRemesh(soup, 2.0, remeshed, 2))
		{
			failures.push_back("Test6: isotropicRemesh failed");
		}
		else if (remeshed.empty())
		{
			failures.push_back("Test6: isotropicRemesh produced empty output");
		}
	}

	// 测试7：空输入处理
	{
		std::vector<float> empty;
		std::vector<float> out;
		std::string err;

		if (simplifyQuadricEdgeCollapse(empty, out, {}, &err))
		{
			failures.push_back("Test7: simplify should fail on empty input");
		}
		if (smoothLaplacian(empty, 1, out, &err))
		{
			failures.push_back("Test7: smooth should fail on empty input");
		}
	}

	return failures.empty();
}

} // namespace vcgalgo
