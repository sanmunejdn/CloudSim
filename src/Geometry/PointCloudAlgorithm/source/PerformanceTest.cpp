/// @file PerformanceTest.cpp
/// @brief PerformanceTest 实现

#include "PerformanceTest.h"

#include "Downsample.h"
#include "Measure.h"
#include "ParallelUtils.h"
#include "PointCloudBuffer.h"
#include "Preprocess.h"
#include "Reconstruction.h"
#include "ReconstructionConfig.h"

#include <iostream>
#include <random>

namespace pclalgo
{
namespace
{
std::vector<float> generateRandomPointCloud(std::size_t pointCount, double size = 100.0)
{
	std::vector<float> xyz;
	xyz.reserve(pointCount * 3U);

	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<float> dis(0.0f, static_cast<float>(size));

	for (std::size_t i = 0; i < pointCount; ++i)
	{
		xyz.push_back(dis(gen));
		xyz.push_back(dis(gen));
		xyz.push_back(dis(gen));
	}
	return xyz;
}

template <typename Func>
double measureExecutionTime(Func func)
{
	auto start = std::chrono::high_resolution_clock::now();
	func();
	auto end = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double, std::milli> duration = end - start;
	return duration.count();
}

} // namespace

bool runPerformanceTest(std::vector<PerformanceResult>& results, std::string* errMsg)
{
	results.clear();

	try
	{
		// 测试1: 小点云法线估计 (1万点)
		{
			PerformanceResult result;
			result.testName = "Small point cloud normal estimation (10K points)";

			auto xyz = generateRandomPointCloud(10000);
			std::vector<float> normals;

			result.executionTimeMs = measureExecutionTime([&]() { estimateNormalsPca(xyz, normals, 12); });
			result.success = !normals.empty();
			results.push_back(result);
		}

		// 测试2: 中点云法线估计 (10万点)
		{
			PerformanceResult result;
			result.testName = "Medium point cloud normal estimation (100K points)";

			auto xyz = generateRandomPointCloud(100000);
			std::vector<float> normals;

			result.executionTimeMs = measureExecutionTime([&]() { estimateNormalsPca(xyz, normals, 12); });
			result.success = !normals.empty();
			results.push_back(result);
		}

		// 测试3: 大点云法线估计 (50万点)
		{
			PerformanceResult result;
			result.testName = "Large point cloud normal estimation (500K points)";

			auto xyz = generateRandomPointCloud(500000);
			std::vector<float> normals;

			result.executionTimeMs = measureExecutionTime([&]() { estimateNormalsPca(xyz, normals, 12); });
			result.success = !normals.empty();
			results.push_back(result);
		}

		// 测试4: 小点云离群移除 (1万点)
		{
			PerformanceResult result;
			result.testName = "Small point cloud outlier removal (10K points)";

			auto xyz = generateRandomPointCloud(10000);

			result.executionTimeMs =
				measureExecutionTime([&]() { removeOutliers(xyz, 5.0, 24, nullptr, nullptr, nullptr); });
			result.success = !xyz.empty();
			results.push_back(result);
		}

		// 测试5: 中点云离群移除 (10万点)
		{
			PerformanceResult result;
			result.testName = "Medium point cloud outlier removal (100K points)";

			auto xyz = generateRandomPointCloud(100000);

			result.executionTimeMs =
				measureExecutionTime([&]() { removeOutliers(xyz, 5.0, 24, nullptr, nullptr, nullptr); });
			result.success = !xyz.empty();
			results.push_back(result);
		}

		// 测试6: 小点云Poisson重建 (1万点)
		{
			PerformanceResult result;
			result.testName = "Small point cloud Poisson reconstruction (10K points)";

			auto xyz = generateRandomPointCloud(10000);
			std::vector<float> normals;
			estimateNormalsPca(xyz, normals, 12);
			orientNormalsMst(xyz, normals, 12, nullptr, nullptr);

			std::vector<float> soup;
			std::string err;
			result.executionTimeMs =
				measureExecutionTime([&]() { reconstructPoisson(xyz, normals, soup, 0.0, 20.0, 30.0, 0.375, &err); });
			result.success = !soup.empty();
			if (!result.success)
			{
				result.errorMessage = err;
			}
			results.push_back(result);
		}

		// 测试7: 中点云Poisson重建 (5万点)
		{
			PerformanceResult result;
			result.testName = "Medium point cloud Poisson reconstruction (50K points)";

			auto xyz = generateRandomPointCloud(50000);
			std::vector<float> normals;
			estimateNormalsPca(xyz, normals, 12);
			orientNormalsMst(xyz, normals, 12, nullptr, nullptr);

			std::vector<float> soup;
			std::string err;
			result.executionTimeMs =
				measureExecutionTime([&]() { reconstructPoisson(xyz, normals, soup, 0.0, 20.0, 30.0, 0.375, &err); });
			result.success = !soup.empty();
			if (!result.success)
			{
				result.errorMessage = err;
			}
			results.push_back(result);
		}

		// 测试8: 配置版本API (自动下采样)
		{
			PerformanceResult result;
			result.testName = "Config API with auto downsampling (100K points)";

			auto xyz = generateRandomPointCloud(100000);

			ReconstructionConfig config;
			config.quality = ReconstructionQuality::Fast;
			config.maxPointsForReconstruction = 50000; // 强制下采样
			config.enableParallel = true;

			std::vector<float> soup;
			std::string err;
			result.executionTimeMs =
				measureExecutionTime([&]() { reconstructPoissonAutoWithConfig(xyz, soup, config, &err); });
			result.success = !soup.empty();
			if (!result.success)
			{
				result.errorMessage = err;
			}
			results.push_back(result);
		}

		// 输出结果
		std::cout << "\n=== Performance Test Results ===\n";
		std::cout << "TBB Available: " << (ParallelUtils::isTbbAvailable() ? "Yes" : "No") << "\n";
		std::cout << "Thread Count: " << ParallelUtils::getThreadCount() << "\n\n";

		for (const auto& result : results)
		{
			std::cout << result.testName << ": " << result.executionTimeMs << " ms";
			if (!result.success)
			{
				std::cout << " [FAILED]";
				if (!result.errorMessage.empty())
				{
					std::cout << " - " << result.errorMessage;
				}
			}
			std::cout << "\n";
		}

		return true;
	}
	catch (const std::exception& e)
	{
		if (errMsg != nullptr)
		{
			*errMsg = std::string("Performance test failed: ") + e.what();
		}
		return false;
	}
}

} // namespace pclalgo