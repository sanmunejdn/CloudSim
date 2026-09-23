/// @file SelfTest.cpp
/// @brief SelfTest 实现

#include "SelfTest.h"

#include "Crop.h"
#include "Downsample.h"
#include "Measure.h"
#include "ParallelUtils.h"
#include "PointCloudBuffer.h"
#include "Preprocess.h"
#include "Reconstruction.h"
#include "ReconstructionConfig.h"
#include "RegistrationGlobal.h"
#include "RegistrationGlobalPcl.h"
#include "RegistrationNonRigid.h"
#include "RegistrationRigid.h"
#include "RegistrationSpare.h"
#include "RegistrationSdf.h"
#include "AdaptiveRemesh.h"
#include "Transform.h"

#include <cmath>
#include <sstream>

#include <Eigen/Geometry>

namespace pclalgo
{
namespace
{
void expectTrue(std::vector<std::string>& failures, const char* name, const bool value)
{
	if (!value)
	{
		failures.push_back(std::string(name) + " expected true");
	}
}

void expectNear(std::vector<std::string>& failures, const char* name, const double actual, const double expected,
				const double eps)
{
	if (!std::isfinite(actual) || std::fabs(actual - expected) > eps)
	{
		std::ostringstream oss;
		oss << name << " expected " << expected << " got " << actual;
		failures.push_back(oss.str());
	}
}

std::vector<float> makePlanePointCloud(const std::size_t grid, const double z)
{
	std::vector<float> xyz;
	xyz.reserve(grid * grid * 3U);
	for (std::size_t i = 0; i < grid; ++i)
	{
		for (std::size_t j = 0; j < grid; ++j)
		{
			xyz.push_back(static_cast<float>(i));
			xyz.push_back(static_cast<float>(j));
			xyz.push_back(static_cast<float>(z));
		}
	}
	return xyz;
}

/// 左半平坦、右半高频起伏的高度场面片，用于验证自适应边长
std::vector<float> makeWavyPlateSoup(const int n, const double spacing)
{
	std::vector<float> soup;
	soup.reserve(static_cast<std::size_t>((n - 1) * (n - 1) * 2 * 9));
	auto height = [&](const int i, const int j) -> double
	{
		const double x = i * spacing;
		const double y = j * spacing;
		if (x < 0.5 * (n - 1) * spacing)
		{
			return 0.0;
		}
		return 1.2 * std::sin(x * 3.5) * std::cos(y * 3.5);
	};
	for (int i = 0; i < n - 1; ++i)
	{
		for (int j = 0; j < n - 1; ++j)
		{
			const double x00 = i * spacing;
			const double y00 = j * spacing;
			const double x10 = (i + 1) * spacing;
			const double y10 = j * spacing;
			const double x01 = i * spacing;
			const double y01 = (j + 1) * spacing;
			const double x11 = (i + 1) * spacing;
			const double y11 = (j + 1) * spacing;
			const double z00 = height(i, j);
			const double z10 = height(i + 1, j);
			const double z01 = height(i, j + 1);
			const double z11 = height(i + 1, j + 1);
			auto pushTri = [&](double ax, double ay, double az, double bx, double by, double bz, double cx, double cy,
							   double cz)
			{
				soup.push_back(static_cast<float>(ax));
				soup.push_back(static_cast<float>(ay));
				soup.push_back(static_cast<float>(az));
				soup.push_back(static_cast<float>(bx));
				soup.push_back(static_cast<float>(by));
				soup.push_back(static_cast<float>(bz));
				soup.push_back(static_cast<float>(cx));
				soup.push_back(static_cast<float>(cy));
				soup.push_back(static_cast<float>(cz));
			};
			pushTri(x00, y00, z00, x10, y10, z10, x11, y11, z11);
			pushTri(x00, y00, z00, x11, y11, z11, x01, y01, z01);
		}
	}
	return soup;
}

std::vector<float> makeFlatPlateSoup(const int n, const double spacing)
{
	std::vector<float> soup;
	soup.reserve(static_cast<std::size_t>((n - 1) * (n - 1) * 2 * 9));
	for (int i = 0; i < n - 1; ++i)
	{
		for (int j = 0; j < n - 1; ++j)
		{
			const float x00 = static_cast<float>(i * spacing);
			const float y00 = static_cast<float>(j * spacing);
			const float x10 = static_cast<float>((i + 1) * spacing);
			const float y10 = static_cast<float>(j * spacing);
			const float x01 = static_cast<float>(i * spacing);
			const float y01 = static_cast<float>((j + 1) * spacing);
			const float x11 = static_cast<float>((i + 1) * spacing);
			const float y11 = static_cast<float>((j + 1) * spacing);
			auto pushTri = [&](float ax, float ay, float bx, float by, float cx, float cy)
			{
				soup.push_back(ax);
				soup.push_back(ay);
				soup.push_back(0.0f);
				soup.push_back(bx);
				soup.push_back(by);
				soup.push_back(0.0f);
				soup.push_back(cx);
				soup.push_back(cy);
				soup.push_back(0.0f);
			};
			pushTri(x00, y00, x10, y10, x11, y11);
			pushTri(x00, y00, x11, y11, x01, y01);
		}
	}
	return soup;
}

double meanEdgeLengthInXRange(const std::vector<float>& soup, const double xMin, const double xMax)
{
	double sum = 0.0;
	int count = 0;
	auto edgeLen = [](float ax, float ay, float az, float bx, float by, float bz)
	{
		const double dx = bx - ax;
		const double dy = by - ay;
		const double dz = bz - az;
		return std::sqrt(dx * dx + dy * dy + dz * dz);
	};
	for (std::size_t i = 0; i + 8U < soup.size(); i += 9U)
	{
		const float cx = (soup[i] + soup[i + 3U] + soup[i + 6U]) / 3.0f;
		if (cx < xMin || cx > xMax)
		{
			continue;
		}
		sum += edgeLen(soup[i], soup[i + 1U], soup[i + 2U], soup[i + 3U], soup[i + 4U], soup[i + 5U]);
		sum += edgeLen(soup[i + 3U], soup[i + 4U], soup[i + 5U], soup[i + 6U], soup[i + 7U], soup[i + 8U]);
		sum += edgeLen(soup[i + 6U], soup[i + 7U], soup[i + 8U], soup[i], soup[i + 1U], soup[i + 2U]);
		count += 3;
	}
	return count > 0 ? (sum / static_cast<double>(count)) : 0.0;
}

} // namespace

bool runSelfTest(std::vector<std::string>& failures)
{
	failures.clear();

	{
		std::vector<float> xyz = makePlanePointCloud(20, 0.0);
		const Eigen::AlignedBox3d box = computeBoundingBox(xyz);
		expectNear(failures, "bbox.max.x", box.max().x(), 19.0, 1e-3);
		expectNear(failures, "bbox.min.z", box.min().z(), 0.0, 1e-3);
	}

	{
		std::vector<float> xyz = makePlanePointCloud(10, 5.0);
		Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
		t.translation() = Eigen::Vector3d(10.0, 20.0, 30.0);
		transformXyzInPlace(xyz, t);
		expectNear(failures, "transform.z", xyz[2], 35.0, 1e-3);
	}

	{
		std::vector<float> xyz = makePlanePointCloud(15, 0.0);
		const Eigen::AlignedBox3d box(Eigen::Vector3d(2.0, 2.0, -1.0), Eigen::Vector3d(12.0, 12.0, 1.0));
		std::vector<float> cropped;
		cropXyzByBox(xyz, box, cropped);
		expectTrue(failures, "crop.nonempty", !cropped.empty());
		expectTrue(failures, "crop.smaller", cropped.size() < xyz.size());
	}

	{
		std::vector<float> xyz = makePlanePointCloud(40, 0.0);
		const std::size_t before = pointCountFromXyz(xyz);
		expectTrue(failures, "downsample.ok", downsampleVoxelGrid(xyz, 2.0));
		expectTrue(failures, "downsample.reduced", pointCountFromXyz(xyz) < before);
	}

	{
		std::vector<float> src = makePlanePointCloud(25, 0.0);
		std::vector<float> tgt = src;
		Eigen::Isometry3d shift = Eigen::Isometry3d::Identity();
		shift.translation() = Eigen::Vector3d(5.0, -3.0, 2.0);
		transformXyzInPlace(tgt, shift);

		Eigen::Isometry3d est = Eigen::Isometry3d::Identity();
		double rmse = 0.0;
		expectTrue(failures, "icp.ok", rigidRegisterIcp(src, tgt, est, &rmse));
		expectNear(failures, "icp.tx", est.translation().x(), 5.0, 0.5);
		expectNear(failures, "icp.ty", est.translation().y(), -3.0, 0.5);
		expectNear(failures, "icp.tz", est.translation().z(), 2.0, 0.5);
	}

	{
		std::vector<float> xyz = makePlanePointCloud(8, 0.0);
		const std::vector<std::size_t> controls = {0U, 7U, 56U};
		const double disp[] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0};
		expectTrue(failures, "tps.ok", tpsDeformFromControls(xyz, controls, disp, 3U));
		expectNear(failures, "tps.ctrl0.x", xyz[0], 1.0, 0.01);
	}

	{
		std::vector<float> xyz = makePlanePointCloud(30, 0.0);
		std::vector<float> normals;
		expectTrue(failures, "normals.ok", estimateNormalsPca(xyz, normals, 8));
		expectTrue(failures, "normals.len", normals.size() == xyz.size());
	}

	{
		std::vector<float> xyz = makePlanePointCloud(25, 0.0);
		std::vector<float> soup;
		std::string err;
		expectTrue(failures, "scalespace.ok", reconstructScaleSpace(xyz, soup, 3, 0.0, &err));
		expectTrue(failures, "scalespace.soup", soup.size() % 9U == 0U && !soup.empty());
	}

	{
		std::vector<float> src = makePlanePointCloud(50, 0.0);
		std::vector<float> tgt = src;
		Eigen::Isometry3d gt = Eigen::Isometry3d::Identity();
		gt.linear() = Eigen::AngleAxisd(0.25, Eigen::Vector3d::UnitZ()).toRotationMatrix();
		gt.translation() = Eigen::Vector3d(12.0, -8.0, 5.0);
		transformXyzInPlace(tgt, gt);

		std::vector<float> srcNormals;
		std::vector<float> tgtNormals;
		expectTrue(failures, "ransac.normals.src", estimateNormalsPca(src, srcNormals, 12U));
		expectTrue(failures, "ransac.normals.tgt", estimateNormalsPca(tgt, tgtNormals, 12U));
		(void)orientNormalsMst(src, srcNormals, 12U, nullptr, nullptr);
		(void)orientNormalsMst(tgt, tgtNormals, 12U, nullptr, nullptr);

		Eigen::Isometry3d est = Eigen::Isometry3d::Identity();
		double inlierRatio = 0.0;
		RigidRegisterRansacParams ransacParams;
		ransacParams.minInliers = 30U;
		ransacParams.maxIterations = 3000;
		expectTrue(failures, "ransac.ok",
				   rigidRegisterFeatureRansac(src, srcNormals, tgt, tgtNormals, est, &inlierRatio, ransacParams));
		expectTrue(failures, "ransac.inlierRatio", inlierRatio > 0.5);
		expectNear(failures, "ransac.tx", est.translation().x(), gt.translation().x(), 1.0);
		expectNear(failures, "ransac.ty", est.translation().y(), gt.translation().y(), 1.0);
		expectNear(failures, "ransac.tz", est.translation().z(), gt.translation().z(), 1.0);
	}

#ifdef CLOUDSIM_HAS_PCL
	{
		std::vector<float> src = makePlanePointCloud(50, 0.0);
		std::vector<float> tgt = src;
		Eigen::Isometry3d gt = Eigen::Isometry3d::Identity();
		gt.linear() = Eigen::AngleAxisd(0.25, Eigen::Vector3d::UnitZ()).toRotationMatrix();
		gt.translation() = Eigen::Vector3d(12.0, -8.0, 5.0);
		transformXyzInPlace(tgt, gt);

		std::vector<float> srcNormals;
		std::vector<float> tgtNormals;
		expectTrue(failures, "ransac.pcl.normals.src", estimateNormalsPca(src, srcNormals, 12U));
		expectTrue(failures, "ransac.pcl.normals.tgt", estimateNormalsPca(tgt, tgtNormals, 12U));
		(void)orientNormalsMst(src, srcNormals, 12U, nullptr, nullptr);
		(void)orientNormalsMst(tgt, tgtNormals, 12U, nullptr, nullptr);

		Eigen::Isometry3d est = Eigen::Isometry3d::Identity();
		double inlierRatio = 0.0;
		PclGlobalAlignParams pclParams;
		pclParams.maxIterations = 20000;
		pclParams.inlierFraction = 0.2f;
		pclParams.minAcceptInlierRatio = 0.2f;
		pclParams.minAcceptReverseInlierRatio = 0.15f;
		pclParams.maxAcceptMeanNnFactor = 2.5;
		pclParams.featureVoxelMm = 1.5;
		pclParams.tryReverse = false;
		expectTrue(failures, "ransac.pcl.ok",
				   rigidRegisterFeatureRansacPcl(src, srcNormals, tgt, tgtNormals, est, &inlierRatio, pclParams));
		expectTrue(failures, "ransac.pcl.inlierRatio", inlierRatio > 0.2);
		expectNear(failures, "ransac.pcl.tx", est.translation().x(), gt.translation().x(), 2.0);
		expectNear(failures, "ransac.pcl.ty", est.translation().y(), gt.translation().y(), 2.0);
		expectNear(failures, "ransac.pcl.tz", est.translation().z(), gt.translation().z(), 2.0);
	}
#endif

	// 测试并行化工具类
	{
		const bool tbbAvailable = ParallelUtils::isTbbAvailable();
		expectTrue(failures, "parallel.tbbAvailable", tbbAvailable);

		const int threads = ParallelUtils::getThreadCount();
		expectTrue(failures, "parallel.threads", threads >= 1);

		const bool enabled = ParallelUtils::isParallelEnabled();
		expectTrue(failures, "parallel.enabled", enabled);
	}

	// 测试配置API
	{
		ReconstructionConfig config;
		config.quality = ReconstructionQuality::Fast;
		config.maxPointsForReconstruction = 100000;
		config.enableParallel = true;

		expectNear(failures, "config.voxelPrefilter", config.getVoxelPrefilterMm(), 2.0, 1e-3);
		expectNear(failures, "config.outlierRemoval", config.getOutlierRemovalPercent(), 3.0, 1e-3);
		expectTrue(failures, "config.smoothIterations", config.getSmoothIterations() == 2);
	}

	// 测试配置版本的重建API
	{
		std::vector<float> xyz = makePlanePointCloud(20, 0.0);
		std::vector<float> normals;
		expectTrue(failures, "config.normals", estimateNormalsPca(xyz, normals, 8));

		ReconstructionConfig config;
		config.quality = ReconstructionQuality::Fast;
		config.maxPointsForReconstruction = 1000; // 强制下采样

		std::vector<float> soup;
		std::string err;
		expectTrue(failures, "config.poisson", reconstructPoissonWithConfig(xyz, normals, soup, config, &err));
		expectTrue(failures, "config.poisson.soup", soup.size() % 9U == 0U && !soup.empty());
	}

	{
		std::vector<float> src = makePlanePointCloud(20, 0.0);
		std::vector<float> tgt;
		tgt.reserve(src.size());
		for (std::size_t i = 0; i < src.size(); i += 3U)
		{
			const double x = src[i];
			const double y = src[i + 1U];
			tgt.push_back(static_cast<float>(x + 0.1 * std::sin(x * 0.3)));
			tgt.push_back(static_cast<float>(y + 0.1 * std::cos(y * 0.3)));
			tgt.push_back(src[i + 2U]);
		}
		std::vector<float> srcNormals;
		std::vector<float> tgtNormals;
		expectTrue(failures, "spare.normals.src", estimateNormalsPca(src, srcNormals, 8U));
		expectTrue(failures, "spare.normals.tgt", estimateNormalsPca(tgt, tgtNormals, 8U));
		(void)orientNormalsMst(src, srcNormals, 8U, nullptr, nullptr);
		(void)orientNormalsMst(tgt, tgtNormals, 8U, nullptr, nullptr);

		std::vector<float> deformed;
		std::vector<float> deformedNormals;
		SpareRegisterParams spareParams;
		spareParams.maxOuterIters = 5;
		spareParams.useCoarseReg = true;
		spareParams.useFineReg = true;
		spareParams.normalizeScale = true;
		spareParams.rigidPreAlign = true;
		SpareRegisterResult spareResult;
		std::string spareErr;
		expectTrue(failures, "spare.ok",
				   spareRegisterPointClouds(src, srcNormals, tgt, tgtNormals, deformed, deformedNormals, spareParams,
											&spareResult, &spareErr));
		expectTrue(failures, "spare.deformed", deformed.size() == src.size());
		expectTrue(failures, "spare.finiteError", std::isfinite(spareResult.meanErrorMm));
	}

	{
		std::vector<float> src = makePlanePointCloud(16, 0.0);
		std::vector<float> tgt = makePlanePointCloud(16, 0.0);
		for (std::size_t i = 0; i < tgt.size(); i += 3U)
		{
			tgt[i] += 0.15f;
		}
		std::vector<float> deformed;
		std::vector<float> deformedNormals;
		SpareRegisterParams spareParams;
		spareParams.maxOuterIters = 5;
		spareParams.rigidPreAlign = true;
		SpareRegisterResult spareResult;
		std::string spareErr;
		expectTrue(failures, "spare.noNormals.ok",
				   spareRegisterPointClouds(src, {}, tgt, {}, deformed, deformedNormals, spareParams, &spareResult,
											&spareErr));
		expectTrue(failures, "spare.noNormals.errEmpty", spareErr.find("mismatch") == std::string::npos);
		expectTrue(failures, "spare.noNormals.deformed", !deformed.empty());
	}

	{
		std::vector<float> src = makePlanePointCloud(24, 0.0);
		std::vector<float> tgt;
		tgt.reserve(src.size());
		for (std::size_t i = 0; i < src.size(); i += 3U)
		{
			const double x = src[i];
			const double y = src[i + 1U];
			tgt.push_back(static_cast<float>(x + 0.08 * std::sin(x * 0.25)));
			tgt.push_back(static_cast<float>(y + 0.08 * std::cos(y * 0.25)));
			tgt.push_back(src[i + 2U]);
		}
		std::vector<float> srcNormals;
		std::vector<float> tgtNormals;
		expectTrue(failures, "sdf.normals.src", estimateNormalsPca(src, srcNormals, 8U));
		expectTrue(failures, "sdf.normals.tgt", estimateNormalsPca(tgt, tgtNormals, 8U));
		(void)orientNormalsMst(src, srcNormals, 8U, nullptr, nullptr);
		(void)orientNormalsMst(tgt, tgtNormals, 8U, nullptr, nullptr);

		std::vector<float> deformed;
		std::vector<float> deformedNormals;
		SdfRegisterParams sdfParams;
		sdfParams.maxOuterIters = 8;
		sdfParams.useCoarseReg = true;
		sdfParams.useFineReg = true;
		sdfParams.normalizeScale = true;
		sdfParams.fieldMode = SdfFieldMode::SignedDistance;
		sdfParams.fineDataTerm = SdfFineDataTerm::PointToPlane;
		sdfParams.rigidPreAlign = true;
		SdfRegisterResult sdfResult;
		std::string sdfErr;
		expectTrue(failures, "sdf.ok",
				   sdfRegisterPointClouds(src, srcNormals, tgt, tgtNormals, deformed, deformedNormals, sdfParams,
										  &sdfResult, &sdfErr));
		expectTrue(failures, "sdf.deformed", deformed.size() == src.size());
		expectTrue(failures, "sdf.finiteError", std::isfinite(sdfResult.meanErrorMm));
		expectTrue(failures, "sdf.fieldVoxel", sdfResult.fieldVoxelMmUsed > 0.0);
	}

	{
		const std::vector<float> wavy = makeWavyPlateSoup(24, 1.0);
		std::vector<float> adapted;
		AdaptiveRemeshParams adapt;
		adapt.characteristicEdgeMm = 1.0;
		adapt.refineIterations = 3;
		adapt.baseRemeshIterations = 2;
		std::string adaptErr;
		expectTrue(failures, "adaptiveRemesh.ok", adaptiveIsotropicRemesh(wavy, adapted, adapt, &adaptErr));
		expectTrue(failures, "adaptiveRemesh.nonempty", !adapted.empty());
		const double flatMean = meanEdgeLengthInXRange(adapted, 0.0, 10.0);
		const double wavyMean = meanEdgeLengthInXRange(adapted, 14.0, 23.0);
		expectTrue(failures, "adaptiveRemesh.flatMean", flatMean > 0.0);
		expectTrue(failures, "adaptiveRemesh.wavyMean", wavyMean > 0.0);
		expectTrue(failures, "adaptiveRemesh.density", wavyMean < flatMean * 0.95);
	}

	{
		// 平坦板 + 左侧高残差 → 左侧应更密（与曲率无关）
		const std::vector<float> flat = makeFlatPlateSoup(20, 1.0);
		AdaptiveRemeshParams adapt;
		adapt.characteristicEdgeMm = 1.0;
		adapt.approxTolMm = 0.5;
		adapt.edgeMinMm = 0.25;
		adapt.edgeMaxMm = 2.0;
		adapt.refineIterations = 4;
		adapt.baseRemeshIterations = 2;
		for (int i = 0; i <= 20; ++i)
		{
			for (int j = 0; j <= 20; ++j)
			{
				const float x = static_cast<float>(i);
				const float y = static_cast<float>(j);
				adapt.residualSampleXyz.push_back(x);
				adapt.residualSampleXyz.push_back(y);
				adapt.residualSampleXyz.push_back(0.0f);
				adapt.residualMm.push_back(x < 10.0f ? 2.0f : 0.05f);
			}
		}
		std::vector<float> adapted;
		std::string adaptErr;
		expectTrue(failures, "adaptiveRemesh.residual.ok",
				   adaptiveIsotropicRemesh(flat, adapted, adapt, &adaptErr));
		const double highResMean = meanEdgeLengthInXRange(adapted, 0.0, 8.0);
		const double lowResMean = meanEdgeLengthInXRange(adapted, 12.0, 20.0);
		expectTrue(failures, "adaptiveRemesh.residual.highMean", highResMean > 0.0);
		expectTrue(failures, "adaptiveRemesh.residual.lowMean", lowResMean > 0.0);
		expectTrue(failures, "adaptiveRemesh.residual.density", highResMean < lowResMean * 0.95);
	}

	return failures.empty();
}

} // namespace pclalgo
