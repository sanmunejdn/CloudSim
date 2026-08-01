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
#include "RegistrationNonRigid.h"
#include "RegistrationRigid.h"
#include "RegistrationSpare.h"
#include "RegistrationSdf.h"
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

	return failures.empty();
}

} // namespace pclalgo
