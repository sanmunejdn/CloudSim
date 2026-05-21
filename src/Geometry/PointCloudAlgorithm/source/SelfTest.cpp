#include "SelfTest.h"

#include "PointCloudBuffer.h"
#include "Crop.h"
#include "Downsample.h"
#include "Measure.h"
#include "Preprocess.h"
#include "Reconstruction.h"
#include "RegistrationNonRigid.h"
#include "RegistrationRigid.h"
#include "Transform.h"

#include <cmath>
#include <sstream>

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

void expectNear(std::vector<std::string>& failures, const char* name, const double actual, const double expected, const double eps)
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

	return failures.empty();
}

} // namespace pclalgo
