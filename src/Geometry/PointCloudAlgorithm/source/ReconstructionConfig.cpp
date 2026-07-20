/// @file ReconstructionConfig.cpp
/// @brief ReconstructionConfig 实现

#include "ReconstructionConfig.h"

#include "Downsample.h"
#include "Measure.h"
#include "PointCloudBuffer.h"
#include "Preprocess.h"
#include "Reconstruction.h"

namespace pclalgo
{
double ReconstructionConfig::getVoxelPrefilterMm() const
{
	switch (quality)
	{
	case ReconstructionQuality::Fast:
		return 2.0;
	case ReconstructionQuality::Balanced:
		return 1.0;
	case ReconstructionQuality::Quality:
		return 0.5;
	default:
		return 1.0;
	}
}

double ReconstructionConfig::getSpacingMm() const
{
	// 返回0.0表示自动计算
	return 0.0;
}

int ReconstructionConfig::getSmoothIterations() const
{
	switch (quality)
	{
	case ReconstructionQuality::Fast:
		return 2;
	case ReconstructionQuality::Balanced:
		return 4;
	case ReconstructionQuality::Quality:
		return 6;
	default:
		return 4;
	}
}

double ReconstructionConfig::getOutlierRemovalPercent() const
{
	switch (quality)
	{
	case ReconstructionQuality::Fast:
		return 3.0;
	case ReconstructionQuality::Balanced:
		return 5.0;
	case ReconstructionQuality::Quality:
		return 7.0;
	default:
		return 5.0;
	}
}

bool reconstructPoissonWithConfig(std::vector<float> xyz, std::vector<float> normals,
								  std::vector<float>& triangleSoupOut, const ReconstructionConfig& config,
								  std::string* errMsg)
{
	triangleSoupOut.clear();

	// 自动下采样
	const std::size_t pointCount = pointCountFromXyz(xyz);
	if (pointCount > config.maxPointsForReconstruction)
	{
		const double ratio = static_cast<double>(config.maxPointsForReconstruction) / pointCount;
		const double voxelSize = computeAverageSpacingMm(xyz, 6) / std::cbrt(ratio);
		if (voxelSize > 0.0)
		{
			if (!downsampleVoxelGrid(xyz, voxelSize))
			{
				if (errMsg != nullptr)
				{
					*errMsg = "Auto downsampling failed";
				}
				return false;
			}
			// 重新计算法线
			normals.clear();
			if (!estimateNormalsPca(xyz, normals, 12, errMsg))
			{
				return false;
			}
		}
	}

	return reconstructPoisson(xyz, normals, triangleSoupOut, 0.0, 20.0, 30.0, 0.375, errMsg);
}

bool reconstructPoissonAutoWithConfig(std::vector<float> xyz, std::vector<float>& triangleSoupOut,
									  const ReconstructionConfig& config, std::string* errMsg)
{
	triangleSoupOut.clear();

	// 自动下采样
	const std::size_t pointCount = pointCountFromXyz(xyz);
	if (pointCount > config.maxPointsForReconstruction)
	{
		const double ratio = static_cast<double>(config.maxPointsForReconstruction) / pointCount;
		const double voxelSize = computeAverageSpacingMm(xyz, 6) / std::cbrt(ratio);
		if (voxelSize > 0.0)
		{
			if (!downsampleVoxelGrid(xyz, voxelSize))
			{
				if (errMsg != nullptr)
				{
					*errMsg = "Auto downsampling failed";
				}
				return false;
			}
		}
	}

	return reconstructPoissonAuto(xyz, triangleSoupOut, config.getVoxelPrefilterMm(), config.getOutlierRemovalPercent(),
								  errMsg);
}

bool reconstructScaleSpaceWithConfig(std::vector<float> xyz, std::vector<float>& triangleSoupOut,
									 const ReconstructionConfig& config, std::string* errMsg)
{
	triangleSoupOut.clear();

	// 自动下采样
	const std::size_t pointCount = pointCountFromXyz(xyz);
	if (pointCount > config.maxPointsForReconstruction)
	{
		const double ratio = static_cast<double>(config.maxPointsForReconstruction) / pointCount;
		const double voxelSize = computeAverageSpacingMm(xyz, 6) / std::cbrt(ratio);
		if (voxelSize > 0.0)
		{
			if (!downsampleVoxelGrid(xyz, voxelSize))
			{
				if (errMsg != nullptr)
				{
					*errMsg = "Auto downsampling failed";
				}
				return false;
			}
		}
	}

	return reconstructScaleSpace(xyz, triangleSoupOut, config.getSmoothIterations(), 0.0, errMsg);
}

} // namespace pclalgo