#pragma once

#include "robotwidget_global.h"
#include "IRobotOsgViewHost.h"
#include "RobotOsgUiTypes.h"

#include <RawTrajectory.h>

#include <Types.h>

#include <cmath>

#include <osg/Matrixd>
#include <osg/Vec3f>

#include <string>

namespace feature_pick_transform
{

inline bool backendWorldRotationMatrix(
	IRobotOsgViewHost* osg,
	const std::string& backendId,
	osg::Matrixd& outRot,
	std::string* errMsg = nullptr)
{
	if (!osg)
	{
		if (errMsg)
		{
			*errMsg = "no osg host";
		}
		return false;
	}
	osg::Matrixd worldMat;
	if (!osg->getBackendRootWorldMatrix(backendId, worldMat))
	{
		if (errMsg)
		{
			*errMsg = "backend world matrix unavailable";
		}
		return false;
	}
	outRot = worldMat;
	outRot.setTrans(0.0, 0.0, 0.0);
	return true;
}

inline bool worldPointToStepModelMm(
	IRobotOsgViewHost* osg,
	const std::string& backendId,
	const osg::Vec3f& worldMm,
	geoalgo::Point3d& outModel,
	std::string* errMsg = nullptr)
{
	if (!osg)
	{
		if (errMsg)
		{
			*errMsg = "no osg host";
		}
		return false;
	}
	osg::Matrixd worldMat;
	if (!osg->getBackendRootWorldMatrix(backendId, worldMat))
	{
		if (errMsg)
		{
			*errMsg = "backend world matrix unavailable";
		}
		return false;
	}
	osg::Matrixd invMat;
	if (!invMat.invert(worldMat))
	{
		if (errMsg)
		{
			*errMsg = "failed to invert backend matrix";
		}
		return false;
	}
	const osg::Vec3d pw(static_cast<double>(worldMm.x()), static_cast<double>(worldMm.y()),
		static_cast<double>(worldMm.z()));
	const osg::Vec3d pOuter = pw * invMat;
	double cx = 0.0;
	double cy = 0.0;
	double cz = 0.0;
	(void)osg->tryGetBackendModelCenterMm(backendId, cx, cy, cz);
	outModel.x = pOuter.x() + cx;
	outModel.y = pOuter.y() + cy;
	outModel.z = pOuter.z() + cz;
	return true;
}

inline bool stepModelPointToWorldMm(
	IRobotOsgViewHost* osg,
	const std::string& backendId,
	const geoalgo::Point3d& modelMm,
	osg::Vec3f& outWorld,
	std::string* errMsg = nullptr)
{
	if (!osg)
	{
		if (errMsg)
		{
			*errMsg = "no osg host";
		}
		return false;
	}
	osg::Matrixd worldMat;
	if (!osg->getBackendRootWorldMatrix(backendId, worldMat))
	{
		if (errMsg)
		{
			*errMsg = "backend world matrix unavailable";
		}
		return false;
	}
	double cx = 0.0;
	double cy = 0.0;
	double cz = 0.0;
	(void)osg->tryGetBackendModelCenterMm(backendId, cx, cy, cz);
	const osg::Vec3d pFile(
		modelMm.x - cx,
		modelMm.y - cy,
		modelMm.z - cz);
	const osg::Vec3d pw = pFile * worldMat;
	outWorld.set(
		static_cast<float>(pw.x()),
		static_cast<float>(pw.y()),
		static_cast<float>(pw.z()));
	return true;
}

inline bool stepModelDirectionToWorld(
	IRobotOsgViewHost* osg,
	const std::string& backendId,
	const geoalgo::Point3d& modelDir,
	osg::Vec3f& outWorldDir,
	std::string* errMsg = nullptr)
{
	osg::Matrixd rot;
	if (!backendWorldRotationMatrix(osg, backendId, rot, errMsg))
	{
		return false;
	}
	const osg::Vec3d d(modelDir.x, modelDir.y, modelDir.z);
	osg::Vec3d dw = d * rot;
	const double len = std::sqrt(dw.x() * dw.x() + dw.y() * dw.y() + dw.z() * dw.z());
	if (len < 1e-12)
	{
		outWorldDir.set(0.0f, 0.0f, 1.0f);
		return true;
	}
	outWorldDir.set(
		static_cast<float>(dw.x() / len),
		static_cast<float>(dw.y() / len),
		static_cast<float>(dw.z() / len));
	return true;
}

ROBOTWIDGET_EXPORT bool transformTrajectoryPointToWorld(
	IRobotOsgViewHost* osg,
	const std::string& backendId,
	const RobotInstruction::TrajectoryPoint& filePoint,
	RobotInstruction::TrajectoryPoint& outWorld,
	std::string* errMsg = nullptr);

ROBOTWIDGET_EXPORT bool transformRawTrajectoryToWorld(
	IRobotOsgViewHost* osg,
	const std::string& backendId,
	const RobotInstruction::RawTrajectory& fileTraj,
	RobotInstruction::RawTrajectory& outWorld,
	std::string* errMsg = nullptr);

ROBOTWIDGET_EXPORT bool transformTrajectoryPointToFile(
	IRobotOsgViewHost* osg,
	const std::string& backendId,
	const RobotInstruction::TrajectoryPoint& worldPoint,
	RobotInstruction::TrajectoryPoint& outFile,
	std::string* errMsg = nullptr);

ROBOTWIDGET_EXPORT bool transformRawTrajectoryWorldToFile(
	IRobotOsgViewHost* osg,
	const std::string& backendId,
	const RobotInstruction::RawTrajectory& worldTraj,
	RobotInstruction::RawTrajectory& outFile,
	std::string* errMsg = nullptr);

ROBOTWIDGET_EXPORT bool buildRawTrajectoryOverlayWorld(
	IRobotOsgViewHost* osg,
	const std::string& backendId,
	const RobotInstruction::RawTrajectory& fileTraj,
	std::vector<RobotOsgUi::RawTrajectoryOverlayVertex>& outOverlay,
	std::string* errMsg = nullptr);

ROBOTWIDGET_EXPORT bool buildRawTrajectoryPreviewWorld(
	IRobotOsgViewHost* osg,
	const std::string& backendId,
	const RobotInstruction::RawTrajectory& fileTraj,
	const RobotOsgUi::RawTrajectoryPreviewOptions& options,
	std::vector<RobotOsgUi::RawTrajectoryOverlayVertex>& outOverlay,
	std::vector<RobotOsgUi::RawTrajectoryOverlayFrame>& outFrames,
	std::string* errMsg = nullptr);

ROBOTWIDGET_EXPORT void applyRawTrajectoryPreviewToOsg(
	IRobotOsgViewHost* osg,
	const std::string& backendId,
	const RobotInstruction::RawTrajectory& fileTraj,
	const RobotOsgUi::RawTrajectoryPreviewOptions& options,
	std::string* errMsg = nullptr);

/// poseMm/eulerDeg 已是世界坐标（Unified Apply/预览链输出），不再做 file→world
ROBOTWIDGET_EXPORT void applyWorldRawTrajectoryPreviewToOsg(
	IRobotOsgViewHost* osg,
	const RobotInstruction::RawTrajectory& worldTraj,
	const RobotOsgUi::RawTrajectoryPreviewOptions& options,
	std::string* errMsg = nullptr);

} // namespace feature_pick_transform
