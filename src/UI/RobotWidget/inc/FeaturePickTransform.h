#ifndef ROBOTWIDGET_FEATUREPICKTRANSFORM_H
#define ROBOTWIDGET_FEATUREPICKTRANSFORM_H

/// @file FeaturePickTransform.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 追加一条 raw 到已有 overlay（多 PathPlan 叠加预览）

#include "robotwidget_global.h"

#include "IRobotOsgViewHost.h"
#include "RobotOsgUiTypes.h"
#include "RobotSimulationMath.h"

#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include <RawTrajectory.h>
#include <Types.h>
#include <osg/Matrixd>
#include <osg/Vec3f>

namespace feature_pick_transform
{
inline bool backendWorldRotationMatrix(IRobotOsgViewHost* osg, const std::string& backendId, osg::Matrixd& outRot,
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
	if (!RobotSimulationMath::getBackendRootWorldMatrixOsg(osg, backendId, worldMat))
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

ROBOTWIDGET_EXPORT bool worldPointToStepModelMm(IRobotOsgViewHost* osg, const std::string& backendId,
												const osg::Vec3f& worldMm, geoalgo::Point3d& outModel,
												std::string* errMsg = nullptr);

ROBOTWIDGET_EXPORT bool stepModelPointToWorldMm(IRobotOsgViewHost* osg, const std::string& backendId,
												const geoalgo::Point3d& modelMm, osg::Vec3f& outWorld,
												std::string* errMsg = nullptr);

inline bool stepModelDirectionToWorld(IRobotOsgViewHost* osg, const std::string& backendId,
									  const geoalgo::Point3d& modelDir, osg::Vec3f& outWorldDir,
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
	outWorldDir.set(static_cast<float>(dw.x() / len), static_cast<float>(dw.y() / len),
					static_cast<float>(dw.z() / len));
	return true;
}

ROBOTWIDGET_EXPORT bool transformTrajectoryPointToWorld(IRobotOsgViewHost* osg, const std::string& backendId,
														const RobotInstruction::TrajectoryPoint& filePoint,
														RobotInstruction::TrajectoryPoint& outWorld,
														std::string* errMsg = nullptr);

ROBOTWIDGET_EXPORT bool transformRawTrajectoryToWorld(IRobotOsgViewHost* osg, const std::string& backendId,
													  const RobotInstruction::RawTrajectory& fileTraj,
													  RobotInstruction::RawTrajectory& outWorld,
													  std::string* errMsg = nullptr);

ROBOTWIDGET_EXPORT bool transformTrajectoryPointToFile(IRobotOsgViewHost* osg, const std::string& backendId,
													   const RobotInstruction::TrajectoryPoint& worldPoint,
													   RobotInstruction::TrajectoryPoint& outFile,
													   std::string* errMsg = nullptr);

ROBOTWIDGET_EXPORT bool transformRawTrajectoryWorldToFile(IRobotOsgViewHost* osg, const std::string& backendId,
														  const RobotInstruction::RawTrajectory& worldTraj,
														  RobotInstruction::RawTrajectory& outFile,
														  std::string* errMsg = nullptr);

ROBOTWIDGET_EXPORT bool buildRawTrajectoryOverlayWorld(IRobotOsgViewHost* osg, const std::string& backendId,
													   const RobotInstruction::RawTrajectory& fileTraj,
													   std::vector<RobotOsgUi::RawTrajectoryOverlayVertex>& outOverlay,
													   std::string* errMsg = nullptr);

ROBOTWIDGET_EXPORT bool buildRawTrajectoryPreviewWorld(IRobotOsgViewHost* osg, const std::string& backendId,
													   const RobotInstruction::RawTrajectory& fileTraj,
													   const RobotOsgUi::RawTrajectoryPreviewOptions& options,
													   std::vector<RobotOsgUi::RawTrajectoryOverlayVertex>& outOverlay,
													   std::vector<RobotOsgUi::RawTrajectoryOverlayFrame>& outFrames,
													   std::string* errMsg = nullptr);

/// 追加一条 raw 到已有 overlay（多 PathPlan 叠加预览）
ROBOTWIDGET_EXPORT bool
appendRawTrajectoryOverlayWorld(IRobotOsgViewHost* osg, const std::string& backendId,
								const RobotInstruction::RawTrajectory& fileTraj,
								std::vector<RobotOsgUi::RawTrajectoryOverlayVertex>& inOutOverlay,
								std::vector<std::size_t>& inOutSegmentEndExclusive, std::string* errMsg = nullptr);

ROBOTWIDGET_EXPORT void finalizeOverlaySegmentEnds(std::size_t totalPoints,
												   std::vector<std::size_t>& segmentEndExclusive);

ROBOTWIDGET_EXPORT void applyRawTrajectoryPreviewToOsg(IRobotOsgViewHost* osg, const std::string& backendId,
													   const RobotInstruction::RawTrajectory& fileTraj,
													   const RobotOsgUi::RawTrajectoryPreviewOptions& options,
													   std::string* errMsg = nullptr);

/// 合并多条 raw 叠加层；axesSources 中每条可见 PathPlan 均绘制坐标轴
ROBOTWIDGET_EXPORT void applyMergedRawTrajectoryPreviewToOsg(
	IRobotOsgViewHost* osg, const std::vector<RobotOsgUi::RawTrajectoryOverlayVertex>& overlay,
	const std::vector<std::size_t>& segmentEndExclusive,
	const std::vector<std::pair<std::string, const RobotInstruction::RawTrajectory*>>& axesSources,
	const RobotOsgUi::RawTrajectoryPreviewOptions& options, std::string* errMsg = nullptr);

/// 单条 raw 预览（内部转调 merge 接口）
ROBOTWIDGET_EXPORT void applyMergedRawTrajectoryPreviewToOsg(
	IRobotOsgViewHost* osg, const std::vector<RobotOsgUi::RawTrajectoryOverlayVertex>& overlay,
	const std::vector<std::size_t>& segmentEndExclusive, const std::string& axesBackendId,
	const RobotInstruction::RawTrajectory* axesTraj, const RobotOsgUi::RawTrajectoryPreviewOptions& options,
	std::string* errMsg = nullptr);

/// poseMm/eulerDeg 已是世界坐标（Unified Apply/预览链输出），不再做 file→world
ROBOTWIDGET_EXPORT void applyWorldRawTrajectoryPreviewToOsg(IRobotOsgViewHost* osg,
															const RobotInstruction::RawTrajectory& worldTraj,
															const RobotOsgUi::RawTrajectoryPreviewOptions& options,
															std::string* errMsg = nullptr);

/// mesh 模型系 raw 预览（与 STEP 文件系共用 backend worldMatrix 变换）
inline void applyMeshLocalRawTrajectoryPreviewToOsg(IRobotOsgViewHost* osg, const std::string& backendId,
													const RobotInstruction::RawTrajectory& meshLocalTraj,
													const RobotOsgUi::RawTrajectoryPreviewOptions& options,
													std::string* errMsg = nullptr)
{
	applyRawTrajectoryPreviewToOsg(osg, backendId, meshLocalTraj, options, errMsg);
}

inline bool transformMeshLocalRawTrajectoryToWorld(IRobotOsgViewHost* osg, const std::string& backendId,
												   const RobotInstruction::RawTrajectory& meshLocalTraj,
												   RobotInstruction::RawTrajectory& outWorld,
												   std::string* errMsg = nullptr)
{
	return transformRawTrajectoryToWorld(osg, backendId, meshLocalTraj, outWorld, errMsg);
}

} // namespace feature_pick_transform

#endif // ROBOTWIDGET_FEATUREPICKTRANSFORM_H
