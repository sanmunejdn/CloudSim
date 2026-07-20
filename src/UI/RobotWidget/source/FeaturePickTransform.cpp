/// @file FeaturePickTransform.cpp
/// @brief FeaturePickTransform 实现

#include "FeaturePickTransform.h"

#include "../../OsgWidgetCore/inc/OsgScene.h"
#include "RobotOsgUiTypes.h"

#include <algorithm>
#include <sstream>
#include <vector>

namespace feature_pick_transform
{
namespace
{
std::string transformBackendId(IRobotOsgViewHost* osg, const std::string& backendId)
{
	if (!osg)
	{
		return backendId;
	}
	return osg->resolvePickScopeBackendId(backendId);
}

void appendUniqueIndex(std::vector<std::size_t>& indices, const std::size_t index)
{
	if (indices.empty() || indices.back() != index)
	{
		indices.push_back(index);
	}
}

std::vector<std::size_t> collectPreviewAxisPointIndices(const std::size_t pointCount, const int axisInterval,
														const std::vector<std::size_t>& segmentEndExclusive)
{
	std::vector<std::size_t> indices;
	if (pointCount == 0U)
	{
		return indices;
	}
	if (pointCount == 1U)
	{
		indices.push_back(0U);
		return indices;
	}

	const int autoStride = std::max(1, static_cast<int>(pointCount / 20U));
	const int stride = axisInterval > 0 ? axisInterval : autoStride;

	indices.reserve(pointCount / static_cast<std::size_t>(stride) + segmentEndExclusive.size() + 2U);
	appendUniqueIndex(indices, 0U);
	for (const std::size_t segmentStart : segmentEndExclusive)
	{
		if (segmentStart < pointCount)
		{
			appendUniqueIndex(indices, segmentStart);
		}
	}
	for (std::size_t i = static_cast<std::size_t>(stride); i < pointCount; i += static_cast<std::size_t>(stride))
	{
		appendUniqueIndex(indices, i);
	}
	appendUniqueIndex(indices, pointCount - 1U);
	return indices;
}

bool appendPreviewFramesForIndices(const std::vector<std::size_t>& indices,
								   const std::vector<RobotInstruction::TrajectoryPoint>& points, IRobotOsgViewHost* osg,
								   const std::string& backendId,
								   std::vector<RobotOsgUi::RawTrajectoryOverlayFrame>& outFrames, std::string* errMsg)
{
	outFrames.reserve(indices.size());
	for (const std::size_t index : indices)
	{
		if (index >= points.size())
		{
			continue;
		}
		RobotInstruction::TrajectoryPoint worldTp;
		if (!transformTrajectoryPointToWorld(osg, backendId, points[index], worldTp, errMsg))
		{
			return false;
		}
		RobotOsgUi::RawTrajectoryOverlayFrame frame;
		frame.positionMm.set(static_cast<float>(worldTp.poseMm.x), static_cast<float>(worldTp.poseMm.y),
							 static_cast<float>(worldTp.poseMm.z));
		frame.eulerDeg.set(static_cast<float>(worldTp.eulerDeg.x), static_cast<float>(worldTp.eulerDeg.y),
						   static_cast<float>(worldTp.eulerDeg.z));
		frame.reachable = worldTp.reachable;
		outFrames.push_back(frame);
	}
	return true;
}

void appendPreviewFramesForIndicesWorld(const std::vector<std::size_t>& indices,
										const std::vector<RobotInstruction::TrajectoryPoint>& points,
										std::vector<RobotOsgUi::RawTrajectoryOverlayFrame>& outFrames)
{
	outFrames.reserve(indices.size());
	for (const std::size_t index : indices)
	{
		if (index >= points.size())
		{
			continue;
		}
		const RobotInstruction::TrajectoryPoint& tp = points[index];
		RobotOsgUi::RawTrajectoryOverlayFrame frame;
		frame.positionMm.set(static_cast<float>(tp.poseMm.x), static_cast<float>(tp.poseMm.y),
							 static_cast<float>(tp.poseMm.z));
		frame.eulerDeg.set(static_cast<float>(tp.eulerDeg.x), static_cast<float>(tp.eulerDeg.y),
						   static_cast<float>(tp.eulerDeg.z));
		frame.reachable = tp.reachable;
		outFrames.push_back(frame);
	}
}

} // namespace

bool worldPointToStepModelMm(IRobotOsgViewHost* osg, const std::string& backendId, const osg::Vec3f& worldMm,
							 geoalgo::Point3d& outModel, std::string* errMsg)
{
	if (!osg)
	{
		if (errMsg)
		{
			*errMsg = "no osg host";
		}
		return false;
	}
	const std::string xformId = transformBackendId(osg, backendId);
	osg::Matrixd worldMat;
	if (!osg->getBackendRootWorldMatrix(xformId, worldMat))
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
	outModel.x = pOuter.x();
	outModel.y = pOuter.y();
	outModel.z = pOuter.z();
	return true;
}

bool stepModelPointToWorldMm(IRobotOsgViewHost* osg, const std::string& backendId, const geoalgo::Point3d& modelMm,
							 osg::Vec3f& outWorld, std::string* errMsg)
{
	if (!osg)
	{
		if (errMsg)
		{
			*errMsg = "no osg host";
		}
		return false;
	}
	const std::string xformId = transformBackendId(osg, backendId);
	osg::Matrixd worldMat;
	if (!osg->getBackendRootWorldMatrix(xformId, worldMat))
	{
		if (errMsg)
		{
			*errMsg = "backend world matrix unavailable";
		}
		return false;
	}
	const osg::Vec3d pFile(modelMm.x, modelMm.y, modelMm.z);
	const osg::Vec3d pw = pFile * worldMat;
	outWorld.set(static_cast<float>(pw.x()), static_cast<float>(pw.y()), static_cast<float>(pw.z()));
	return true;
}

bool transformTrajectoryPointToWorld(IRobotOsgViewHost* osg, const std::string& backendId,
									 const RobotInstruction::TrajectoryPoint& filePoint,
									 RobotInstruction::TrajectoryPoint& outWorld, std::string* errMsg)
{
	geoalgo::Point3d filePos{filePoint.poseMm.x, filePoint.poseMm.y, filePoint.poseMm.z};
	osg::Vec3f worldPos;
	if (!stepModelPointToWorldMm(osg, backendId, filePos, worldPos, errMsg))
	{
		return false;
	}
	outWorld = filePoint;
	outWorld.poseMm.x = static_cast<double>(worldPos.x());
	outWorld.poseMm.y = static_cast<double>(worldPos.y());
	outWorld.poseMm.z = static_cast<double>(worldPos.z());

	osg::Matrixd rot;
	if (!backendWorldRotationMatrix(osg, backendId, rot, errMsg))
	{
		return false;
	}
	const osg::Quat qFile = OsgScene::eulerDegToQuat(osg::Vec3f(static_cast<float>(filePoint.eulerDeg.x),
																static_cast<float>(filePoint.eulerDeg.y),
																static_cast<float>(filePoint.eulerDeg.z)));
	osg::Matrixd mFile = osg::Matrixd::rotate(qFile);
	osg::Matrixd mWorld = mFile * rot;
	const osg::Vec3f eulerWorld = OsgScene::quatToEulerDeg(mWorld.getRotate());
	outWorld.eulerDeg.x = static_cast<double>(eulerWorld.x());
	outWorld.eulerDeg.y = static_cast<double>(eulerWorld.y());
	outWorld.eulerDeg.z = static_cast<double>(eulerWorld.z());
	return true;
}

bool transformRawTrajectoryToWorld(IRobotOsgViewHost* osg, const std::string& backendId,
								   const RobotInstruction::RawTrajectory& fileTraj,
								   RobotInstruction::RawTrajectory& outWorld, std::string* errMsg)
{
	outWorld = fileTraj;
	outWorld.points.clear();
	outWorld.points.reserve(fileTraj.points.size());
	for (const RobotInstruction::TrajectoryPoint& tp : fileTraj.points)
	{
		RobotInstruction::TrajectoryPoint worldTp;
		if (!transformTrajectoryPointToWorld(osg, backendId, tp, worldTp, errMsg))
		{
			return false;
		}
		outWorld.points.push_back(worldTp);
	}
	return true;
}

bool transformTrajectoryPointToFile(IRobotOsgViewHost* osg, const std::string& backendId,
									const RobotInstruction::TrajectoryPoint& worldPoint,
									RobotInstruction::TrajectoryPoint& outFile, std::string* errMsg)
{
	geoalgo::Point3d modelPos{};
	const osg::Vec3f worldMm(static_cast<float>(worldPoint.poseMm.x), static_cast<float>(worldPoint.poseMm.y),
							 static_cast<float>(worldPoint.poseMm.z));
	if (!worldPointToStepModelMm(osg, backendId, worldMm, modelPos, errMsg))
	{
		return false;
	}
	outFile = worldPoint;
	outFile.poseMm.x = modelPos.x;
	outFile.poseMm.y = modelPos.y;
	outFile.poseMm.z = modelPos.z;

	osg::Matrixd rot;
	if (!backendWorldRotationMatrix(osg, backendId, rot, errMsg))
	{
		return false;
	}
	osg::Matrixd rotInv;
	if (!rotInv.invert(rot))
	{
		if (errMsg)
		{
			*errMsg = "failed to invert backend rotation";
		}
		return false;
	}
	const osg::Quat qWorld = OsgScene::eulerDegToQuat(osg::Vec3f(static_cast<float>(worldPoint.eulerDeg.x),
																 static_cast<float>(worldPoint.eulerDeg.y),
																 static_cast<float>(worldPoint.eulerDeg.z)));
	const osg::Matrixd mWorld = osg::Matrixd::rotate(qWorld);
	const osg::Matrixd mFile = mWorld * rotInv;
	const osg::Vec3f eulerFile = OsgScene::quatToEulerDeg(mFile.getRotate());
	outFile.eulerDeg.x = static_cast<double>(eulerFile.x());
	outFile.eulerDeg.y = static_cast<double>(eulerFile.y());
	outFile.eulerDeg.z = static_cast<double>(eulerFile.z());
	return true;
}

bool transformRawTrajectoryWorldToFile(IRobotOsgViewHost* osg, const std::string& backendId,
									   const RobotInstruction::RawTrajectory& worldTraj,
									   RobotInstruction::RawTrajectory& outFile, std::string* errMsg)
{
	outFile = worldTraj;
	outFile.points.clear();
	outFile.points.reserve(worldTraj.points.size());
	for (const RobotInstruction::TrajectoryPoint& tp : worldTraj.points)
	{
		RobotInstruction::TrajectoryPoint fileTp;
		if (!transformTrajectoryPointToFile(osg, backendId, tp, fileTp, errMsg))
		{
			return false;
		}
		outFile.points.push_back(fileTp);
	}
	return true;
}

bool buildRawTrajectoryOverlayWorld(IRobotOsgViewHost* osg, const std::string& backendId,
									const RobotInstruction::RawTrajectory& fileTraj,
									std::vector<RobotOsgUi::RawTrajectoryOverlayVertex>& outOverlay,
									std::string* errMsg)
{
	outOverlay.clear();
	std::vector<std::size_t> emptySegments;
	return appendRawTrajectoryOverlayWorld(osg, backendId, fileTraj, outOverlay, emptySegments, errMsg);
}

bool appendRawTrajectoryOverlayWorld(IRobotOsgViewHost* osg, const std::string& backendId,
									 const RobotInstruction::RawTrajectory& fileTraj,
									 std::vector<RobotOsgUi::RawTrajectoryOverlayVertex>& inOutOverlay,
									 std::vector<std::size_t>& inOutSegmentEndExclusive, std::string* errMsg)
{
	if (fileTraj.points.empty())
	{
		return true;
	}
	const std::size_t baseCount = inOutOverlay.size();
	if (baseCount > 0U)
	{
		inOutSegmentEndExclusive.push_back(baseCount);
	}
	inOutOverlay.reserve(baseCount + fileTraj.points.size());
	for (const RobotInstruction::TrajectoryPoint& tp : fileTraj.points)
	{
		const geoalgo::Point3d filePos{tp.poseMm.x, tp.poseMm.y, tp.poseMm.z};
		osg::Vec3f worldPos;
		if (!stepModelPointToWorldMm(osg, backendId, filePos, worldPos, errMsg))
		{
			return false;
		}
		RobotOsgUi::RawTrajectoryOverlayVertex v;
		v.positionMm = worldPos;
		v.reachable = tp.reachable;
		inOutOverlay.push_back(v);
	}
	for (const std::size_t endExclusive : fileTraj.segmentEndExclusive)
	{
		inOutSegmentEndExclusive.push_back(baseCount + endExclusive);
	}
	return true;
}

bool buildRawTrajectoryPreviewWorld(IRobotOsgViewHost* osg, const std::string& backendId,
									const RobotInstruction::RawTrajectory& fileTraj,
									const RobotOsgUi::RawTrajectoryPreviewOptions& options,
									std::vector<RobotOsgUi::RawTrajectoryOverlayVertex>& outOverlay,
									std::vector<RobotOsgUi::RawTrajectoryOverlayFrame>& outFrames, std::string* errMsg)
{
	outFrames.clear();
	if (!buildRawTrajectoryOverlayWorld(osg, backendId, fileTraj, outOverlay, errMsg))
	{
		return false;
	}
	if (!options.showAxes || fileTraj.points.empty())
	{
		return true;
	}
	const std::vector<std::size_t> axisIndices =
		collectPreviewAxisPointIndices(fileTraj.points.size(), options.axisInterval, fileTraj.segmentEndExclusive);
	return appendPreviewFramesForIndices(axisIndices, fileTraj.points, osg, backendId, outFrames, errMsg);
}

namespace
{
void finalizeOverlaySegmentEndsImpl(const std::size_t totalPoints, std::vector<std::size_t>& segmentEndExclusive)
{
	if (totalPoints < 2U)
	{
		segmentEndExclusive.clear();
		return;
	}
	if (segmentEndExclusive.empty() || segmentEndExclusive.back() != totalPoints)
	{
		segmentEndExclusive.push_back(totalPoints);
	}
}
} // namespace

void finalizeOverlaySegmentEnds(const std::size_t totalPoints, std::vector<std::size_t>& segmentEndExclusive)
{
	finalizeOverlaySegmentEndsImpl(totalPoints, segmentEndExclusive);
}

void applyMergedRawTrajectoryPreviewToOsg(
	IRobotOsgViewHost* osg, const std::vector<RobotOsgUi::RawTrajectoryOverlayVertex>& overlay,
	const std::vector<std::size_t>& segmentEndExclusive,
	const std::vector<std::pair<std::string, const RobotInstruction::RawTrajectory*>>& axesSources,
	const RobotOsgUi::RawTrajectoryPreviewOptions& options, std::string* errMsg)
{
	if (!osg)
	{
		if (errMsg)
		{
			*errMsg = "no osg host";
		}
		return;
	}
	if (overlay.empty())
	{
		osg->clearRawTrajectoryOverlay();
		osg->clearRawTrajectoryOverlayFrames();
		osg->requestRedraw();
		return;
	}
	std::vector<RobotOsgUi::RawTrajectoryOverlayFrame> frames;
	if (options.showAxes)
	{
		for (const auto& source : axesSources)
		{
			if (!source.second || source.second->points.empty() || source.first.empty())
			{
				continue;
			}
			std::vector<RobotOsgUi::RawTrajectoryOverlayVertex> unusedOverlay;
			std::vector<RobotOsgUi::RawTrajectoryOverlayFrame> batchFrames;
			if (!buildRawTrajectoryPreviewWorld(osg, source.first, *source.second, options, unusedOverlay, batchFrames,
												errMsg))
			{
				continue;
			}
			frames.insert(frames.end(), batchFrames.begin(), batchFrames.end());
		}
	}
	osg->clearInstructionPoseAxes();
	osg->setRawTrajectoryOverlay(overlay, segmentEndExclusive);
	if (options.showAxes && !frames.empty())
	{
		osg->setRawTrajectoryOverlayAxisComponents(options.showAxisX, options.showAxisY, options.showAxisZ);
		osg->setRawTrajectoryOverlayFrames(frames);
	}
	else
	{
		osg->clearRawTrajectoryOverlayFrames();
	}
	osg->requestRedraw();
}

void applyMergedRawTrajectoryPreviewToOsg(IRobotOsgViewHost* osg,
										  const std::vector<RobotOsgUi::RawTrajectoryOverlayVertex>& overlay,
										  const std::vector<std::size_t>& segmentEndExclusive,
										  const std::string& axesBackendId,
										  const RobotInstruction::RawTrajectory* axesTraj,
										  const RobotOsgUi::RawTrajectoryPreviewOptions& options, std::string* errMsg)
{
	std::vector<std::pair<std::string, const RobotInstruction::RawTrajectory*>> axesSources;
	if (axesTraj && !axesTraj->points.empty() && !axesBackendId.empty())
	{
		axesSources.emplace_back(axesBackendId, axesTraj);
	}
	applyMergedRawTrajectoryPreviewToOsg(osg, overlay, segmentEndExclusive, axesSources, options, errMsg);
}

void applyRawTrajectoryPreviewToOsg(IRobotOsgViewHost* osg, const std::string& backendId,
									const RobotInstruction::RawTrajectory& fileTraj,
									const RobotOsgUi::RawTrajectoryPreviewOptions& options, std::string* errMsg)
{
	if (!osg)
	{
		if (errMsg)
		{
			*errMsg = "no osg host";
		}
		return;
	}
	std::vector<RobotOsgUi::RawTrajectoryOverlayVertex> overlay;
	std::vector<std::size_t> segmentEnds;
	if (!appendRawTrajectoryOverlayWorld(osg, backendId, fileTraj, overlay, segmentEnds, errMsg))
	{
		return;
	}
	feature_pick_transform::finalizeOverlaySegmentEnds(overlay.size(), segmentEnds);
	applyMergedRawTrajectoryPreviewToOsg(osg, overlay, segmentEnds, backendId, &fileTraj, options, errMsg);
}

void applyWorldRawTrajectoryPreviewToOsg(IRobotOsgViewHost* osg, const RobotInstruction::RawTrajectory& worldTraj,
										 const RobotOsgUi::RawTrajectoryPreviewOptions& options, std::string* errMsg)
{
	if (!osg)
	{
		if (errMsg)
		{
			*errMsg = "no osg host";
		}
		return;
	}
	if (worldTraj.points.empty())
	{
		if (errMsg)
		{
			*errMsg = "empty trajectory";
		}
		return;
	}
	std::vector<RobotOsgUi::RawTrajectoryOverlayVertex> overlay;
	overlay.reserve(worldTraj.points.size());
	for (const RobotInstruction::TrajectoryPoint& tp : worldTraj.points)
	{
		RobotOsgUi::RawTrajectoryOverlayVertex v;
		v.positionMm.set(static_cast<float>(tp.poseMm.x), static_cast<float>(tp.poseMm.y),
						 static_cast<float>(tp.poseMm.z));
		v.reachable = tp.reachable;
		overlay.push_back(v);
	}
	std::vector<RobotOsgUi::RawTrajectoryOverlayFrame> frames;
	if (options.showAxes)
	{
		const std::vector<std::size_t> axisIndices = collectPreviewAxisPointIndices(
			worldTraj.points.size(), options.axisInterval, worldTraj.segmentEndExclusive);
		appendPreviewFramesForIndicesWorld(axisIndices, worldTraj.points, frames);
	}
	osg->clearInstructionPoseAxes();
	osg->setRawTrajectoryOverlay(overlay, worldTraj.segmentEndExclusive);
	if (options.showAxes && !frames.empty())
	{
		osg->setRawTrajectoryOverlayAxisComponents(options.showAxisX, options.showAxisY, options.showAxisZ);
		osg->setRawTrajectoryOverlayFrames(frames);
	}
	else
	{
		osg->clearRawTrajectoryOverlayFrames();
	}
	osg->requestRedraw();
}

} // namespace feature_pick_transform
