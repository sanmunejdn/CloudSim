#include "FeaturePickTransform.h"

#include "RobotOsgUiTypes.h"

#include "../../OsgWidgetCore/inc/OsgScene.h"

namespace feature_pick_transform
{

bool transformTrajectoryPointToWorld(
	IRobotOsgViewHost* osg,
	const std::string& backendId,
	const RobotInstruction::TrajectoryPoint& filePoint,
	RobotInstruction::TrajectoryPoint& outWorld,
	std::string* errMsg)
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
	const osg::Quat qFile = OsgScene::eulerDegToQuat(osg::Vec3f(
		static_cast<float>(filePoint.eulerDeg.x),
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

bool transformRawTrajectoryToWorld(
	IRobotOsgViewHost* osg,
	const std::string& backendId,
	const RobotInstruction::RawTrajectory& fileTraj,
	RobotInstruction::RawTrajectory& outWorld,
	std::string* errMsg)
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

bool buildRawTrajectoryOverlayWorld(
	IRobotOsgViewHost* osg,
	const std::string& backendId,
	const RobotInstruction::RawTrajectory& fileTraj,
	std::vector<RobotOsgUi::RawTrajectoryOverlayVertex>& outOverlay,
	std::string* errMsg)
{
	outOverlay.clear();
	if (fileTraj.points.empty())
	{
		if (errMsg)
		{
			*errMsg = "empty trajectory";
		}
		return false;
	}
	outOverlay.reserve(fileTraj.points.size());
	for (const RobotInstruction::TrajectoryPoint& tp : fileTraj.points)
	{
		geoalgo::Point3d filePos{tp.poseMm.x, tp.poseMm.y, tp.poseMm.z};
		osg::Vec3f worldPos;
		if (!stepModelPointToWorldMm(osg, backendId, filePos, worldPos, errMsg))
		{
			return false;
		}
		RobotOsgUi::RawTrajectoryOverlayVertex v;
		v.positionMm = worldPos;
		v.reachable = tp.reachable;
		outOverlay.push_back(v);
	}
	return true;
}

bool buildRawTrajectoryPreviewWorld(
	IRobotOsgViewHost* osg,
	const std::string& backendId,
	const RobotInstruction::RawTrajectory& fileTraj,
	const RobotOsgUi::RawTrajectoryPreviewOptions& options,
	std::vector<RobotOsgUi::RawTrajectoryOverlayVertex>& outOverlay,
	std::vector<RobotOsgUi::RawTrajectoryOverlayFrame>& outFrames,
	std::string* errMsg)
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
	const std::size_t n = fileTraj.points.size();
	const int autoInterval = std::max(1, static_cast<int>(n / 20U));
	const int interval = options.axisInterval > 0 ? options.axisInterval : autoInterval;
	const int maxAxes = options.maxAxes > 0 ? options.maxAxes : 50;
	for (std::size_t i = 0; i < n; ++i)
	{
		const bool isEnd = (i == 0U || i + 1U == n);
		if (!isEnd && interval > 1 && static_cast<int>(i % static_cast<std::size_t>(interval)) != 0)
		{
			continue;
		}
		if (static_cast<int>(outFrames.size()) >= maxAxes)
		{
			break;
		}
		RobotInstruction::TrajectoryPoint worldTp;
		if (!transformTrajectoryPointToWorld(osg, backendId, fileTraj.points[i], worldTp, errMsg))
		{
			return false;
		}
		RobotOsgUi::RawTrajectoryOverlayFrame frame;
		frame.positionMm.set(
			static_cast<float>(worldTp.poseMm.x),
			static_cast<float>(worldTp.poseMm.y),
			static_cast<float>(worldTp.poseMm.z));
		frame.eulerDeg.set(
			static_cast<float>(worldTp.eulerDeg.x),
			static_cast<float>(worldTp.eulerDeg.y),
			static_cast<float>(worldTp.eulerDeg.z));
		frame.reachable = worldTp.reachable;
		outFrames.push_back(frame);
	}
	return true;
}

void applyRawTrajectoryPreviewToOsg(
	IRobotOsgViewHost* osg,
	const std::string& backendId,
	const RobotInstruction::RawTrajectory& fileTraj,
	const RobotOsgUi::RawTrajectoryPreviewOptions& options,
	std::string* errMsg)
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
	std::vector<RobotOsgUi::RawTrajectoryOverlayFrame> frames;
	if (!buildRawTrajectoryPreviewWorld(osg, backendId, fileTraj, options, overlay, frames, errMsg))
	{
		return;
	}
	osg->clearInstructionPoseAxes();
	osg->setRawTrajectoryOverlay(overlay);
	if (options.showAxes && !frames.empty())
	{
		osg->setRawTrajectoryOverlayFrames(frames);
	}
	else
	{
		osg->clearRawTrajectoryOverlayFrames();
	}
	osg->requestRedraw();
}

} // namespace feature_pick_transform
