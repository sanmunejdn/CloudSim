#include "FeaturePickTransform.h"

#include "RobotOsgUiTypes.h"

#include "../../OsgWidgetCore/inc/OsgScene.h"
#include <sstream>

namespace feature_pick_transform
{

namespace {

std::string transformBackendId(IRobotOsgViewHost* osg, const std::string& backendId)
{
	if (!osg)
	{
		return backendId;
	}
	return osg->resolvePickScopeBackendId(backendId);
}

} // namespace

bool worldPointToStepModelMm(
	IRobotOsgViewHost* osg,
	const std::string& backendId,
	const osg::Vec3f& worldMm,
	geoalgo::Point3d& outModel,
	std::string* errMsg)
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

bool stepModelPointToWorldMm(
	IRobotOsgViewHost* osg,
	const std::string& backendId,
	const geoalgo::Point3d& modelMm,
	osg::Vec3f& outWorld,
	std::string* errMsg)
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
	outWorld.set(
		static_cast<float>(pw.x()),
		static_cast<float>(pw.y()),
		static_cast<float>(pw.z()));
	return true;
}

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

bool transformTrajectoryPointToFile(
	IRobotOsgViewHost* osg,
	const std::string& backendId,
	const RobotInstruction::TrajectoryPoint& worldPoint,
	RobotInstruction::TrajectoryPoint& outFile,
	std::string* errMsg)
{
	geoalgo::Point3d modelPos{};
	const osg::Vec3f worldMm(
		static_cast<float>(worldPoint.poseMm.x),
		static_cast<float>(worldPoint.poseMm.y),
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
	const osg::Quat qWorld = OsgScene::eulerDegToQuat(osg::Vec3f(
		static_cast<float>(worldPoint.eulerDeg.x),
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

bool transformRawTrajectoryWorldToFile(
	IRobotOsgViewHost* osg,
	const std::string& backendId,
	const RobotInstruction::RawTrajectory& worldTraj,
	RobotInstruction::RawTrajectory& outFile,
	std::string* errMsg)
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
	osg->setRawTrajectoryOverlay(overlay, fileTraj.segmentEndExclusive);
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

void applyWorldRawTrajectoryPreviewToOsg(
	IRobotOsgViewHost* osg,
	const RobotInstruction::RawTrajectory& worldTraj,
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
		v.positionMm.set(
			static_cast<float>(tp.poseMm.x),
			static_cast<float>(tp.poseMm.y),
			static_cast<float>(tp.poseMm.z));
		v.reachable = tp.reachable;
		overlay.push_back(v);
	}
	std::vector<RobotOsgUi::RawTrajectoryOverlayFrame> frames;
	if (options.showAxes)
	{
		const std::size_t n = worldTraj.points.size();
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
			if (static_cast<int>(frames.size()) >= maxAxes)
			{
				break;
			}
			const RobotInstruction::TrajectoryPoint& tp = worldTraj.points[i];
			RobotOsgUi::RawTrajectoryOverlayFrame frame;
			frame.positionMm.set(
				static_cast<float>(tp.poseMm.x),
				static_cast<float>(tp.poseMm.y),
				static_cast<float>(tp.poseMm.z));
			frame.eulerDeg.set(
				static_cast<float>(tp.eulerDeg.x),
				static_cast<float>(tp.eulerDeg.y),
				static_cast<float>(tp.eulerDeg.z));
			frame.reachable = tp.reachable;
			frames.push_back(frame);
		}
	}
	osg->clearInstructionPoseAxes();
	osg->setRawTrajectoryOverlay(overlay, worldTraj.segmentEndExclusive);
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
