#include "RobotSimulationMath.h"
#include "IRobotDocumentHost.h"
#include "IRobotOsgViewHost.h"
#include "RobotInstructionTransform.h"
#include "RobotMatrixOsgBridge.h"
#include "UrdfRobotLoader.h"

#include <Adapters.h>
#include <RigidTransform.h>
#include <ToolKinematics.h>

#include <QHash>

namespace RobotSimulationMath
{

BackendMat4 toolMat4ForFrames(
	const RobotCoordinate::RobotCoordinateFrameSet& frames,
	const RobotInstruction::Base* instructionWithTool)
{
	if (instructionWithTool)
	{
		return RobotCoordinate::toolMat4ForExtension(frames, instructionWithTool->extensionProperties());
	}
	if (const RobotCoordinate::RobotToolFrame* tool = RobotCoordinate::activeToolFrame(frames))
	{
		return RobotCoordinate::frameToMat4(tool->T_flange_tool);
	}
	return BackendMat4::identity();
}

static std::string resolveFlangeLinkName(
	const RobotCoordinate::RobotCoordinateFrameSet& frames,
	const QString& fallbackFlangeLink,
	const RobotInstruction::Base* instructionWithTool)
{
	if (instructionWithTool)
	{
		if (const RobotCoordinate::RobotToolFrame* tool = RobotCoordinate::resolveToolFrameForExtension(
				frames, instructionWithTool->extensionProperties()))
		{
			return RobotCoordinate::effectiveFlangeLinkName(frames, *tool);
		}
	}
	if (const RobotCoordinate::RobotToolFrame* tool = RobotCoordinate::activeToolFrame(frames))
	{
		return RobotCoordinate::effectiveFlangeLinkName(frames, *tool);
	}
	if (!frames.flangeLinkName.empty())
	{
		return frames.flangeLinkName;
	}
	return fallbackFlangeLink.toStdString();
}

bool targetInBaseFromUrdfFlangeFk(
	const QString& urdfPath,
	const QVector<double>& jointQ,
	const RobotCoordinate::RobotCoordinateFrameSet& frames,
	const QString& fallbackFlangeLink,
	BackendMat4& outTargetInBase,
	const RobotInstruction::Base* instructionWithTool,
	QString* outFlangeLinkName)
{
	const std::string flangeLink = resolveFlangeLinkName(frames, fallbackFlangeLink, instructionWithTool);
	if (flangeLink.empty())
	{
		return false;
	}
	QHash<QString, osg::Matrixd> linkWorld;
	if (!UrdfRobotLoader::computeLinkWorldMatrices(urdfPath, jointQ, linkWorld, nullptr))
	{
		return false;
	}
	const QString flangeQ = QString::fromStdString(flangeLink);
	if (!linkWorld.contains(flangeQ))
	{
		return false;
	}
	const BackendMat4 T_tool = toolMat4ForFrames(frames, instructionWithTool);
	const engine::RigidTransform T_base_flange =
		engine::rigidTransformFromOsg(linkWorld.value(flangeQ));
	const engine::RigidTransform T_flange_tool =
		RobotCoordinate::rigidTransformFromBackendMat4(T_tool);
	const engine::RigidTransform T_base_target =
		engine::toolOriginFromFlange(T_base_flange, T_flange_tool);
	outTargetInBase = RobotCoordinate::backendMat4FromRigidTransform(T_base_target);
	if (outFlangeLinkName)
	{
		*outFlangeLinkName = flangeQ;
	}
	return true;
}

bool targetRigidTransformFromUrdfFlangeFk(
	const QString& urdfPath,
	const QVector<double>& jointQ,
	const RobotCoordinate::RobotCoordinateFrameSet& frames,
	const QString& fallbackFlangeLink,
	engine::RigidTransform& outTargetInBase,
	QString* outFlangeLinkName,
	const RobotInstruction::Base* instructionWithTool)
{
	BackendMat4 m;
	if (!targetInBaseFromUrdfFlangeFk(
			urdfPath, jointQ, frames, fallbackFlangeLink, m, instructionWithTool, outFlangeLinkName))
	{
		return false;
	}
	outTargetInBase = RobotCoordinate::rigidTransformFromBackendMat4(m);
	return true;
}

QString defaultTcpLinkNameForUrdf(const QString& urdfPath, const QString& comboTcpLink)
{
	if (!comboTcpLink.isEmpty())
	{
		return comboTcpLink;
	}
	QString preferred;
	if (UrdfRobotLoader::loadPrimaryTerminalLinkName(urdfPath, preferred, nullptr) && !preferred.isEmpty())
	{
		return preferred;
	}
	QStringList childLinks;
	if (UrdfRobotLoader::loadRevoluteJointChildLinksInOrder(urdfPath, childLinks, nullptr) && !childLinks.isEmpty())
	{
		return childLinks.back();
	}
	return QString();
}

osg::Matrixd osgMatrixFromRobotRigidFrame(const RobotCoordinate::RobotRigidFrame& frame)
{
	return engine::osgMatrixFromRigidTransform(RobotCoordinate::rigidTransformFromFrame(frame));
}

osg::Matrixd osgMatrixFromBackendMat4(const BackendMat4& m)
{
	return RobotMatrixOsg::matrixFromBackendColMajor(m);
}

QString linkMeshBackendIdForInstance(IRobotDocumentHost* doc, int instIdx, const std::string& linkName)
{
	if (!doc || instIdx < 0)
	{
		return QString();
	}
	if (doc->robotUsesPerLinkBackendsForInstance(instIdx))
	{
		RobotPerLinkKinematicsSlice slice;
		if (doc->robotPerLinkKinematicsForInstance(instIdx, slice))
		{
			return slice.linkNameToBackendId.value(QString::fromStdString(linkName));
		}
	}
	return doc->robotLinkNameToBackendId().value(QString::fromStdString(linkName));
}

bool perLinkUsesWorldBakedMeshVertices(IRobotDocumentHost* doc, int instIdx)
{
	if (!doc || instIdx < 0 || !doc->robotUsesPerLinkBackendsForInstance(instIdx))
	{
		return false;
	}
	RobotPerLinkKinematicsSlice slice;
	if (!doc->robotPerLinkKinematicsForInstance(instIdx, slice))
	{
		return false;
	}
	return !slice.meshVerticesInLinkFrame;
}

QString urdfRootLinkBackendIdForInstance(
	IRobotDocumentHost* doc,
	int instIdx,
	const QString& urdfPath,
	const QString& fallbackBackendId)
{
	QString rootLinkName;
	QHash<QString, QString> linkMeshes;
	if (UrdfRobotLoader::enumerateLinkVisualMeshes(urdfPath, rootLinkName, linkMeshes, nullptr)
		&& !rootLinkName.isEmpty())
	{
		const QString rootBackendId = linkMeshBackendIdForInstance(doc, instIdx, rootLinkName.toStdString());
		if (!rootBackendId.isEmpty())
		{
			return rootBackendId;
		}
	}
	return fallbackBackendId;
}

bool robotBaseWorldMatrixForInstance(
	IRobotDocumentHost* doc,
	IRobotOsgViewHost* osg,
	int instIdx,
	osg::Matrixd& outWorld,
	const QVector<double>* jointAnglesRad)
{
	(void)jointAnglesRad;
	(void)osg;
	if (!doc || instIdx < 0)
	{
		return false;
	}
	outWorld.makeIdentity();
	// per-link：基座↔世界须用 basePlacementWorld，根连杆 mesh 世界矩阵含连杆偏置
	if (doc->robotUsesPerLinkBackendsForInstance(instIdx))
	{
		RobotPerLinkKinematicsSlice slice;
		if (doc->robotPerLinkKinematicsForInstance(instIdx, slice))
		{
			outWorld = slice.robotBasePlacementWorld;
			return true;
		}
	}
	const QString sceneRootId = doc->robotSceneBackendIdForInstance(instIdx);
	if (osg && !sceneRootId.isEmpty() && osg->getBackendRootWorldMatrix(sceneRootId.toStdString(), outWorld))
	{
		return true;
	}
	QString refId = doc->robotFrameWorldReferenceBackendId(instIdx);
	if (refId.isEmpty())
	{
		refId = sceneRootId;
	}
	if (osg && !refId.isEmpty() && osg->getBackendRootWorldMatrix(refId.toStdString(), outWorld))
	{
		return true;
	}
	RobotPerLinkKinematicsSlice slice;
	if (doc->robotPerLinkKinematicsForInstance(instIdx, slice))
	{
		outWorld = slice.robotBasePlacementWorld;
		return true;
	}
	return false;
}

bool tcpInBaseFromLinkWorldAndToolFrames(
	const QHash<QString, osg::Matrixd>& linkWorldByName,
	const RobotCoordinate::RobotCoordinateFrameSet& frames,
	const QString& tcpLinkName,
	osg::Matrixd& outTcpInBase,
	QString& outFlangeLink,
	const RobotInstruction::Base* instructionWithTool)
{
	const std::string flange = resolveFlangeLinkName(frames, tcpLinkName, instructionWithTool);
	if (flange.empty())
	{
		return false;
	}
	const QString flangeQ = QString::fromStdString(flange);
	if (!linkWorldByName.contains(flangeQ))
	{
		return false;
	}
	const BackendMat4 T_tool = toolMat4ForFrames(frames, instructionWithTool);
	outTcpInBase = RobotMatrixOsg::matrixFromBackendColMajor(
		RobotMatrixOsg::targetInBaseFromFlangeLinkWorld(linkWorldByName.value(flangeQ), T_tool));
	outFlangeLink = flangeQ;
	return true;
}

BackendMat4 toolTcpInBaseFromFk(
	const QString& urdfPath,
	const QVector<double>& jointQ,
	const RobotCoordinate::RobotCoordinateFrameSet& frames,
	const RobotCoordinate::RobotToolFrame& tool)
{
	BackendMat4 out = BackendMat4::identity();
	const std::string flangeLink = RobotCoordinate::effectiveFlangeLinkName(frames, tool);
	if (flangeLink.empty())
	{
		return out;
	}
	QHash<QString, osg::Matrixd> linkWorld;
	if (!UrdfRobotLoader::computeLinkWorldMatrices(urdfPath, jointQ, linkWorld, nullptr))
	{
		return out;
	}
	const QString flangeQ = QString::fromStdString(flangeLink);
	if (!linkWorld.contains(flangeQ))
	{
		return out;
	}
	const BackendMat4 T_tool = RobotCoordinate::frameToMat4(tool.T_flange_tool);
	const engine::RigidTransform T_base_flange =
		engine::rigidTransformFromOsg(linkWorld.value(flangeQ));
	const engine::RigidTransform T_flange_tool =
		RobotCoordinate::rigidTransformFromBackendMat4(T_tool);
	const engine::RigidTransform T_base_target =
		engine::toolOriginFromFlange(T_base_flange, T_flange_tool);
	out = RobotCoordinate::backendMat4FromRigidTransform(T_base_target);
	return out;
}

bool captureTcpFromSceneFlangeBackend(
	IRobotDocumentHost* doc,
	IRobotOsgViewHost* osg,
	int instIdx,
	const RobotCoordinate::RobotCoordinateFrameSet& frames,
	const QString& fallbackFlangeLink,
	const osg::Matrixd& robotBaseWorld,
	osg::Matrixd& outTcpLocal,
	osg::Matrixd& outTcpRenderWorld,
	QString& outTcpLinkName,
	QString& outTcpSource)
{
	if (!doc || !osg)
	{
		return false;
	}
	const std::string flangeLink = resolveFlangeLinkName(frames, fallbackFlangeLink, nullptr);
	if (flangeLink.empty())
	{
		return false;
	}
	const QString backendId = linkMeshBackendIdForInstance(doc, instIdx, flangeLink);
	if (backendId.isEmpty())
	{
		return false;
	}
	osg::Matrixd flangeWorld;
	if (!osg->getBackendRootWorldMatrix(backendId.toStdString(), flangeWorld))
	{
		return false;
	}
	const BackendMat4 T_tool = toolMat4ForFrames(frames, nullptr);
	const osg::Matrixd toolWorld = RobotMatrixOsg::matrixFromBackendColMajor(
		RobotMatrixOsg::targetInBaseFromFlangeLinkWorld(flangeWorld, T_tool));
	osg::Matrixd invBase;
	invBase.makeIdentity();
	if (!robotBaseWorld.isIdentity())
	{
		invBase = osg::Matrixd::inverse(robotBaseWorld);
	}
	outTcpLocal = invBase * toolWorld;
	outTcpRenderWorld = toolWorld;
	outTcpLinkName = QString::fromStdString(flangeLink);
	outTcpSource = QStringLiteral("SceneFlangeBackend");
	return true;
}

} // namespace RobotSimulationMath
