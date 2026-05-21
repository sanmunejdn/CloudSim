#pragma once

#include "RobotCoordinateFrames.h"
#include "robotwidget_global.h"
#include "../../Robot/RobotKinematics/inc/SerialLinkKinematics.h"

#include <RigidTransform.h>

#include <QHash>
#include <QString>
#include <QVector>

#include <osg/Matrixd>
#include <osg/Node>
#include <memory>
#include <string>

class IRobotDocumentHost;
class IRobotOsgViewHost;
namespace RobotInstruction { class Base; }

namespace RobotSimulationMath
{

ROBOTWIDGET_EXPORT bool matrixFromNodeWorld(osg::Node* node, osg::Matrixd& outWorld);
ROBOTWIDGET_EXPORT std::string encodeMatrix4Csv(const osg::Matrixd& m);
ROBOTWIDGET_EXPORT bool decodeMatrix4Csv(const std::string& text, osg::Matrixd& out);

ROBOTWIDGET_EXPORT bool buildDhRowsFromUrdf(
	const QString& urdfPath,
	std::vector<robot_kinematics::DhRow>& outRows,
	QString* errMsg = nullptr);

ROBOTWIDGET_EXPORT QString defaultTcpLinkNameForUrdf(const QString& urdfPath, const QString& comboTcpLink);

ROBOTWIDGET_EXPORT BackendMat4 toolMat4ForFrames(
	const RobotCoordinate::RobotCoordinateFrameSet& frames,
	const RobotInstruction::Base* instructionWithTool = nullptr);

ROBOTWIDGET_EXPORT bool targetInBaseFromUrdfFlangeFk(
	const QString& urdfPath,
	const QVector<double>& jointQ,
	const RobotCoordinate::RobotCoordinateFrameSet& frames,
	const QString& fallbackFlangeLink,
	BackendMat4& outTargetInBase,
	const RobotInstruction::Base* instructionWithTool = nullptr,
	QString* outFlangeLinkName = nullptr);

ROBOTWIDGET_EXPORT bool targetRigidTransformFromUrdfFlangeFk(
	const QString& urdfPath,
	const QVector<double>& jointQ,
	const RobotCoordinate::RobotCoordinateFrameSet& frames,
	const QString& fallbackFlangeLink,
	engine::RigidTransform& outTargetInBase,
	QString* outFlangeLinkName = nullptr,
	const RobotInstruction::Base* instructionWithTool = nullptr);

ROBOTWIDGET_EXPORT bool captureTcpFromSceneFlangeBackend(
	IRobotDocumentHost* doc,
	IRobotOsgViewHost* osg,
	int instIdx,
	const RobotCoordinate::RobotCoordinateFrameSet& frames,
	const QString& fallbackFlangeLink,
	const osg::Matrixd& robotBaseWorld,
	osg::Matrixd& outTcpLocal,
	osg::Matrixd& outTcpRenderWorld,
	QString& outTcpLinkName,
	QString& outTcpSource);

ROBOTWIDGET_EXPORT osg::Matrixd osgMatrixFromRobotRigidFrame(const RobotCoordinate::RobotRigidFrame& frame);
ROBOTWIDGET_EXPORT osg::Matrixd osgMatrixFromBackendMat4(const BackendMat4& m);

ROBOTWIDGET_EXPORT bool robotBaseWorldMatrixForInstance(
	IRobotDocumentHost* doc,
	IRobotOsgViewHost* osg,
	int instIdx,
	osg::Matrixd& outWorld,
	const QVector<double>* jointAnglesRad = nullptr);

ROBOTWIDGET_EXPORT QString linkMeshBackendIdForInstance(
	IRobotDocumentHost* doc,
	int instIdx,
	const std::string& linkName);

ROBOTWIDGET_EXPORT bool tcpInBaseFromLinkWorldAndToolFrames(
	const QHash<QString, osg::Matrixd>& linkWorldByName,
	const RobotCoordinate::RobotCoordinateFrameSet& frames,
	const QString& tcpLinkName,
	osg::Matrixd& outTcpInBase,
	QString& outFlangeLink,
	const RobotInstruction::Base* instructionWithTool = nullptr);

ROBOTWIDGET_EXPORT BackendMat4 toolTcpInBaseFromFk(
	const QString& urdfPath,
	const QVector<double>& jointQ,
	const RobotCoordinate::RobotCoordinateFrameSet& frames,
	const RobotCoordinate::RobotToolFrame& tool);

} // namespace RobotSimulationMath
