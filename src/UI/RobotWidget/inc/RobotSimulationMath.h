#ifndef ROBOTWIDGET_ROBOTSIMULATIONMATH_H
#define ROBOTWIDGET_ROBOTSIMULATIONMATH_H

/// @file RobotSimulationMath.h
/// @brief per-link 且顶点已烘焙到世界系（meshVerticesInLinkFrame=false）时，法兰 OSG 外矩阵仅为 FK 增量，工具轴须走 FK 基系矩阵

#include "robotwidget_global.h"

#include "../../Robot/RobotKinematics/inc/SerialLinkKinematics.h"
#include "CoreTypes.h"
#include "RobotCoordinateFrames.h"

#include <QHash>
#include <QString>
#include <QVector>
#include <memory>
#include <string>

#include <RigidTransform.h>
#include <osg/Matrixd>
#include <osg/Node>

class IRobotDocumentHost;
class IRobotOsgViewHost;
namespace RobotInstruction
{
class Base;
}

namespace RobotSimulationMath
{
ROBOTWIDGET_EXPORT bool matrixFromNodeWorld(osg::Node* node, osg::Matrixd& outWorld);
ROBOTWIDGET_EXPORT std::string encodeMatrix4Csv(const osg::Matrixd& m);
ROBOTWIDGET_EXPORT bool decodeMatrix4Csv(const std::string& text, osg::Matrixd& out);

/// core::Mat4 列主序 ↔ osg::Matrixd（禁止 ptr 直拷）
ROBOTWIDGET_EXPORT osg::Matrixd osgMatrixFromCoreMat4(const cloudsim::core::Mat4& columnMajor);
ROBOTWIDGET_EXPORT cloudsim::core::Mat4 coreMat4FromOsgMatrix(const osg::Matrixd& m);
ROBOTWIDGET_EXPORT bool getBackendRootWorldMatrixOsg(IRobotOsgViewHost* view, const std::string& backendId,
													 osg::Matrixd& outWorld);

ROBOTWIDGET_EXPORT bool buildDhRowsFromUrdf(const QString& urdfPath, std::vector<robot_kinematics::DhRow>& outRows,
											QString* errMsg = nullptr);

ROBOTWIDGET_EXPORT QString defaultTcpLinkNameForUrdf(const QString& urdfPath, const QString& comboTcpLink);

ROBOTWIDGET_EXPORT BackendMat4 toolMat4ForFrames(const RobotCoordinate::RobotCoordinateFrameSet& frames,
												 const RobotInstruction::Base* instructionWithTool = nullptr);

ROBOTWIDGET_EXPORT bool targetInBaseFromUrdfFlangeFk(const QString& urdfPath, const QVector<double>& jointQ,
													 const RobotCoordinate::RobotCoordinateFrameSet& frames,
													 const QString& fallbackFlangeLink, BackendMat4& outTargetInBase,
													 const RobotInstruction::Base* instructionWithTool = nullptr,
													 QString* outFlangeLinkName = nullptr);

ROBOTWIDGET_EXPORT bool targetRigidTransformFromUrdfFlangeFk(
	const QString& urdfPath, const QVector<double>& jointQ, const RobotCoordinate::RobotCoordinateFrameSet& frames,
	const QString& fallbackFlangeLink, engine::RigidTransform& outTargetInBase, QString* outFlangeLinkName = nullptr,
	const RobotInstruction::Base* instructionWithTool = nullptr);

ROBOTWIDGET_EXPORT bool captureTcpFromSceneFlangeBackend(IRobotDocumentHost* doc, IRobotOsgViewHost* osg, int instIdx,
														 const RobotCoordinate::RobotCoordinateFrameSet& frames,
														 const QString& fallbackFlangeLink,
														 const osg::Matrixd& robotBaseWorld, osg::Matrixd& outTcpLocal,
														 osg::Matrixd& outTcpRenderWorld, QString& outTcpLinkName,
														 QString& outTcpSource);

ROBOTWIDGET_EXPORT osg::Matrixd osgMatrixFromRobotRigidFrame(const RobotCoordinate::RobotRigidFrame& frame);
ROBOTWIDGET_EXPORT osg::Matrixd osgMatrixFromBackendMat4(const BackendMat4& m);

ROBOTWIDGET_EXPORT bool robotBaseWorldMatrixForInstance(IRobotDocumentHost* doc, IRobotOsgViewHost* osg, int instIdx,
														osg::Matrixd& outWorld,
														const QVector<double>* jointAnglesRad = nullptr);

ROBOTWIDGET_EXPORT QString linkMeshBackendIdForInstance(IRobotDocumentHost* doc, int instIdx,
														const std::string& linkName);

/// per-link 且顶点已烘焙到世界系（meshVerticesInLinkFrame=false）时，法兰 OSG 外矩阵仅为 FK 增量，工具轴须走 FK 基系矩阵
ROBOTWIDGET_EXPORT bool perLinkUsesWorldBakedMeshVertices(IRobotDocumentHost* doc, int instIdx);

ROBOTWIDGET_EXPORT QString urdfRootLinkBackendIdForInstance(IRobotDocumentHost* doc, int instIdx,
															const QString& urdfPath, const QString& fallbackBackendId);

ROBOTWIDGET_EXPORT bool
tcpInBaseFromLinkWorldAndToolFrames(const QHash<QString, osg::Matrixd>& linkWorldByName,
									const RobotCoordinate::RobotCoordinateFrameSet& frames, const QString& tcpLinkName,
									osg::Matrixd& outTcpInBase, QString& outFlangeLink,
									const RobotInstruction::Base* instructionWithTool = nullptr);

ROBOTWIDGET_EXPORT BackendMat4 toolTcpInBaseFromFk(const QString& urdfPath, const QVector<double>& jointQ,
												   const RobotCoordinate::RobotCoordinateFrameSet& frames,
												   const RobotCoordinate::RobotToolFrame& tool);

/// per-link 且几何在 mesh 文件系时：连杆系局部矩阵 → 挂 mesh backend 的 OSG 局部矩阵
ROBOTWIDGET_EXPORT osg::Matrixd linkFrameLocalOnMeshBackend(const QString& urdfPath, const QString& linkName,
															const osg::Matrixd& linkFrameLocal,
															bool meshVerticesInLinkFrame);

} // namespace RobotSimulationMath

#endif // ROBOTWIDGET_ROBOTSIMULATIONMATH_H
