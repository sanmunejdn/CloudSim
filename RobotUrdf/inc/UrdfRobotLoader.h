#pragma once

#include <QString>
#include <QStringList>
#include <QHash>
#include <QVector>
#include <vector>

#include <osg/Matrixd>

#include "MeshBackendData.h"

#include "robot_urdf_global.h"

/// Parses a URDF file, resolves mesh paths under the package root, and fills MeshHierarchyPart
/// with triangle soups in one world frame (revolute joints at zero angle).
///
/// Conventions (ROS URDF):
/// - Joint origin is the rigid transform from child link frame to parent link frame:
///   p_parent = R(rpy) * p_child + xyz, with R = Rz(yaw)*Ry(pitch)*Rx(roll) on column vectors.
/// - Link visual origin: p_link = R_vis * p_mesh (mesh vertices in visual / mesh file frame).
/// - Revolute / continuous: child pose includes origin * rotation about axis (axis in joint frame).
/// - After the kinematic chain, an optional fixed rotation maps ROS Z-up (REP-103) to the viewer Y-up
///   (see kApplyRosZUpToOsgYUp in UrdfRobotLoader.cpp). Disable if your meshes already match the viewer.
namespace UrdfRobotLoader
{
	ROBOT_URDF_API bool loadMeshHierarchyParts(const QString& urdfFilePath, std::vector<MeshHierarchyPart>& outParts, QString* errorMessage);

	/// Revolute + continuous joints in BFS traversal order (matches jointAnglesRad indices in computeMeshWorldMatrices).
	ROBOT_URDF_API bool loadRevoluteJointNamesInOrder(const QString& urdfFilePath, QStringList& outJointNames, QString* errorMessage = nullptr);

	/// Same order as \ref computeMeshWorldMatrices joint vector. Limits from URDF <limit> (rad); if missing, revolute uses
	/// [-pi, pi], continuous [-2pi, 2pi].
	ROBOT_URDF_API bool loadRevoluteJointMeta(const QString& urdfFilePath,
		QStringList& outJointNames,
		QVector<double>& outLowerRad,
		QVector<double>& outUpperRad,
		QString* errorMessage = nullptr);

	/// Forward kinematics: OSG world matrix from mesh file frame to viewer frame (same convention as mesh baking).
	/// \a jointAnglesRad must align with loadRevoluteJointNamesInOrder; if shorter, missing entries are treated as 0.
	/// Uses an in-memory cache of the parsed URDF (keyed by canonical path + mtime); only the FK walk runs per call.
	ROBOT_URDF_API bool computeMeshWorldMatrices(
		const QString& urdfFilePath,
		const QVector<double>& jointAnglesRad,
		QHash<QString, osg::Matrixd>& outLinkNameToMeshWorld,
		QString* errorMessage = nullptr);

	/// Drops cached parse trees (e.g. after replacing URDF on disk in edge cases). Normally unnecessary: cache keys include mtime.
	ROBOT_URDF_API void clearUrdfModelCache();
}
