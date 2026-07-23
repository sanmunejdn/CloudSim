#ifndef ROBOTSCENE_ROBOTPERLINKKINEMATICSSLICEOSG_H
#define ROBOTSCENE_ROBOTPERLINKKINEMATICSSLICEOSG_H

/// @file RobotPerLinkKinematicsSliceOsg.h
/// @brief per-link FK 切片的 OSG 形态（仅 RobotScene / Host 适配内使用）

#include "robot_scene_global.h"

#include "CoreTypes.h"

#include <QHash>
#include <QString>

#include <osg/Matrixd>

/// 单台机器人 per-link FK 切片（OSG 矩阵；由 Core DTO 转换而来）
struct ROBOT_SCENE_API RobotPerLinkKinematicsSlice
{
	QString urdfAbsolutePath;
	QString sceneRootBackendId;
	QHash<QString, QString> linkNameToBackendId;
	QHash<QString, osg::Matrixd> fkMeshWorldT0;
	QHash<QString, osg::Matrixd> outerWorldAtBindByBackendId;
	osg::Matrixd robotBasePlacementWorld;
	bool meshVerticesInLinkFrame = false;
};

namespace RobotSceneKinematics
{
inline osg::Matrixd osgMatrixFromCoreMat4(const cloudsim::core::Mat4& columnMajor)
{
	osg::Matrixd m;
	for (int c = 0; c < 4; ++c)
	{
		for (int r = 0; r < 4; ++r)
		{
			m(r, c) = columnMajor[static_cast<size_t>(c * 4 + r)];
		}
	}
	return m;
}

inline cloudsim::core::Mat4 coreMat4FromOsgMatrix(const osg::Matrixd& m)
{
	cloudsim::core::Mat4 out{};
	for (int c = 0; c < 4; ++c)
	{
		for (int r = 0; r < 4; ++r)
		{
			out[static_cast<size_t>(c * 4 + r)] = m(r, c);
		}
	}
	return out;
}

inline RobotPerLinkKinematicsSlice robotPerLinkSliceFromDto(const cloudsim::core::RobotPerLinkKinematicsSliceDto& d)
{
	RobotPerLinkKinematicsSlice s;
	s.urdfAbsolutePath = d.urdfAbsolutePath;
	s.sceneRootBackendId = d.sceneRootBackendId;
	s.linkNameToBackendId = d.linkNameToBackendId;
	s.meshVerticesInLinkFrame = d.meshVerticesInLinkFrame;
	s.robotBasePlacementWorld = osgMatrixFromCoreMat4(d.robotBasePlacementWorld);
	for (auto it = d.fkMeshWorldT0.constBegin(); it != d.fkMeshWorldT0.constEnd(); ++it)
	{
		s.fkMeshWorldT0.insert(it.key(), osgMatrixFromCoreMat4(it.value()));
	}
	for (auto it = d.outerWorldAtBindByBackendId.constBegin(); it != d.outerWorldAtBindByBackendId.constEnd(); ++it)
	{
		s.outerWorldAtBindByBackendId.insert(it.key(), osgMatrixFromCoreMat4(it.value()));
	}
	return s;
}

} // namespace RobotSceneKinematics

#endif // ROBOTSCENE_ROBOTPERLINKKINEMATICSSLICEOSG_H
