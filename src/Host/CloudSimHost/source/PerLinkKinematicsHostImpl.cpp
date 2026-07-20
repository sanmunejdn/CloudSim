/// @file PerLinkKinematicsHostImpl.cpp
/// @brief PerLinkKinematicsHostImpl 实现

#include "PerLinkKinematicsHostImpl.h"

#include "BackendDataBase.h"
#include "BackendDataManager.h"
#include "IRobotBackendPoseSink.h"
#include "RobotSceneKinematics.h"
#include "UrdfRobotLoader.h"

#include <osg/Matrixd>

namespace
{
osg::Matrixd osgFromMat4(const cloudsim::core::Mat4& m)
{
	osg::Matrixd out;
	for (int c = 0; c < 4; ++c)
	{
		for (int r = 0; r < 4; ++r)
		{
			out(r, c) = m[static_cast<size_t>(c * 4 + r)];
		}
	}
	return out;
}

cloudsim::core::Mat4 mat4FromOsg(const osg::Matrixd& m)
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

} // namespace

namespace cloudsim::host
{
PerLinkKinematicsHostImpl::PerLinkKinematicsHostImpl(IPerLinkRobotStateAccessor* accessor) : m_accessor(accessor) {}

bool PerLinkKinematicsHostImpl::applyPerLinkRobotFkFromGizmoAnchor(int instanceIndex,
																   const QString& anchorLinkBackendId,
																   const QVector<double>& jointAnglesRad)
{
	if (!m_accessor || instanceIndex < 0 || anchorLinkBackendId.isEmpty())
	{
		return false;
	}

	auto snap = m_accessor->extractPerLinkStateSnapshot(instanceIndex);
	if (snap.instanceIndex < 0 || snap.linkNameToBackendId.isEmpty())
	{
		return false;
	}

	IRobotBackendPoseSink* poseSink = m_accessor->urdfImportScenePoseSink();
	if (!poseSink)
	{
		return false;
	}

	// 转换快照为 RobotSceneKinematics 需要的格式
	RobotPerLinkKinematicsSlice slice;
	slice.urdfAbsolutePath = snap.urdfAbsolutePath;
	slice.sceneRootBackendId = QString(); // 不使用
	slice.linkNameToBackendId = snap.linkNameToBackendId;
	for (auto it = snap.fkMeshWorldT0.constBegin(); it != snap.fkMeshWorldT0.constEnd(); ++it)
	{
		slice.fkMeshWorldT0.insert(it.key(), osgFromMat4(it.value()));
	}
	for (auto it = snap.outerWorldAtBindByBackendId.constBegin(); it != snap.outerWorldAtBindByBackendId.constEnd();
		 ++it)
	{
		slice.outerWorldAtBindByBackendId.insert(it.key(), osgFromMat4(it.value()));
	}
	slice.robotBasePlacementWorld = osgFromMat4(snap.basePlacementWorld);
	slice.meshVerticesInLinkFrame = snap.meshVerticesInLinkFrame;

	osg::Matrixd anchorWorld;
	if (!poseSink->getBackendRootWorldMatrix(anchorLinkBackendId.toStdString(), anchorWorld))
	{
		return false;
	}

	osg::Matrixd placement;
	if (!RobotSceneKinematics::computeBasePlacementFromAnchorLinkWorld(slice, anchorLinkBackendId, jointAnglesRad,
																	   anchorWorld, placement))
	{
		return false;
	}

	slice.robotBasePlacementWorld = placement;
	if (!RobotSceneKinematics::applyPerLinkRobotBasePlacement(poseSink, m_accessor->backend(), slice, jointAnglesRad,
															  placement))
	{
		return false;
	}

	PerLinkRobotFkResult result;
	result.computedPlacementWorld = mat4FromOsg(placement);
	result.success = true;

	// 回填 root backend pose（与原有逻辑一致）
	const QString sceneRootId = snap.linkNameToBackendId.isEmpty() ? QString() : QString();
	// 注意：sceneRootId 需由调用方或快照提供；此处简化处理，实际可扩展快照字段
	(void)sceneRootId;

	m_accessor->applyPerLinkFkResult(result);
	return true;
}

void PerLinkKinematicsHostImpl::reconcilePerLinkOuterBindFromScene(int instanceIndex,
																   const QVector<double>& jointAnglesRad)
{
	if (!m_accessor || instanceIndex < 0)
	{
		return;
	}

	auto snap = m_accessor->extractPerLinkStateSnapshot(instanceIndex);
	if (snap.instanceIndex < 0 || snap.linkNameToBackendId.isEmpty() || snap.urdfAbsolutePath.isEmpty())
	{
		return;
	}

	IRobotBackendPoseSink* poseSink = m_accessor->urdfImportScenePoseSink();
	if (!poseSink)
	{
		return;
	}

	QHash<QString, osg::Matrixd> Tq;
	QString fkErr;
	if (!UrdfRobotLoader::computeMeshWorldMatrices(snap.urdfAbsolutePath, jointAnglesRad, Tq, &fkErr,
												   snap.meshVerticesInLinkFrame))
	{
		return;
	}

	const osg::Matrixd Pinv = osg::Matrixd::inverse(osgFromMat4(snap.basePlacementWorld));
	for (auto it = snap.linkNameToBackendId.constBegin(); it != snap.linkNameToBackendId.constEnd(); ++it)
	{
		const QString& linkName = it.key();
		const QString& linkBackendId = it.value();
		if (linkName.isEmpty() || linkBackendId.isEmpty())
		{
			continue;
		}
		const auto t0It = snap.fkMeshWorldT0.constFind(linkName);
		const auto tqIt = Tq.constFind(linkName);
		if (t0It == snap.fkMeshWorldT0.constEnd() || tqIt == Tq.constEnd())
		{
			continue;
		}
		osg::Matrixd world;
		if (!poseSink->getBackendRootWorldMatrix(linkBackendId.toStdString(), world))
		{
			continue;
		}
		const osg::Matrixd Mnew = world * Pinv * tqIt.value() * osgFromMat4(t0It.value());
		poseSink->setBackendRootWorldMatrix(linkBackendId.toStdString(), Mnew);

		PerLinkRobotFkResult partial;
		partial.updatedOuterWorlds.insert(linkBackendId, mat4FromOsg(Mnew));
		partial.success = true;
		m_accessor->applyPerLinkFkResult(partial);
	}
}

} // namespace cloudsim::host