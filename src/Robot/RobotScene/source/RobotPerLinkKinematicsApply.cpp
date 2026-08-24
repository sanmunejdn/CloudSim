#include "RobotPerLinkKinematicsApply.h"

#include "BackendDataManager.h"
#include "IRobotBackendPoseSink.h"
#include "MeshBackendData.h"
#include "RobotMatrixOsgBridge.h"
#include "RobotSceneKinematics.h"

#include <QVector>

namespace RobotPerLinkKinematicsApply
{
namespace
{
BackendMat4 osgMatToBackendColMajor(const osg::Matrixd& m)
{
	return RobotMatrixOsg::backendColMajorFromMatrix(m);
}
} // namespace

bool applyLinkWorldFromCoreFk(IRobotBackendPoseSink* osg, BackendDataManager& mgr,
							  const RobotPerLinkKinematicsSlice& slice,
							  const QHash<QString, osg::Matrixd>& meshWorldTq)
{
	const QHash<QString, QString>& linkToId = slice.linkNameToBackendId;
	if (linkToId.isEmpty())
	{
		return false;
	}
	const QHash<QString, osg::Matrixd>& T0 = slice.fkMeshWorldT0;
	const QHash<QString, osg::Matrixd>& m0bind = slice.outerWorldAtBindByBackendId;

	QVector<QString> linkOrder;
	RobotSceneKinematics::resolveRobotLinkUpdateOrder(&mgr, linkToId, linkOrder);

	bool any = false;
	for (const QString& linkName : linkOrder)
	{
		const QString backendIdStr = linkToId.value(linkName);
		if (backendIdStr.isEmpty())
		{
			continue;
		}
		const std::string bid = backendIdStr.toStdString();
		const auto meshPtr = std::dynamic_pointer_cast<MeshBackendData>(mgr.getData(bid));
		if (!meshPtr)
		{
			continue;
		}
		if (!meshWorldTq.contains(linkName) || !T0.contains(linkName))
		{
			continue;
		}
		const auto m0It = m0bind.constFind(backendIdStr);
		if (m0It == m0bind.cend())
		{
			continue;
		}
		const osg::Matrixd Mnew =
			m0It.value() * osg::Matrixd::inverse(T0[linkName]) * meshWorldTq[linkName] * slice.robotBasePlacementWorld;
		meshPtr->setWorldMatrix(osgMatToBackendColMajor(Mnew), &mgr);
		if (osg)
		{
			osg->syncRobotMeshBackendPoseAfterKinematics(*meshPtr);
		}
		any = true;
	}
	return any;
}

} // namespace RobotPerLinkKinematicsApply
