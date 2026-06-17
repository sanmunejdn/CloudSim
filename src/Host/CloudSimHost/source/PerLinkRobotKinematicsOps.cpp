#include "DocumentHost.h"

#include "IRobotUrdfImportContext.h"
#include "BackendDataManager.h"
#include "BackendDataBase.h"

#include <osg/Matrixd>

#include "RobotSceneKinematics.h"
#include "UrdfRobotLoader.h"
#include "OsgScene.h"

#include <QVector>

namespace cloudsim::host {

bool DocumentHost::applyPerLinkRobotFkFromGizmoAnchor(
	int instanceIndex,
	const QString& anchorLinkBackendId,
	const QVector<double>& jointAnglesRad)
{
	// 由 DocumentPage 实现具体逻辑；基类默认返回 false
	// 子类 DocumentPage 重写此方法
	(void)instanceIndex;
	(void)anchorLinkBackendId;
	(void)jointAnglesRad;
	return false;
}

void DocumentHost::reconcilePerLinkOuterBindFromScene(
	int instanceIndex,
	const QVector<double>& jointAnglesRad)
{
	// 由 DocumentPage 实现具体逻辑；基类默认空操作
	(void)instanceIndex;
	(void)jointAnglesRad;
}

} // namespace cloudsim::host