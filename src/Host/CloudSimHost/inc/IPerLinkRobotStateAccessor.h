#ifndef CLOUDSIMHOST_IPERLINKROBOTSTATEACCESSOR_H
#define CLOUDSIMHOST_IPERLINKROBOTSTATEACCESSOR_H

/// @file IPerLinkRobotStateAccessor.h
/// @brief per-link 机器人状态快照（Mat4 版本，供 Host 实现类操作）

#include "cloudsim_host_global.h"

#include "CoreTypes.h"

#include <QHash>
#include <QString>
#include <QVector>

class BackendDataManager;
class IRobotBackendPoseSink;

namespace cloudsim::host
{
/// per-link 机器人状态快照（Mat4 版本，供 Host 实现类操作）
struct PerLinkRobotStateSnapshot
{
	int instanceIndex = -1;
	QString urdfAbsolutePath;
	QHash<QString, QString> linkNameToBackendId;
	QHash<QString, core::Mat4> fkMeshWorldT0;
	QHash<QString, core::Mat4> outerWorldAtBindByBackendId;
	core::Mat4 basePlacementWorld = core::PlanContextDto::identityMat4();
	bool meshVerticesInLinkFrame = false;
};

/// per-link FK 计算结果（供 DocumentPage 应用回内部状态）
struct PerLinkRobotFkResult
{
	core::Mat4 computedPlacementWorld{};
	QHash<QString, core::Mat4> updatedOuterWorlds; // linkBackendId -> new outer matrix
	bool success = false;
	QString errorMessage;
};

/// per-link 机器人状态访问器接口（由 DocumentPage 实现，Host 实现类通过该接口访问状态）
class CLOUDSIM_HOST_EXPORT IPerLinkRobotStateAccessor
{
public:
	virtual ~IPerLinkRobotStateAccessor() = default;

	virtual PerLinkRobotStateSnapshot extractPerLinkStateSnapshot(int instanceIndex) const = 0;
	virtual void applyPerLinkFkResult(const PerLinkRobotFkResult& result) = 0;

	virtual IRobotBackendPoseSink* urdfImportScenePoseSink() = 0;
	virtual BackendDataManager& backend() = 0;
};

} // namespace cloudsim::host

#endif // CLOUDSIMHOST_IPERLINKROBOTSTATEACCESSOR_H
