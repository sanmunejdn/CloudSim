#ifndef ROBOTSCENE_IIOSIGNALNETWORK_H
#define ROBOTSCENE_IIOSIGNALNETWORK_H

/// @file IIoSignalNetwork.h
/// @brief Kernel IO 相位依赖的网络抽象（桌面/Host 适配）

#include "robot_scene_global.h"

#include "IRobotIoSink.h"

#include <QString>
#include <QStringList>

class ROBOT_SCENE_API IIoSignalNetwork
{
public:
	virtual ~IIoSignalNetwork() = default;

	virtual QStringList ownerIds() const = 0;
	virtual IRobotIoSink* sink(const QString& ownerId) = 0;
	virtual void propagateFrom(const QString& ownerId) = 0;
	virtual void propagateAll() = 0;
};

#endif // ROBOTSCENE_IIOSIGNALNETWORK_H
