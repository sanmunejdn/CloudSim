#ifndef ROBOTSCENE_IOPHASE_H
#define ROBOTSCENE_IOPHASE_H

/// @file IoPhase.h
/// @brief Kernel IO 相位：propagate + 下一拍 DI 锁存

#include "robot_scene_global.h"

#include "LatchedIoSink.h"

#include <QHash>
#include <QString>
#include <memory>

class IIoSignalNetwork;

class ROBOT_SCENE_API IoPhase
{
public:
	void setNetwork(IIoSignalNetwork* network) { m_network = network; }

	LatchedIoSink* ensureLatchedSink(const QString& ownerId);
	LatchedIoSink* latchedSink(const QString& ownerId) const;
	void clear();

	/// propagateAll 后按 owner 采样常用端口到 next，再 swap
	void latchAfterPropagate(const QHash<QString, QList<int>>& portsByOwner);
	void swapAll();

private:
	IIoSignalNetwork* m_network = nullptr;
	QHash<QString, std::shared_ptr<LatchedIoSink>> m_latched;
};

#endif // ROBOTSCENE_IOPHASE_H
