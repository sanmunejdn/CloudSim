#ifndef ROBOTSCENE_LATCHEDIOSINK_H
#define ROBOTSCENE_LATCHEDIOSINK_H

/// @file LatchedIoSink.h
/// @brief 下一拍可见：读 DI 走锁存，写 DO/AO 转发底层 sink

#include "robot_scene_global.h"

#include "IRobotIoSink.h"

#include <QHash>
#include <QList>

class ROBOT_SCENE_API LatchedIoSink : public IRobotIoSink
{
public:
	explicit LatchedIoSink(IRobotIoSink* backend = nullptr);

	void setBackend(IRobotIoSink* backend) { m_backend = backend; }
	IRobotIoSink* backend() const { return m_backend; }

	/// 从底层 sink 采样端口列表到「下一拍」缓冲（调用方在 propagate 之后）
	void capturePortsToNext(const QList<int>& digitalPorts);
	void swapNextToCurrent();
	void clearLatches();

	bool getDigitalInput(int port, bool* outValue) const override;
	void setDigitalOutput(int port, bool value) override;
	void setAnalogOutput(int port, double value) override;
	int resolveNamedPort(const std::string& signalName, int fallbackPort) const override;

private:
	IRobotIoSink* m_backend = nullptr;
	QHash<int, bool> m_currentDi;
	QHash<int, bool> m_nextDi;
};

#endif // ROBOTSCENE_LATCHEDIOSINK_H
