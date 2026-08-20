#ifndef PROCESSFLOWPLUGIN_SIM_ISTATIONEXECUTOR_H
#define PROCESSFLOWPLUGIN_SIM_ISTATIONEXECUTOR_H

/// @file IStationExecutor.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 工位执行：Null=纯 DES；Preview=记录绑定仍按节拍推进

#include <QString>
#include <QVector>

struct StationBinding
{
	QString backendId;
	QString programId;
};

struct StationPreviewEvent
{
	int nodeId = -1;
	int entityId = -1;
	double cycleTimeSec = 0.0;
	StationBinding binding;
};

class IStationExecutor
{
public:
	virtual ~IStationExecutor() = default;
	virtual double beginProcess(int nodeId, int entityId, double cycleTimeSec,
								const StationBinding& binding) = 0;
};

class NullStationExecutor final : public IStationExecutor
{
public:
	double beginProcess(int /*nodeId*/, int /*entityId*/, double cycleTimeSec,
						const StationBinding& /*binding*/) override
	{
		return cycleTimeSec;
	}
};

/// DES 仍用 cycleTime；侧写绑定供后续 Host Preview 接线
class PreviewStationExecutor final : public IStationExecutor
{
public:
	double beginProcess(int nodeId, int entityId, double cycleTimeSec, const StationBinding& binding) override
	{
		StationPreviewEvent ev;
		ev.nodeId = nodeId;
		ev.entityId = entityId;
		ev.cycleTimeSec = cycleTimeSec;
		ev.binding = binding;
		m_events.append(ev);
		return cycleTimeSec;
	}

	const QVector<StationPreviewEvent>& events() const { return m_events; }

private:
	QVector<StationPreviewEvent> m_events;
};

#endif
