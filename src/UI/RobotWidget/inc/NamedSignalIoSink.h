#ifndef ROBOTWIDGET_NAMEDSIGNALIOSINK_H
#define ROBOTWIDGET_NAMEDSIGNALIOSINK_H

/// @file NamedSignalIoSink.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 仿真命名 IO：分表 DI/DO/AI/AO，面板可强制 DI；实现 IRobotIoSink

#include "robotwidget_global.h"

#include "IRobotIoSink.h"
#include "NamedSignalTable.h"

#include <QHash>
#include <QObject>

/// 仿真命名 IO：分表 DI/DO/AI/AO，面板可强制 DI；实现 IRobotIoSink
class ROBOTWIDGET_EXPORT NamedSignalIoSink : public QObject, public IRobotIoSink
{
	Q_OBJECT

public:
	explicit NamedSignalIoSink(QObject* parent = nullptr);

	void setSignalTable(RobotIo::NamedSignalTable* table);
	RobotIo::NamedSignalTable* signalTable() const { return m_table; }

	void setIoSinkBackend(RobotIoSinkBackend backend);
	RobotIoSinkBackend ioSinkBackend() const { return m_backend; }

	/// 按定义表重置运行时值（保留已强制 DI 可选）
	void resetRuntimeFromTable(bool keepForcedDi = false);

	void setDigitalOutput(int port, bool value) override;
	void setAnalogOutput(int port, double value) override;
	bool getDigitalInput(int port, bool* outValue) const override;
	int resolveNamedPort(const std::string& signalName, int fallbackPort) const override;

	bool getDigitalOutput(int port, bool* outValue) const;
	bool getAnalogOutput(int port, double* outValue) const;
	bool getAnalogInput(int port, double* outValue) const;

	void setDigitalInput(int port, bool value);
	void setDigitalInputForced(int port, bool value);
	void clearDigitalInputForced(int port);
	bool isDigitalInputForced(int port) const;
	void setAnalogInput(int port, double value);

signals:
	void ioValuesChanged();

private:
	RobotIo::NamedSignalTable* m_table = nullptr;
	RobotIoSinkBackend m_backend = RobotIoSinkBackend::Simulation;

	QHash<int, bool> m_di;
	QHash<int, bool> m_do;
	QHash<int, double> m_ai;
	QHash<int, double> m_ao;
	QHash<int, bool> m_diForced;
};

/// 兼容旧名：指向 NamedSignalIoSink
using SimulationLogIoSink = NamedSignalIoSink;

#endif // ROBOTWIDGET_NAMEDSIGNALIOSINK_H
