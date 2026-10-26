#ifndef ROBOTWIDGET_PLCSIGNALMAPTYPES_H
#define ROBOTWIDGET_PLCSIGNALMAPTYPES_H

/// @file PlcSignalMapTypes.h
/// @brief 外接 PLC 连接与 bool 映射表

#include "robotwidget_global.h"

#include <QJsonArray>
#include <QJsonObject>
#include <QString>
#include <QVector>

struct ROBOTWIDGET_EXPORT ExternalPlcLinkConfig
{
	/// "modbusTcp" | "abEip"（B 验收以 modbusTcp 为准）
	QString protocol = QStringLiteral("modbusTcp");
	QString gateway = QStringLiteral("127.0.0.1");
	int port = 502;
	/// Modbus 单元 ID；AB 为 path
	QString path = QStringLiteral("1");
	QString cpu;
	int timeoutMs = 3000;
	bool enabled = false;
};

enum class ROBOTWIDGET_EXPORT PlcMapDirection
{
	In = 0, // PLC → 仿真 DI
	Out		// 仿真 DO → PLC
};

struct ROBOTWIDGET_EXPORT PlcSignalMapEntry
{
	QString id;
	QString plcTag;
	QString ownerId;
	QString signal;
	PlcMapDirection direction = PlcMapDirection::In;
	/// 固定 "bool"
	QString type = QStringLiteral("bool");
};

ROBOTWIDGET_EXPORT QJsonObject externalPlcLinkToJson(const ExternalPlcLinkConfig& c);
ROBOTWIDGET_EXPORT ExternalPlcLinkConfig externalPlcLinkFromJson(const QJsonObject& o);
ROBOTWIDGET_EXPORT QJsonArray plcSignalMapToJson(const QVector<PlcSignalMapEntry>& map);
ROBOTWIDGET_EXPORT bool plcSignalMapFromJson(const QJsonArray& arr, QVector<PlcSignalMapEntry>* out, QString* err = nullptr);

#endif // ROBOTWIDGET_PLCSIGNALMAPTYPES_H
