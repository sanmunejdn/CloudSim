#ifndef PROCESSFLOWPLUGIN_PROCESSFLOWNODEPROPS_H
#define PROCESSFLOWPLUGIN_PROCESSFLOWNODEPROPS_H

/// @file ProcessFlowNodeProps.h
/// @brief 工艺流程节点类型与扩展属性

#include <QJsonObject>
#include <QString>
#include <QStringList>

struct ProcessFlowNodeProps
{
	QString kind = QStringLiteral("station");
	double cycleTimeSec = 0.0;
	double inventoryQty = 0.0;
	double capacityQty = 0.0;
	double setupTimeSec = 0.0;
	double priority = 0.0;
	double batchSize = 1.0;
	double scrapRate = 0.0;
	double mtbfSec = 0.0;
	double mttrSec = 0.0;
	double requiredInputs = 2.0;
	QString bindingBackendId;
	QString bindingProgramId;

	QJsonObject toJson() const;
	static ProcessFlowNodeProps fromJson(const QJsonObject& obj);
	static ProcessFlowNodeProps defaultsForKind(const QString& kind);
	static QString inferKindFromTitle(const QString& title, const QString& subtitle);
	static QString displayNameZh(const QString& kind);
	static QString displayNameEn(const QString& kind);
	static QStringList allKinds();
	static bool isMachineKind(const QString& kind);
	static bool isBufferKind(const QString& kind);
};

/// 节点库拖放 MIME
inline const char* processFlowNodeMimeType()
{
	return "application/x-cloudsim-processflow-node";
}

#endif
