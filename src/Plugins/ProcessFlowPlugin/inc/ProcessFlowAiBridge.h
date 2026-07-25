#ifndef PROCESSFLOWPLUGIN_PROCESSFLOWAIBRIDGE_H
#define PROCESSFLOWPLUGIN_PROCESSFLOWAIBRIDGE_H

/// @file ProcessFlowAiBridge.h
/// @brief IProcessFlowAiBridge 插件实现

#include "IProcessFlowAiBridge.h"

class ProcessFlowPlugin;

class ProcessFlowAiBridge final : public IProcessFlowAiBridge
{
public:
	explicit ProcessFlowAiBridge(ProcessFlowPlugin* plugin);

	bool ensureEntered(QString* outError) override;
	bool applyFlowJson(const QJsonObject& flow, bool autoLayout, QString* outError) override;
	bool runSimSync(const QJsonObject& config, QJsonObject* outStats, QString* outError) override;
	bool compareSync(const QJsonObject& config, QJsonArray* outRows, QString* outError) override;
	QJsonObject exportFlowJson() const override;

private:
	ProcessFlowPlugin* m_plugin = nullptr;
};

#endif
