#ifndef CLOUDSIMPLUGINHOST_AIAGENTRUNTIME_H
#define CLOUDSIMPLUGINHOST_AIAGENTRUNTIME_H

/// @file AiAgentRuntime.h
/// @brief Agent 状态机：规划 → 确认 → 执行 → 观测回灌

#include "AiAgentTypes.h"
#include "AiConfigDto.h"
#include "AiInferenceTypes.h"

#include <QByteArray>
#include <QString>
#include <QStringList>
#include <memory>
#include <optional>

#include <json.hpp>

class AiAssistantHostImpl;
class PluginHostContext;

class AiAgentRuntime
{
public:
	explicit AiAgentRuntime(PluginHostContext* host, AiAssistantHostImpl* assistant);

	AiAgentState state() const { return m_state; }

	void runTurnAsync(const AiInferenceRequest& request, const AiConfigDto& config,
					  const AiInferenceProgressFn& progress, const AiAgentEventFn& onEvent);

	void beginDomainConfirm(const AiDomainConfirmRequest& request, const AiAgentEventFn& onEvent);

	void submitConfirm(const QString& pendingId, const QByteArray& argsJsonUtf8);
	void secondaryConfirm(const QString& pendingId);
	void cancelConfirm(const QString& pendingId);
	void cancelTurn();

private:
	struct Pending
	{
		QString id;
		QString toolId;
		QString risk;
		QByteArray schemaJson;
		QByteArray proposedArgs;
		QByteArray domainPayload;
		AiAgentConfirmKind confirmKind = AiAgentConfirmKind::CatalogTool;
		QString confirmLabel;
		QString secondaryLabel;
		QString title;
		int stepIndex = 0;
		QString lastToolCallId;
	};

	void setState(AiAgentState s);
	void schedulePropose(bool afterToolObservation);
	void continueAfterConfirm(const QByteArray& argsJson);
	void finishOk(const QString& message, const QString& toolId = QString(),
				  AiAgentConfirmKind kind = AiAgentConfirmKind::CatalogTool);
	void finishErr(const QString& message);
	bool proposeNextTool(QString* toolId, QByteArray* argsJson, QString* via, QString* toolCallId);
	bool tryBuildInitialPlan(QString* via);
	bool tryReplanAfterFailure(const QString& failureObservation);
	void emitPlanSummaryIfNeeded();
	QByteArray apiEntryJson(const QString& toolId) const;
	QByteArray mergePrefs(const QString& toolId, const QByteArray& proposed) const;
	void emitNeedConfirm();
	bool userWantsMultiStep() const;
	bool hasRemainingKeywordMatch() const;
	bool hasPlanRemaining() const;
	QString formatObservation(const QString& toolId, const QByteArray& argsJson, const AiToolResult& result) const;
	int objectCountInSnapshot(const QByteArray& snap) const;
	QString currentDocumentId() const;
	void trace(const QString& state, const QString& toolId, bool ok, const QByteArray& detail);
	AiToolResult executeCatalogTool(const QString& toolId, const nlohmann::json& args);
	const AiDomainModelConfig* findDomainConfig(const QString& domainId) const;

	PluginHostContext* m_host = nullptr;
	AiAssistantHostImpl* m_assistant = nullptr;
	AiAgentEventFn m_onEvent;
	AiInferenceProgressFn m_progress;
	AiConfigDto m_config;
	AiInferenceRequest m_request;
	QByteArray m_catalog;
	QByteArray m_snapshot;
	nlohmann::json m_llmMessages = nlohmann::json::array();
	int m_objectsBeforeStep = 0;
	QStringList m_doneTools;
	std::optional<AiAgentPlan> m_plan;
	int m_planIndex = 0;
	bool m_planSummaryEmitted = false;
	bool m_didReplan = false;
	AiAgentState m_state = AiAgentState::Idle;
	int m_step = 0;
	int m_generation = 0;
	int m_maxSteps = 8;
	int m_planMaxSteps = 8;
	bool m_autoLowRisk = true;
	bool m_enableTrace = true;
	bool m_enablePlan = true;
	bool m_replanOnFailure = true;
	bool m_usedLlm = false;
	std::unique_ptr<Pending> m_pending;
};

#endif
