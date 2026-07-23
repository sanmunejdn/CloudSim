#ifndef CLOUDSIMAISDK_AIAGENTTYPES_H
#define CLOUDSIMAISDK_AIAGENTTYPES_H

/// @file AiAgentTypes.h
/// @brief Agent 事件、ToolResult、域确认载荷

#include "cloudsim_ai_sdk_global.h"

#include <QByteArray>
#include <QString>
#include <QStringList>
#include <QVector>
#include <functional>

enum class AiAgentState
{
	Idle,
	Proposing,
	AwaitingConfirm,
	Executing,
	Done
};

enum class AiAgentEventKind
{
	NeedConfirm,
	StepDone,
	Finished,
	Error,
	Secondary // 面板次要按钮（如轨迹「重新识别」）
};

enum class AiAgentConfirmKind
{
	CatalogTool,
	RecognizeCreate,
	TrajectoryCommit
};

struct AiToolResult
{
	bool handled = false; // false=非本 Dispatch 认识的 api
	bool ok = false;
	QString summary;
	QString error;
	QStringList newBackendIds;
};

struct AiAgentEvent
{
	AiAgentEventKind kind = AiAgentEventKind::Finished;
	QString pendingId;
	QString toolId;
	QString title;
	QString risk;
	QByteArray argsSchemaJson;
	QByteArray proposedArgsJson;
	QByteArray sceneSnapshotJson;
	QString message;
	bool isError = false;
	int stepIndex = 0;
	QString confirmLabel;
	QString secondaryLabel;
	AiAgentConfirmKind confirmKind = AiAgentConfirmKind::CatalogTool;
};

struct AiDomainConfirmRequest
{
	AiAgentConfirmKind kind = AiAgentConfirmKind::RecognizeCreate;
	QByteArray payloadUtf8;
	QString title;
	QString risk = QStringLiteral("medium");
	QString confirmLabel;
	QString secondaryLabel;
	QString parserVia;
};

/// Agent 有序步骤（规则/LLM 规划产出）
struct AiAgentPlanStep
{
	QString apiId;
	QByteArray argsJson;
	QString rationale;
};

struct AiAgentPlan
{
	QVector<AiAgentPlanStep> steps;
	QString summary;
};

using AiAgentEventFn = std::function<void(const AiAgentEvent&)>;

#endif
