#ifndef AIWIDGET_AIASSISTANTCOORDINATOR_H
#define AIWIDGET_AIASSISTANTCOORDINATOR_H

/// @file AiAssistantCoordinator.h
/// @brief 经 CloudSimAiSDK 编排解析与执行

#include "aiwidget_global.h"

#include "AiAgentTypes.h"
#include "AiConfigDto.h"
#include "AiInferenceTypes.h"
#include "AiParseTypes.h"
#include "AiTrajectoryFeatureTypes.h"

#include <QByteArray>
#include <QObject>
#include <QString>

class AiAssistantDockWidget;
class IAiAssistantHost;
class IPluginHostContext;

class AIWIDGET_EXPORT AiAssistantCoordinator : public QObject
{
	Q_OBJECT

public:
	explicit AiAssistantCoordinator(AiAssistantDockWidget* dock, QObject* parent = nullptr);

	void setAiHost(IAiAssistantHost* host);
	void setPluginHost(IPluginHostContext* host);

public slots:
	void onUserMessageSubmitted(const QString& text);
	void onCreateRecognitionConfirmed();
	void onConfirmTrajectoryFeaturesClicked();
	void onRetryTrajectoryFeaturesClicked();
	void onAgentConfirmAccepted(const QString& pendingId, const QByteArray& argsJsonUtf8);
	void onAgentConfirmRejected(const QString& pendingId);
	void onAgentConfirmSecondary(const QString& pendingId);

signals:
	void assistantFinished(const QString& reply, bool isError, const QString& parserVia);
	void parseFailed(const QString& message, const QString& parserVia);

private:
	enum class FeatureSessionState
	{
		Idle,
		AwaitingAxisClarify,
		PreviewCandidates,
		AwaitingSelection
	};

	bool needsViewportCapture(const QString& domainId, const QString& userText, const AiConfigDto& cfg) const;
	bool prepareTrajectoryFeatureRequest(const QString& userText, AiInferenceRequest& req, QString* err);
	void handleTrajectoryParseResult(const AiParseResult& result);
	void retryTrajectoryFeatureWithRules(const QString& userText);
	void scheduleTrajectoryCatalogRetry(const QString& userText);
	void resetFeatureSession();
	bool tryHandleFeatureFollowUp(const QString& text);
	/// 按上次/指定指令重新枚举并规则解析特征候选
	bool rerunTrajectoryFeatureRecognize(const QString& userText);
	bool shouldUseAgentRuntime(const QString& resolvedDomainId) const;
	void startAgentTurn(const QString& text, const QString& domainId);
	void beginUnifiedDomainConfirm(AiAgentConfirmKind kind, const QByteArray& payload, const QString& title,
								   const QString& confirmLabel, const QString& secondaryLabel, const QString& parserVia);
	void handleAgentEvent(const AiAgentEvent& ev);
	/// 模态离散对话框（经 Runtime TrajectoryCommit）；返回重选保留会话
	void openTrajectoryDiscretizeDialog(const QString& pendingId, const QByteArray& planIn, bool showRetry);
	void restoreTrajectoryCandidatePreview();
	bool tryHandleTrajectoryPlanRevise(const QString& text);

	AiAssistantDockWidget* m_dock = nullptr;
	IAiAssistantHost* m_aiHost = nullptr;
	IPluginHostContext* m_pluginHost = nullptr;

	QByteArray m_pendingRecognitionJson;
	QString m_pendingRecognitionParserVia;

	FeatureSessionState m_featureSessionState = FeatureSessionState::Idle;
	QByteArray m_pendingFeaturePlanJson;
	QByteArray m_pendingCatalogSliceUtf8;
	QByteArray m_pendingCatalogFullUtf8;
	QString m_pendingFeatureParserVia;
	QString m_pendingWorkpieceBackendId;
	QString m_pendingWorkpieceStepPath;
	AiFeatureAxis m_pendingFeatureAxis = AiFeatureAxis::Ambiguous;
	QString m_pendingPipelineTemplate;
	QString m_lastTrajectoryUserText;
	bool m_trajCatalogRetryUsed = false;
};

#endif
