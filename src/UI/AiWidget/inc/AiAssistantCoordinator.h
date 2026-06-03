#pragma once

#include "aiwidget_global.h"

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

/// 经 CloudSimAiSDK 编排解析与执行
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
