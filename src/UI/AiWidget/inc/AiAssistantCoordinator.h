#pragma once

#include "aiwidget_global.h"

#include "AiConfigDto.h"

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

signals:
	void assistantFinished(const QString& reply, bool isError, const QString& parserVia);
	void parseFailed(const QString& message, const QString& parserVia);

private:
	bool needsViewportCapture(const QString& domainId, const QString& userText, const AiConfigDto& cfg) const;

	AiAssistantDockWidget* m_dock = nullptr;
	IAiAssistantHost* m_aiHost = nullptr;
	IPluginHostContext* m_pluginHost = nullptr;
	QByteArray m_pendingRecognitionJson;
	QString m_pendingRecognitionParserVia;
};
