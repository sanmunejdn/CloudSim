#pragma once

#include "aiwidget_global.h"

#include <QObject>
#include <QString>

class AiAssistantDockWidget;
class IAiAssistantHost;

/// 经 CloudSimAiSDK 编排解析与执行
class AIWIDGET_EXPORT AiAssistantCoordinator : public QObject
{
	Q_OBJECT

public:
	explicit AiAssistantCoordinator(AiAssistantDockWidget* dock, QObject* parent = nullptr);

	void setAiHost(IAiAssistantHost* host);

public slots:
	void onUserMessageSubmitted(const QString& text);

signals:
	void assistantFinished(const QString& reply, bool isError, const QString& parserVia);
	void parseFailed(const QString& message, const QString& parserVia);

private:
	AiAssistantDockWidget* m_dock = nullptr;
	IAiAssistantHost* m_aiHost = nullptr;
};
