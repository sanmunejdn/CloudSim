#pragma once

#include "aiwidget_global.h"

#include <functional>

#include <QObject>
#include <QString>

class AiAssistantDockWidget;

/// Orchestrates rule/LLM parsing (AiBackend) and drives dock UI messages.
class AIWIDGET_EXPORT AiAssistantCoordinator : public QObject
{
	Q_OBJECT

public:
	using EnqueueBackgroundWork = std::function<void(
		const QString& title,
		std::function<void(const std::function<void(double, const QString&)>&)> work,
		std::function<void(bool threw, const QString& throwMessage)> onFinished)>;

	explicit AiAssistantCoordinator(AiAssistantDockWidget* dock, QObject* parent = nullptr);

	void setBackgroundEnqueue(EnqueueBackgroundWork enqueue);

public slots:
	void onUserMessageSubmitted(const QString& text);

signals:
	/// Parsed create_mesh JSON (UTF-8) ready for application layer (Widget/MainWindow).
	void createMeshCommandReady(const QByteArray& commandJsonUtf8, const QString& parserVia);
	void parseFailed(const QString& message, const QString& parserVia);

private:
	AiAssistantDockWidget* m_dock = nullptr;
	EnqueueBackgroundWork m_enqueue;
};
