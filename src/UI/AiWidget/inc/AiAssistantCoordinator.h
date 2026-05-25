#pragma once

#include "aiwidget_global.h"

#include <functional>

#include <QObject>
#include <QString>

class AiAssistantDockWidget;

/// 编排规则/LLM 解析（AiBackend）并驱动 Dock 消息
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
	/// create_mesh JSON（UTF-8），供 Widget/MainWindow 应用
	void createMeshCommandReady(const QByteArray& commandJsonUtf8, const QString& parserVia);
	void parseFailed(const QString& message, const QString& parserVia);

private:
	AiAssistantDockWidget* m_dock = nullptr;
	EnqueueBackgroundWork m_enqueue;
};
