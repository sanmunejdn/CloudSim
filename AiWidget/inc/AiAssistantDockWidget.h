#pragma once

#include <QWidget>

#include "aiwidget_global.h"

class QTextBrowser;
class QLineEdit;
class QPushButton;

/// AI assistant panel: chat history and natural-language mesh creation (rule parser).
class AIWIDGET_EXPORT AiAssistantDockWidget : public QWidget
{
	Q_OBJECT

public:
	explicit AiAssistantDockWidget(QWidget* parent = nullptr);

	void appendUserMessage(const QString& text);
	void appendAssistantMessage(const QString& text);
	void appendSystemMessage(const QString& text);
	void setBusy(bool busy);

signals:
	void messageSubmitted(const QString& text);

private slots:
	void onSendClicked();
	void onSettingsClicked();

private:
	QTextBrowser* m_history = nullptr;
	QLineEdit* m_input = nullptr;
	QPushButton* m_settingsBtn = nullptr;
	QPushButton* m_sendBtn = nullptr;
};
