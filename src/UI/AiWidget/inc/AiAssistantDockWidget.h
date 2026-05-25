#pragma once

#include <QString>
#include <QWidget>

#include "aiwidget_global.h"

class QTextBrowser;
class QLineEdit;
class QPushButton;

/// AI 助手面板：对话与自然语言建网格（规则解析）
class AIWIDGET_EXPORT AiAssistantDockWidget : public QWidget
{
	Q_OBJECT

public:
	explicit AiAssistantDockWidget(QWidget* parent = nullptr);

	void appendUserMessage(const QString& text);
	void appendAssistantMessage(const QString& text);
	void appendSystemMessage(const QString& text);
	void setBusy(bool busy);
	void setUseChinese(bool chinese);

signals:
	void messageSubmitted(const QString& text);

private slots:
	void onSendClicked();
	void onSettingsClicked();

private:
	bool m_useChinese = true;
	QTextBrowser* m_history = nullptr;
	QLineEdit* m_input = nullptr;
	QPushButton* m_settingsBtn = nullptr;
	QPushButton* m_sendBtn = nullptr;
};
