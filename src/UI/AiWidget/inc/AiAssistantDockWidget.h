#ifndef AIWIDGET_AIASSISTANTDOCKWIDGET_H
#define AIWIDGET_AIASSISTANTDOCKWIDGET_H

/// @file AiAssistantDockWidget.h
/// @brief AI 助手面板：对话与自然语言建网格（规则解析）

#include "aiwidget_global.h"

#include <QString>
#include <QWidget>

class QComboBox;
class IAiAssistantHost;
class QLabel;
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
	void setAiHost(IAiAssistantHost* host);

	QString selectedDomainId() const;
	void showRecognitionResult(const QByteArray& jsonUtf8, const QString& parserVia);
	void hideCreateFromRecognitionButton();

	void showTrajectoryFeatureResult(const QByteArray& planJsonUtf8, const QByteArray& catalogSliceUtf8,
									 const QString& parserVia);
	void hideTrajectoryFeatureConfirmButtons();

signals:
	void messageSubmitted(const QString& text);
	void createFromRecognitionClicked();
	void confirmTrajectoryFeaturesClicked();
	void retryTrajectoryFeaturesClicked();

private slots:
	void onSendClicked();
	void onSettingsClicked();
	void onDomainChanged(int index);

private:
	static QString prefixWithParser(const QString& parserVia, const QString& text);

	bool m_useChinese = true;
	IAiAssistantHost* m_aiHost = nullptr;
	QComboBox* m_domainCombo = nullptr;
	QLabel* m_viewportHint = nullptr;
	QTextBrowser* m_history = nullptr;
	QLineEdit* m_input = nullptr;
	QPushButton* m_settingsBtn = nullptr;
	QPushButton* m_sendBtn = nullptr;
	QPushButton* m_createFromRecognitionBtn = nullptr;
	QPushButton* m_confirmTrajectoryBtn = nullptr;
	QPushButton* m_retryTrajectoryBtn = nullptr;
};

#endif // AIWIDGET_AIASSISTANTDOCKWIDGET_H
