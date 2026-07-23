#ifndef AIWIDGET_AIASSISTANTDOCKWIDGET_H
#define AIWIDGET_AIASSISTANTDOCKWIDGET_H

/// @file AiAssistantDockWidget.h
/// @brief AI 助手面板：对话 + Agent 确认表单

#include "aiwidget_global.h"

#include <QByteArray>
#include <QString>
#include <QWidget>

class AiConfirmPanel;
class QComboBox;
class IAiAssistantHost;
class QLabel;
class QTextBrowser;
class QLineEdit;
class QPushButton;

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

	void showAgentConfirmPanel(const QString& pendingId, const QString& title, const QString& risk,
							   const QByteArray& argsSchemaJson, const QByteArray& proposedArgsJson,
							   const QByteArray& sceneSnapshotJson, const QString& confirmLabel = QString(),
							   const QString& secondaryLabel = QString());
	void hideAgentConfirmPanel();

signals:
	void messageSubmitted(const QString& text);
	void agentConfirmAccepted(const QString& pendingId, const QByteArray& argsJsonUtf8);
	void agentConfirmRejected(const QString& pendingId);
	void agentConfirmSecondary(const QString& pendingId);

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
	AiConfirmPanel* m_confirmPanel = nullptr;
	QLineEdit* m_input = nullptr;
	QPushButton* m_settingsBtn = nullptr;
	QPushButton* m_sendBtn = nullptr;
};

#endif
