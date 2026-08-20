#ifndef AIWIDGET_AILLMSETTINGSDIALOG_H
#define AIWIDGET_AILLMSETTINGSDIALOG_H

/// @file AiLlmSettingsDialog.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 编辑 ai_config.json（经 IAiAssistantHost）

#include "aiwidget_global.h"

#include "AiConfigDto.h"

#include <QDialog>
#include <QString>

class IAiAssistantHost;
class QCheckBox;
class QDialogButtonBox;
class QDoubleSpinBox;
class QFormLayout;
class QGroupBox;
class QLabel;
class QLineEdit;
class QSpinBox;

/// 编辑 ai_config.json（经 IAiAssistantHost）
class AIWIDGET_EXPORT AiLlmSettingsDialog : public QDialog
{
	Q_OBJECT

public:
	explicit AiLlmSettingsDialog(QWidget* parent = nullptr);

	void setAiHost(IAiAssistantHost* host);
	void setUseChinese(bool chinese);

private slots:
	void onAccepted();

private:
	void buildUi();
	void loadFromFileOrDefaults();
	void applyLanguage();
	void updatePathLabel();
	void setConfig(const AiConfigDto& cfg);
	AiConfigDto config() const;

	bool m_useChinese = true;
	IAiAssistantHost* m_aiHost = nullptr;
	QLabel* m_pathLabel = nullptr;
	QLabel* m_hintLabel = nullptr;
	QGroupBox* m_connectionGroup = nullptr;
	QFormLayout* m_form = nullptr;
	QCheckBox* m_enabled = nullptr;
	QCheckBox* m_enableRules = nullptr;
	QLineEdit* m_baseUrl = nullptr;
	QLineEdit* m_apiKey = nullptr;
	QLineEdit* m_apiKeyEnv = nullptr;
	QLineEdit* m_model = nullptr;
	QSpinBox* m_timeoutMs = nullptr;
	QDoubleSpinBox* m_temperature = nullptr;
	QDialogButtonBox* m_buttons = nullptr;
};

#endif // AIWIDGET_AILLMSETTINGSDIALOG_H
