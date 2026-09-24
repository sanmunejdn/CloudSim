#ifndef ROBOTWIDGET_EXTERNALCONTROLLERDIALOG_H
#define ROBOTWIDGET_EXTERNALCONTROLLERDIALOG_H

/// @file ExternalControllerDialog.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 外置控制器设置：语言、源码预览、监听与运行

#include "robotwidget_global.h"

#include <QDialog>
#include <QProcess>

class QCheckBox;
class QComboBox;
class QLabel;
class QPlainTextEdit;
class QPushButton;

class ROBOTWIDGET_EXPORT ExternalControllerDialog : public QDialog
{
	Q_OBJECT

public:
	explicit ExternalControllerDialog(QWidget* parent = nullptr);
	~ExternalControllerDialog() override;

	void setUseChinese(bool chinese);
	void setListening(bool listening);
	void setClientConnected(bool connected);
	bool isListeningChecked() const;

signals:
	void listenToggled(bool enabled);

private slots:
	void onLanguageChanged(int index);
	void onScriptFileChanged(int index);
	void onListenCheckToggled(bool on);
	void onReloadClicked();
	void onRunClicked();
	void onStopClicked();
	void onOpenControllersClicked();
	void onProcessFinished(int exitCode, QProcess::ExitStatus status);

private:
	enum class Lang
	{
		Python = 0,
		Cpp = 1
	};

	void retranslate();
	void refreshSourceView();
	void updateStatusLabel();
	void updateRunStopEnabled();
	QString controllerPythonDir() const;
	QString controllersRootDir() const;
	QString pythonScriptPath(const QString& fileName) const;
	QString loadPythonSource(const QString& fileName) const;
	QString embeddedCppSample() const;
	Lang currentLang() const;

	bool m_useChinese = true;
	bool m_listening = false;
	bool m_clientConnected = false;

	QComboBox* m_langCombo = nullptr;
	QComboBox* m_scriptCombo = nullptr;
	QLabel* m_portLabel = nullptr;
	QCheckBox* m_listenCheck = nullptr;
	QLabel* m_statusLabel = nullptr;
	QPlainTextEdit* m_sourceEdit = nullptr;
	QPushButton* m_reloadBtn = nullptr;
	QPushButton* m_runBtn = nullptr;
	QPushButton* m_stopBtn = nullptr;
	QPushButton* m_openControllersBtn = nullptr;
	QPushButton* m_closeBtn = nullptr;
	QProcess* m_process = nullptr;
};

#endif // ROBOTWIDGET_EXTERNALCONTROLLERDIALOG_H
