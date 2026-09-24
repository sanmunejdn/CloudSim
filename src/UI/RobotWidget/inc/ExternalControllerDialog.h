#ifndef ROBOTWIDGET_EXTERNALCONTROLLERDIALOG_H
#define ROBOTWIDGET_EXTERNALCONTROLLERDIALOG_H

/// @file ExternalControllerDialog.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 外置控制器设置：语言、源码预览、监听与运行

#include "robotwidget_global.h"

#include <QDialog>
#include <QPlainTextEdit>
#include <QProcess>

class QCheckBox;
class QComboBox;
class QLabel;
class QPushButton;
class QResizeEvent;
class QPaintEvent;
class QWidget;

/// 源码区：左侧行号栏（不改文本内容，避免污染运行脚本）
class ROBOTWIDGET_EXPORT ControllerSourceEdit : public QPlainTextEdit
{
	Q_OBJECT

public:
	explicit ControllerSourceEdit(QWidget* parent = nullptr);

	int lineNumberAreaWidth() const;
	void lineNumberAreaPaintEvent(QPaintEvent* event);

protected:
	void resizeEvent(QResizeEvent* event) override;

private slots:
	void updateLineNumberAreaWidth(int newBlockCount);
	void updateLineNumberArea(const QRect& rect, int dy);

private:
	QWidget* m_lineNumberArea = nullptr;
};

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
	/// 文档内机器人实例标签；Run 时注入 CLOUDSIM_ROBOT_INDEX / 端口
	void setRobotInstanceOptions(const QStringList& labels, int selectIndex = 0);

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
	void onProcessReadyRead();

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
	void applyControllerEnvironment();
	void appendProcessLog(const QString& text);
	int selectedRobotInstanceIndex() const;
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
	QComboBox* m_robotInstanceCombo = nullptr;
	QLabel* m_portLabel = nullptr;
	QCheckBox* m_listenCheck = nullptr;
	QLabel* m_statusLabel = nullptr;
	ControllerSourceEdit* m_sourceEdit = nullptr;
	QPlainTextEdit* m_logEdit = nullptr;
	QPushButton* m_reloadBtn = nullptr;
	QPushButton* m_runBtn = nullptr;
	QPushButton* m_stopBtn = nullptr;
	QPushButton* m_openControllersBtn = nullptr;
	QPushButton* m_closeBtn = nullptr;
	QProcess* m_process = nullptr;
};

#endif // ROBOTWIDGET_EXTERNALCONTROLLERDIALOG_H
