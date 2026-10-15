#ifndef WIDGET_PROGRESSDIALOGCONTROLLER_H
#define WIDGET_PROGRESSDIALOGCONTROLLER_H

/// @file ProgressDialogController.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 将 Progress / JobSystem 事件映射到 ProgressDialog

#include "widget_global.h"

#include "Progress.h"

#include <QHash>
#include <QObject>
#include <QPointer>
#include <QString>
#include <QVector>
#include <cstdint>

class JobSystem;
class MainWindow;
class ProgressDialog;
class RunInfoPage;

class WIDGET_EXPORT ProgressDialogController : public QObject
{
	Q_OBJECT

public:
	explicit ProgressDialogController(MainWindow* mainWindow, QObject* parent = nullptr);
	~ProgressDialogController() override;

	void attachJobSystem(JobSystem* jobs);
	void setRunInfoPage(RunInfoPage* page);
	void setChinese(bool useChinese);

	/// 关窗：清空状态并隐藏对话框，忽略后续排队事件
	void dismissAll();

	/// 供 Progress::setUiSink；任意线程可调，内部 Queued 到 UI
	void handleProgressEvent(const Progress::Event& ev);

private:
	struct JobState
	{
		QString title;
		QString message;
		double fraction = -1.0;
		bool cancellable = false;
	};

	void applyEventOnUi(const Progress::Event& ev);
	void refreshDialog();
	void hideDialogIfIdle();

	QPointer<MainWindow> m_mainWindow;
	QPointer<JobSystem> m_jobs;
	QPointer<RunInfoPage> m_runInfo;
	ProgressDialog* m_dialog = nullptr;
	QHash<quint64, JobState> m_jobsState;
	QVector<quint64> m_stack;
	bool m_useChinese = true;
	bool m_dismissed = false;
};

#endif // WIDGET_PROGRESSDIALOGCONTROLLER_H
