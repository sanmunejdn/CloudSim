#ifndef WIDGET_PROGRESSMANAGER_H
#define WIDGET_PROGRESSMANAGER_H

/// @file ProgressManager.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 跨线程进度桥：工作线程上报，信号在本对象线程（UI）发出

#include "widget_global.h"

#include <QObject>
#include <QString>
#include <functional>

/// 跨线程进度桥：工作线程上报，信号在本对象线程（UI）发出
class WIDGET_EXPORT ProgressManager : public QObject
{
	Q_OBJECT

public:
	explicit ProgressManager(QObject* parent = nullptr);

	void queueOnMainThread(std::function<void()> fn);

	void reportJobStarted(quint64 jobId, const QString& title);
	void reportProgress(quint64 jobId, double fraction, const QString& message);
	void reportJobFinished(quint64 jobId, bool success, const QString& errorMessage);

signals:
	void jobStarted(quint64 jobId, QString title);
	void jobProgress(quint64 jobId, double fraction, QString message);
	/// success 表示 worker 未抛 C++ 异常；业务失败可能在完成回调里处理且 success 仍为 true
	void jobFinished(quint64 jobId, bool success, QString errorMessage);
};

#endif // WIDGET_PROGRESSMANAGER_H
