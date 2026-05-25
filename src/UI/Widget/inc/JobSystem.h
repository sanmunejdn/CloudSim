#pragma once

#include <atomic>
#include <functional>

#include <QObject>
#include <QString>
#include <QThreadPool>

#include "widget_global.h"

class ProgressManager;

/// 后台任务进度 sink（fraction∈[0,1]，message 可空）
using JobProgressSink = std::function<void(double fraction, const QString& message)>;

/// 线程池提交 CPU 重活；完成与进度通知 marshal 到 UI 线程
class WIDGET_EXPORT JobSystem : public QObject
{
public:
	explicit JobSystem(QObject* parent = nullptr);

	ProgressManager* progressManager() const { return m_progress; }

	/// UI 线程调用；work 在线程池执行，onFinished 在 work 返回后于 UI 线程执行
	/// work 抛异常时 onFinished 仍执行，threw 为 true 且 throwMessage 有值
	void enqueue(const QString& title, std::function<void(const JobProgressSink&)> work,
		std::function<void(bool threw, const QString& throwMessage)> onFinished);

private:
	ProgressManager* m_progress = nullptr;
	QThreadPool m_pool;
	std::atomic<quint64> m_nextJobId{ 0 };
};
