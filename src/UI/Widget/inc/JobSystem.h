#ifndef WIDGET_JOBSYSTEM_H
#define WIDGET_JOBSYSTEM_H

/// @file JobSystem.h
/// @brief 后台任务进度 sink（fraction∈[0,1]，message 可空）

#include "widget_global.h"

#include <QObject>
#include <QString>
#include <QThreadPool>
#include <atomic>
#include <functional>
#include <memory>

class ProgressManager;

/// 后台任务进度 sink（fraction∈[0,1]，message 可空）
using JobProgressSink = std::function<void(double fraction, const QString& message)>;

/// 线程池提交 CPU 重活；完成与进度通知 marshal 到 UI 线程
class WIDGET_EXPORT JobSystem : public QObject
{
public:
	explicit JobSystem(QObject* parent = nullptr);
	~JobSystem() override;

	ProgressManager* progressManager() const { return m_progress; }

	/// UI 线程调用；work 在线程池执行，onFinished 在 work 返回后于 UI 线程执行
	/// work 抛异常时 onFinished 仍执行，threw 为 true 且 throwMessage 有值
	void enqueue(const QString& title, std::function<void(const JobProgressSink&)> work,
				 std::function<void(bool threw, const QString& throwMessage)> onFinished);

	/// 拒收新任务并清空排队；限时等待在跑任务，超时弃池以免关窗卡死
	void shutdown();
	bool isShutdown() const { return m_shuttingDown.load(); }

private:
	ProgressManager* m_progress = nullptr;
	std::unique_ptr<QThreadPool> m_pool;
	std::atomic<quint64> m_nextJobId{0};
	std::atomic<bool> m_shuttingDown{false};
};

#endif // WIDGET_JOBSYSTEM_H
