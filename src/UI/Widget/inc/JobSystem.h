#ifndef WIDGET_JOBSYSTEM_H
#define WIDGET_JOBSYSTEM_H

/// @file JobSystem.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 后台任务进度 sink（fraction∈[0,1]，message 可空）

#include "widget_global.h"

#include <QHash>
#include <QMutex>
#include <QObject>
#include <QString>
#include <QThreadPool>
#include <atomic>
#include <functional>
#include <memory>

class ProgressManager;

/// 后台任务进度 sink（fraction∈[0,1]，message 可空）
using JobProgressSink = std::function<void(double fraction, const QString& message)>;

/// 协作取消：长循环须主动查 canceled()，不会强杀线程
class WIDGET_EXPORT JobCancelToken
{
public:
	JobCancelToken() = default;
	explicit JobCancelToken(std::shared_ptr<const std::atomic<bool>> flag) : m_flag(std::move(flag)) {}

	bool canceled() const
	{
		return m_flag && m_flag->load(std::memory_order_acquire);
	}

private:
	std::shared_ptr<const std::atomic<bool>> m_flag;
};

using JobCancellableWork = std::function<void(const JobProgressSink&, const JobCancelToken&)>;

/// 线程池提交 CPU 重活；完成与进度通知 marshal 到 UI 线程
class WIDGET_EXPORT JobSystem : public QObject
{
public:
	explicit JobSystem(QObject* parent = nullptr);
	~JobSystem() override;

	ProgressManager* progressManager() const { return m_progress; }

	/// UI 线程调用；work 在线程池执行，onFinished 在 work 返回后于 UI 线程执行
	/// work 抛异常时 onFinished 仍执行，threw 为 true 且 throwMessage 有值
	quint64 enqueue(const QString& title, std::function<void(const JobProgressSink&)> work,
					std::function<void(bool threw, const QString& throwMessage)> onFinished);

	quint64 enqueueCancellable(const QString& title, JobCancellableWork work,
							  std::function<void(bool threw, const QString& throwMessage)> onFinished);

	/// 协作取消；已跑完返回 false
	bool cancel(quint64 jobId);

	/// 拒收新任务并清空排队；限时等待在跑任务，超时弃池以免关窗卡死
	void shutdown();
	bool isShutdown() const { return m_shuttingDown.load(); }

private:
	ProgressManager* m_progress = nullptr;
	std::unique_ptr<QThreadPool> m_pool;
	std::atomic<quint64> m_nextJobId{0};
	std::atomic<bool> m_shuttingDown{false};
	mutable QMutex m_cancelMutex;
	QHash<quint64, std::shared_ptr<std::atomic<bool>>> m_cancelFlags;
};

#endif // WIDGET_JOBSYSTEM_H
