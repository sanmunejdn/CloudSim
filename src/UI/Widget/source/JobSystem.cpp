/// @file JobSystem.cpp
/// @brief 后台作业线程池

#include "JobSystem.h"

#include "ProgressManager.h"

#include <QPointer>
#include <QThread>
#include <algorithm>
#include <exception>

namespace
{
constexpr int kShutdownWaitMs = 1500;

class JobRunnable : public QRunnable
{
public:
	JobRunnable(quint64 jobId, QPointer<ProgressManager> progress, JobCancellableWork work, JobCancelToken token,
				std::function<void(bool threw, const QString& throwMessage)> onFinished,
				std::function<void(quint64)> onDone)
		: m_jobId(jobId), m_progress(std::move(progress)), m_work(std::move(work)), m_token(std::move(token)),
		  m_onFinished(std::move(onFinished)), m_onDone(std::move(onDone))
	{
		setAutoDelete(true);
	}

	void run() override
	{
		bool threw = false;
		QString throwMsg;
		const bool canceledUpfront = m_token.canceled();
		const JobProgressSink sink = [this](double fraction, const QString& message)
		{
			if (m_progress)
			{
				m_progress->reportProgress(m_jobId, fraction, message);
			}
		};
		try
		{
			if (m_work && !canceledUpfront)
			{
				m_work(sink, m_token);
			}
		}
		catch (const std::exception& e)
		{
			threw = true;
			throwMsg = QString::fromUtf8(e.what());
		}
		catch (...)
		{
			threw = true;
			throwMsg = QStringLiteral("Unknown exception in background job.");
		}

		if (!threw && (canceledUpfront || m_token.canceled()))
		{
			throwMsg = QStringLiteral("Canceled");
		}

		if (m_onDone)
		{
			m_onDone(m_jobId);
		}

		// 关窗后 ProgressManager 可能已销毁，勿再回投 UI
		QPointer<ProgressManager> pm = m_progress;
		if (!pm)
		{
			return;
		}
		const quint64 id = m_jobId;
		auto onFinished = std::move(m_onFinished);
		pm->queueOnMainThread(
			[pm, id, threw, throwMsg, onFinished = std::move(onFinished)]()
			{
				if (onFinished)
				{
					onFinished(threw, throwMsg);
				}
				if (pm)
				{
					pm->reportJobFinished(id, !threw && throwMsg != QStringLiteral("Canceled"),
										  threw ? throwMsg : QString());
				}
			});
	}

private:
	quint64 m_jobId = 0;
	QPointer<ProgressManager> m_progress;
	JobCancellableWork m_work;
	JobCancelToken m_token;
	std::function<void(bool threw, const QString& throwMessage)> m_onFinished;
	std::function<void(quint64)> m_onDone;
};

} // namespace

JobSystem::JobSystem(QObject* parent) : QObject(parent), m_progress(new ProgressManager(this))
{
	m_pool = std::make_unique<QThreadPool>();
	m_pool->setMaxThreadCount(std::max(2, QThread::idealThreadCount()));
	m_pool->setExpiryTimeout(1000);
}

JobSystem::~JobSystem()
{
	shutdown();
}

void JobSystem::shutdown()
{
	if (m_shuttingDown.exchange(true))
	{
		return;
	}
	{
		QMutexLocker lock(&m_cancelMutex);
		for (auto it = m_cancelFlags.begin(); it != m_cancelFlags.end(); ++it)
		{
			if (it.value())
			{
				it.value()->store(true, std::memory_order_release);
			}
		}
	}
	if (!m_pool)
	{
		return;
	}

	m_pool->clear();
	if (m_pool->waitForDone(kShutdownWaitMs))
	{
		return;
	}

	// ~QThreadPool 会无限 waitForDone；超时则弃池，进程退出由 OS 回收线程
	(void)m_pool.release();
}

quint64 JobSystem::enqueue(const QString& title, std::function<void(const JobProgressSink&)> work,
						   std::function<void(bool threw, const QString& throwMessage)> onFinished)
{
	return enqueueCancellable(
		title,
		[work = std::move(work)](const JobProgressSink& sink, const JobCancelToken&)
		{
			if (work)
			{
				work(sink);
			}
		},
		std::move(onFinished));
}

quint64 JobSystem::enqueueCancellable(const QString& title, JobCancellableWork work,
									  std::function<void(bool threw, const QString& throwMessage)> onFinished)
{
	if (m_shuttingDown.load() || !m_pool)
	{
		if (onFinished)
		{
			onFinished(true, QStringLiteral("JobSystem not available"));
		}
		return 0;
	}
	const quint64 id = ++m_nextJobId;
	auto flag = std::make_shared<std::atomic<bool>>(false);
	{
		QMutexLocker lock(&m_cancelMutex);
		m_cancelFlags.insert(id, flag);
	}
	if (m_progress)
	{
		m_progress->reportJobStarted(id, title);
	}
	JobCancelToken token(flag);
	const QPointer<JobSystem> self(this);
	auto* runnable = new JobRunnable(
		id, m_progress, std::move(work), std::move(token), std::move(onFinished),
		[self](quint64 doneId)
		{
			if (!self)
			{
				return;
			}
			QMutexLocker lock(&self->m_cancelMutex);
			self->m_cancelFlags.remove(doneId);
		});
	m_pool->start(runnable);
	return id;
}

bool JobSystem::cancel(quint64 jobId)
{
	if (jobId == 0)
	{
		return false;
	}
	QMutexLocker lock(&m_cancelMutex);
	const auto it = m_cancelFlags.find(jobId);
	if (it == m_cancelFlags.end() || !it.value())
	{
		return false;
	}
	it.value()->store(true, std::memory_order_release);
	return true;
}
