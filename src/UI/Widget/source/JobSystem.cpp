/// @file JobSystem.cpp
/// @brief JobSystem 实现

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
	JobRunnable(quint64 jobId, QPointer<ProgressManager> progress, std::function<void(const JobProgressSink&)> work,
				std::function<void(bool threw, const QString& throwMessage)> onFinished)
		: m_jobId(jobId), m_progress(std::move(progress)), m_work(std::move(work)), m_onFinished(std::move(onFinished))
	{
		setAutoDelete(true);
	}

	void run() override
	{
		bool threw = false;
		QString throwMsg;
		const JobProgressSink sink = [this](double fraction, const QString& message)
		{
			if (m_progress)
			{
				m_progress->reportProgress(m_jobId, fraction, message);
			}
		};
		try
		{
			if (m_work)
			{
				m_work(sink);
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

		// 关窗后 ProgressManager 可能已销毁，勿再回投 UI
		QPointer<ProgressManager> pm = m_progress;
		if (!pm)
		{
			return;
		}
		const quint64 id = m_jobId;
		auto onFinished = std::move(m_onFinished);
		pm->queueOnMainThread(
			[pm, id, threw, throwMsg, onFinished = std::move(onFinished)]() mutable
			{
				if (onFinished)
				{
					onFinished(threw, throwMsg);
				}
				if (pm)
				{
					pm->reportJobFinished(id, !threw, threw ? throwMsg : QString());
				}
			});
	}

private:
	quint64 m_jobId = 0;
	QPointer<ProgressManager> m_progress;
	std::function<void(const JobProgressSink&)> m_work;
	std::function<void(bool threw, const QString& throwMessage)> m_onFinished;
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

void JobSystem::enqueue(const QString& title, std::function<void(const JobProgressSink&)> work,
						std::function<void(bool threw, const QString& throwMessage)> onFinished)
{
	if (m_shuttingDown.load() || !m_pool)
	{
		return;
	}
	const quint64 id = ++m_nextJobId;
	if (m_progress)
	{
		m_progress->reportJobStarted(id, title);
	}
	auto* runnable = new JobRunnable(id, m_progress, std::move(work), std::move(onFinished));
	m_pool->start(runnable);
}
