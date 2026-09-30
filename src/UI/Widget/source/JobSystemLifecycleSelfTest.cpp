/// @file JobSystemLifecycleSelfTest.cpp
/// @brief JobSystem shutdown 契约自检（QCoreApplication，无 MainWindow）

#include "JobSystem.h"

#include <QCoreApplication>
#include <QElapsedTimer>
#include <QThread>
#include <atomic>

namespace
{
// 须大于 JobSystem.cpp 内 kShutdownWaitMs(1500)，逼出超时弃池路径
constexpr int kSlowJobMs = 3500;
constexpr int kShutdownBudgetMs = 5000;

void fail(std::vector<std::string>* failures, const char* msg)
{
	if (failures)
	{
		failures->push_back(msg);
	}
}
} // namespace

bool runJobSystemLifecycleSelfTest(std::vector<std::string>* failures)
{
	if (!QCoreApplication::instance())
	{
		fail(failures, "QCoreApplication required");
		return false;
	}

	bool ok = true;
	JobSystem js;

	std::atomic<bool> workEntered{false};
	const quint64 id = js.enqueue(
		QStringLiteral("JobSystemLifecycleSelfTest-slow"),
		[&workEntered](const JobProgressSink&)
		{
			workEntered.store(true, std::memory_order_release);
			QThread::msleep(kSlowJobMs);
		},
		[](bool, const QString&) {});
	if (id == 0)
	{
		fail(failures, "enqueue slow job returned 0");
		return false;
	}

	QElapsedTimer waitStart;
	waitStart.start();
	while (!workEntered.load(std::memory_order_acquire) && waitStart.elapsed() < 2000)
	{
		QThread::msleep(10);
		QCoreApplication::processEvents();
	}
	if (!workEntered.load(std::memory_order_acquire))
	{
		fail(failures, "slow job never started");
		js.shutdown();
		return false;
	}

	QElapsedTimer shutdownTimer;
	shutdownTimer.start();
	js.shutdown();
	const qint64 shutdownMs = shutdownTimer.elapsed();

	if (!js.isShutdown())
	{
		fail(failures, "isShutdown false after shutdown()");
		ok = false;
	}
	if (shutdownMs > kShutdownBudgetMs)
	{
		fail(failures, "shutdown exceeded budget (expected timeout abandon pool)");
		ok = false;
	}

	bool postRejected = false;
	QString postMsg;
	const quint64 postId = js.enqueue(
		QStringLiteral("after-shutdown"), [](const JobProgressSink&) {},
		[&postRejected, &postMsg](bool threw, const QString& msg)
		{
			postRejected = threw;
			postMsg = msg;
		});
	if (postId != 0)
	{
		fail(failures, "enqueue after shutdown should return 0");
		ok = false;
	}
	if (!postRejected)
	{
		fail(failures, "enqueue after shutdown should invoke onFinished(threw=true)");
		ok = false;
	}

	return ok;
}
