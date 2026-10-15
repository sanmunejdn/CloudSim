/// @file ProgressManager.cpp
/// @brief Progress 管理：跨线程信号 + 同步 Infra/Progress 总线

#include "ProgressManager.h"

#include "Progress.h"

#include <QMetaObject>

namespace
{
double clampJobFraction(double fraction)
{
	if (fraction < 0.0)
	{
		return fraction;
	}
	if (fraction > 1.5)
	{
		fraction *= 0.01;
	}
	if (fraction > 1.0)
	{
		return 1.0;
	}
	return fraction;
}
} // namespace

ProgressManager::ProgressManager(QObject* parent) : QObject(parent) {}

void ProgressManager::queueOnMainThread(std::function<void()> fn)
{
	if (!fn)
	{
		return;
	}
	QMetaObject::invokeMethod(this, std::move(fn), Qt::QueuedConnection);
}

void ProgressManager::reportJobStarted(quint64 jobId, const QString& title, bool cancellable)
{
	Progress::announceStarted(static_cast<std::uint64_t>(jobId), title.toStdString(), cancellable);
	QMetaObject::invokeMethod(
		this, [this, jobId, title, cancellable]() { emit jobStarted(jobId, title, cancellable); },
		Qt::QueuedConnection);
}

void ProgressManager::reportProgress(quint64 jobId, double fraction, const QString& message)
{
	const double f = clampJobFraction(fraction);
	Progress::report(static_cast<std::uint64_t>(jobId), f, message.toStdString());
	QMetaObject::invokeMethod(
		this, [this, jobId, f, message]() { emit jobProgress(jobId, f, message); }, Qt::QueuedConnection);
}

void ProgressManager::reportJobFinished(quint64 jobId, bool success, const QString& errorMessage)
{
	Progress::end(static_cast<std::uint64_t>(jobId), success, errorMessage.toStdString());
	QMetaObject::invokeMethod(
		this, [this, jobId, success, errorMessage]() { emit jobFinished(jobId, success, errorMessage); },
		Qt::QueuedConnection);
}
