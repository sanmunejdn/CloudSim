/// @file ProgressManager.cpp
/// @brief Progress 管理

#include "ProgressManager.h"

#include <QMetaObject>

ProgressManager::ProgressManager(QObject* parent) : QObject(parent) {}

void ProgressManager::queueOnMainThread(std::function<void()> fn)
{
	if (!fn)
	{
		return;
	}
	QMetaObject::invokeMethod(this, std::move(fn), Qt::QueuedConnection);
}

void ProgressManager::reportJobStarted(quint64 jobId, const QString& title)
{
	QMetaObject::invokeMethod(
		this, [this, jobId, title]() { emit jobStarted(jobId, title); }, Qt::QueuedConnection);
}

void ProgressManager::reportProgress(quint64 jobId, double fraction, const QString& message)
{
	QMetaObject::invokeMethod(
		this, [this, jobId, fraction, message]() { emit jobProgress(jobId, fraction, message); }, Qt::QueuedConnection);
}

void ProgressManager::reportJobFinished(quint64 jobId, bool success, const QString& errorMessage)
{
	QMetaObject::invokeMethod(
		this, [this, jobId, success, errorMessage]() { emit jobFinished(jobId, success, errorMessage); },
		Qt::QueuedConnection);
}
