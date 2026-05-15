#pragma once

#include <atomic>
#include <functional>

#include <QObject>
#include <QString>
#include <QThreadPool>

#include "widget_global.h"

class ProgressManager;

/// Progress sink passed to background work (fraction in [0,1]; message may be empty).
using JobProgressSink = std::function<void(double fraction, const QString& message)>;

/// Submits CPU-heavy work to a thread pool; completion and progress notifications are marshalled to the UI thread.
class WIDGET_EXPORT JobSystem : public QObject
{
public:
	explicit JobSystem(QObject* parent = nullptr);

	ProgressManager* progressManager() const { return m_progress; }

	/// Called from the UI thread. \a work runs on a pool thread; \a onFinished runs on the UI thread after \a work returns.
	/// If \a work throws, \a onFinished still runs with \a threw == true and \a throwMessage set.
	void enqueue(const QString& title, std::function<void(const JobProgressSink&)> work,
		std::function<void(bool threw, const QString& throwMessage)> onFinished);

private:
	ProgressManager* m_progress = nullptr;
	QThreadPool m_pool;
	std::atomic<quint64> m_nextJobId{ 0 };
};
