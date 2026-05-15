#pragma once

#include <functional>

#include <QObject>
#include <QString>

#include "widget_global.h"

/// Thread-safe bridge: workers report progress on any thread; signals are emitted on this object's thread (UI).
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
	/// success == worker exited without a C++ exception; business failures handled in the job completion callback may still leave success true.
	void jobFinished(quint64 jobId, bool success, QString errorMessage);
};
