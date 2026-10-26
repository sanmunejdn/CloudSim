#ifndef ROBOTWIDGET_TIMELINESCHEDULER_H
#define ROBOTWIDGET_TIMELINESCHEDULER_H

/// @file TimelineScheduler.h
/// @brief 按 simTime 边沿触发时间轴片段动作

#include "robotwidget_global.h"

#include "TimelineStudyTypes.h"

#include <QHash>
#include <QObject>
#include <QSet>
#include <QString>

class RobotSimulationController;

class ROBOTWIDGET_EXPORT TimelineScheduler : public QObject
{
	Q_OBJECT

public:
	explicit TimelineScheduler(QObject* parent = nullptr);

	void setController(RobotSimulationController* ctrl);
	void setStudy(const TimelineStudy& study);
	const TimelineStudy& study() const { return m_study; }

	void resetEdgeState();
	/// 每拍调用；返回是否应因 duration 结束而停
	bool onSimTime(double tSec);

signals:
	void warningMessage(const QString& msg);

private:
	void fireClipEnter(const TimelineTrack& track, const TimelineClip& clip);
	void applyMaterialAt(double tSec);
	void applySignalWindows(double tSec);

	RobotSimulationController* m_ctrl = nullptr;
	TimelineStudy m_study;
	QSet<QString> m_firedClipIds;
	QHash<QString, bool> m_lastSignalOut; // owner|signal
};

#endif // ROBOTWIDGET_TIMELINESCHEDULER_H
