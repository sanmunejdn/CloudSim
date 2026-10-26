#ifndef ROBOTWIDGET_PLAYBACKTAPE_H
#define ROBOTWIDGET_PLAYBACKTAPE_H

/// @file PlaybackTape.h
/// @brief 时间轴采样磁带：scrub 读帧，避免可逆 tick

#include "robotwidget_global.h"

#include "TimelineStudyTypes.h"

#include <QHash>
#include <QString>
#include <QVector>

struct ROBOTWIDGET_EXPORT TapeMaterialState
{
	bool visible = true;
	TimelinePose3 pose;
};

struct ROBOTWIDGET_EXPORT TapeFrame
{
	double t = 0;
	QVector<double> jointAnglesRad;
	QHash<QString, bool> digitalOut; // owner|signal
	QHash<QString, TapeMaterialState> materials; // objectId
};

class ROBOTWIDGET_EXPORT PlaybackTape
{
public:
	void clear();
	void setCapacityHint(int maxFrames);

	void push(const TapeFrame& frame);
	bool empty() const { return m_frames.isEmpty(); }
	int size() const { return m_frames.size(); }
	double minT() const;
	double maxT() const;

	/// 最近邻；空带返回 false，越界钳到端帧
	bool sampleAt(double t, TapeFrame* out) const;

private:
	QVector<TapeFrame> m_frames;
	int m_maxFrames = 2400; // ~120s@20Hz
};

#endif // ROBOTWIDGET_PLAYBACKTAPE_H
