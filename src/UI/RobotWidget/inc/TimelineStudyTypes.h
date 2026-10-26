#ifndef ROBOTWIDGET_TIMELINESTUDYTYPES_H
#define ROBOTWIDGET_TIMELINESTUDYTYPES_H

/// @file TimelineStudyTypes.h
/// @brief Study 多轨时间轴数据模型（编排侧车）

#include "robotwidget_global.h"

#include <QJsonObject>
#include <QString>
#include <QVector>

enum class ROBOTWIDGET_EXPORT TimelineTrackType
{
	Robot = 0,
	Device,
	Signal,
	Material
};

struct ROBOTWIDGET_EXPORT TimelinePose3
{
	double x = 0;
	double y = 0;
	double z = 0;
	double exDeg = 0;
	double eyDeg = 0;
	double ezDeg = 0;
};

struct ROBOTWIDGET_EXPORT TimelineClip
{
	QString id;
	double tStart = 0;
	double tEnd = 0;
	/// robot
	QString programId;
	/// device
	QString poseId;
	double durationSec = 1.0;
	/// signal
	bool signalValue = true;
	/// material 关键帧
	bool visible = true;
	TimelinePose3 pose;
};

struct ROBOTWIDGET_EXPORT TimelineTrack
{
	QString id;
	TimelineTrackType type = TimelineTrackType::Robot;
	QString name;
	int instanceIndex = 0;
	QString ownerId;
	QString signal;
	/// out=写 DO；in=仅显示期望
	QString signalDirection = QStringLiteral("out");
	QString objectId;
	QVector<TimelineClip> clips;
};

struct ROBOTWIDGET_EXPORT TimelineStudy
{
	int version = 1;
	double durationSec = 30.0;
	double sampleHz = 20.0;
	QVector<TimelineTrack> tracks;
};

ROBOTWIDGET_EXPORT QString timelineTrackTypeToString(TimelineTrackType t);
ROBOTWIDGET_EXPORT TimelineTrackType timelineTrackTypeFromString(const QString& s);

ROBOTWIDGET_EXPORT QJsonObject timelineStudyToJson(const TimelineStudy& s);
ROBOTWIDGET_EXPORT bool timelineStudyFromJson(const QJsonObject& o, TimelineStudy* out, QString* err = nullptr);

/// 同轨片段区间半开 [tStart,tEnd) 禁止重叠（物料关键帧 tStart==tEnd 允许同点多个）
ROBOTWIDGET_EXPORT bool timelineStudyValidate(const TimelineStudy& s, QString* err = nullptr);

#endif // ROBOTWIDGET_TIMELINESTUDYTYPES_H
