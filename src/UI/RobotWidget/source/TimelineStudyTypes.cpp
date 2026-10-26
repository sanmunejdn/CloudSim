/// @file TimelineStudyTypes.cpp

#include "TimelineStudyTypes.h"

#include <QJsonArray>
#include <QUuid>
#include <algorithm>
#include <cmath>

QString timelineTrackTypeToString(TimelineTrackType t)
{
	switch (t)
	{
	case TimelineTrackType::Device:
		return QStringLiteral("device");
	case TimelineTrackType::Signal:
		return QStringLiteral("signal");
	case TimelineTrackType::Material:
		return QStringLiteral("material");
	case TimelineTrackType::Robot:
	default:
		return QStringLiteral("robot");
	}
}

TimelineTrackType timelineTrackTypeFromString(const QString& s)
{
	if (s.compare(QStringLiteral("device"), Qt::CaseInsensitive) == 0)
	{
		return TimelineTrackType::Device;
	}
	if (s.compare(QStringLiteral("signal"), Qt::CaseInsensitive) == 0)
	{
		return TimelineTrackType::Signal;
	}
	if (s.compare(QStringLiteral("material"), Qt::CaseInsensitive) == 0)
	{
		return TimelineTrackType::Material;
	}
	return TimelineTrackType::Robot;
}

namespace
{
QJsonObject poseToJson(const TimelinePose3& p)
{
	QJsonObject o;
	o.insert(QStringLiteral("x"), p.x);
	o.insert(QStringLiteral("y"), p.y);
	o.insert(QStringLiteral("z"), p.z);
	o.insert(QStringLiteral("exDeg"), p.exDeg);
	o.insert(QStringLiteral("eyDeg"), p.eyDeg);
	o.insert(QStringLiteral("ezDeg"), p.ezDeg);
	return o;
}

TimelinePose3 poseFromJson(const QJsonObject& o)
{
	TimelinePose3 p;
	p.x = o.value(QStringLiteral("x")).toDouble();
	p.y = o.value(QStringLiteral("y")).toDouble();
	p.z = o.value(QStringLiteral("z")).toDouble();
	p.exDeg = o.value(QStringLiteral("exDeg")).toDouble();
	p.eyDeg = o.value(QStringLiteral("eyDeg")).toDouble();
	p.ezDeg = o.value(QStringLiteral("ezDeg")).toDouble();
	return p;
}

QJsonObject clipToJson(const TimelineClip& c)
{
	QJsonObject o;
	o.insert(QStringLiteral("id"), c.id);
	o.insert(QStringLiteral("tStart"), c.tStart);
	o.insert(QStringLiteral("tEnd"), c.tEnd);
	o.insert(QStringLiteral("programId"), c.programId);
	o.insert(QStringLiteral("poseId"), c.poseId);
	o.insert(QStringLiteral("durationSec"), c.durationSec);
	o.insert(QStringLiteral("value"), c.signalValue);
	o.insert(QStringLiteral("visible"), c.visible);
	o.insert(QStringLiteral("pose"), poseToJson(c.pose));
	return o;
}

TimelineClip clipFromJson(const QJsonObject& o)
{
	TimelineClip c;
	c.id = o.value(QStringLiteral("id")).toString();
	if (c.id.isEmpty())
	{
		c.id = QUuid::createUuid().toString(QUuid::WithoutBraces);
	}
	c.tStart = o.value(QStringLiteral("tStart")).toDouble();
	c.tEnd = o.value(QStringLiteral("tEnd")).toDouble();
	c.programId = o.value(QStringLiteral("programId")).toString();
	c.poseId = o.value(QStringLiteral("poseId")).toString();
	c.durationSec = o.value(QStringLiteral("durationSec")).toDouble(1.0);
	c.signalValue = o.value(QStringLiteral("value")).toBool(true);
	c.visible = o.value(QStringLiteral("visible")).toBool(true);
	c.pose = poseFromJson(o.value(QStringLiteral("pose")).toObject());
	return c;
}
} // namespace

QJsonObject timelineStudyToJson(const TimelineStudy& s)
{
	QJsonObject root;
	root.insert(QStringLiteral("version"), s.version);
	root.insert(QStringLiteral("durationSec"), s.durationSec);
	root.insert(QStringLiteral("sampleHz"), s.sampleHz);
	QJsonArray tracks;
	for (const TimelineTrack& tr : s.tracks)
	{
		QJsonObject to;
		to.insert(QStringLiteral("id"), tr.id);
		to.insert(QStringLiteral("type"), timelineTrackTypeToString(tr.type));
		to.insert(QStringLiteral("name"), tr.name);
		to.insert(QStringLiteral("instanceIndex"), tr.instanceIndex);
		to.insert(QStringLiteral("ownerId"), tr.ownerId);
		to.insert(QStringLiteral("signal"), tr.signal);
		to.insert(QStringLiteral("direction"), tr.signalDirection);
		to.insert(QStringLiteral("objectId"), tr.objectId);
		QJsonArray clips;
		for (const TimelineClip& c : tr.clips)
		{
			clips.append(clipToJson(c));
		}
		to.insert(QStringLiteral("clips"), clips);
		tracks.append(to);
	}
	root.insert(QStringLiteral("tracks"), tracks);
	return root;
}

bool timelineStudyFromJson(const QJsonObject& o, TimelineStudy* out, QString* err)
{
	if (!out)
	{
		if (err)
		{
			*err = QStringLiteral("null out");
		}
		return false;
	}
	TimelineStudy s;
	s.version = o.value(QStringLiteral("version")).toInt(1);
	s.durationSec = o.value(QStringLiteral("durationSec")).toDouble(30.0);
	s.sampleHz = o.value(QStringLiteral("sampleHz")).toDouble(20.0);
	const QJsonArray tracks = o.value(QStringLiteral("tracks")).toArray();
	for (const QJsonValue& tv : tracks)
	{
		if (!tv.isObject())
		{
			continue;
		}
		const QJsonObject to = tv.toObject();
		TimelineTrack tr;
		tr.id = to.value(QStringLiteral("id")).toString();
		if (tr.id.isEmpty())
		{
			tr.id = QUuid::createUuid().toString(QUuid::WithoutBraces);
		}
		tr.type = timelineTrackTypeFromString(to.value(QStringLiteral("type")).toString());
		tr.name = to.value(QStringLiteral("name")).toString();
		tr.instanceIndex = to.value(QStringLiteral("instanceIndex")).toInt();
		tr.ownerId = to.value(QStringLiteral("ownerId")).toString();
		tr.signal = to.value(QStringLiteral("signal")).toString();
		tr.signalDirection = to.value(QStringLiteral("direction")).toString(QStringLiteral("out"));
		tr.objectId = to.value(QStringLiteral("objectId")).toString();
		const QJsonArray clips = to.value(QStringLiteral("clips")).toArray();
		for (const QJsonValue& cv : clips)
		{
			if (cv.isObject())
			{
				tr.clips.push_back(clipFromJson(cv.toObject()));
			}
		}
		s.tracks.push_back(tr);
	}
	QString verr;
	if (!timelineStudyValidate(s, &verr))
	{
		if (err)
		{
			*err = verr;
		}
		return false;
	}
	*out = s;
	return true;
}

bool timelineStudyValidate(const TimelineStudy& s, QString* err)
{
	for (const TimelineTrack& tr : s.tracks)
	{
		if (tr.type == TimelineTrackType::Material)
		{
			continue;
		}
		QVector<TimelineClip> sorted = tr.clips;
		std::sort(sorted.begin(), sorted.end(),
				  [](const TimelineClip& a, const TimelineClip& b) { return a.tStart < b.tStart; });
		for (int i = 1; i < sorted.size(); ++i)
		{
			if (sorted[i].tStart < sorted[i - 1].tEnd - 1e-9)
			{
				if (err)
				{
					*err = QStringLiteral("overlapping clips on track %1").arg(tr.name.isEmpty() ? tr.id : tr.name);
				}
				return false;
			}
		}
	}
	return true;
}
