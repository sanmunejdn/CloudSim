/// @file TimelineScheduler.cpp

#include "TimelineScheduler.h"

#include "BackendDataBase.h"
#include "CoreTypes.h"
#include "IRobotDocumentHost.h"
#include "IRobotMainWindowHost.h"
#include "NamedSignalIoSink.h"
#include "NamedSignalTable.h"
#include "RobotSimulationController.h"


#include <algorithm>
#include <cmath>

namespace
{

QString clipKey(const QString& trackId, const QString& clipId)
{
	return trackId + QLatin1Char('|') + clipId;
}

QString signalKey(const QString& ownerId, const QString& signal)
{
	return ownerId + QLatin1Char('|') + signal;
}

TimelinePose3 lerpPose(const TimelinePose3& a, const TimelinePose3& b, double u)
{
	TimelinePose3 o;
	o.x = a.x + (b.x - a.x) * u;
	o.y = a.y + (b.y - a.y) * u;
	o.z = a.z + (b.z - a.z) * u;
	o.exDeg = a.exDeg + (b.exDeg - a.exDeg) * u;
	o.eyDeg = a.eyDeg + (b.eyDeg - a.eyDeg) * u;
	o.ezDeg = a.ezDeg + (b.ezDeg - a.ezDeg) * u;
	return o;
}

void applyMaterialState(IRobotDocumentHost* doc, const QString& objectId, bool visible, const TimelinePose3& pose)
{
	if (!doc || objectId.isEmpty())
	{
		return;
	}
	doc->documentData().setVisible(objectId, visible);
	if (auto obj = doc->findObject(objectId.toStdString()))
	{
		obj->setPose(BackendVec3{pose.x, pose.y, pose.z});
		obj->setRotation(BackendVec3{pose.exDeg, pose.eyDeg, pose.ezDeg});
	}
	doc->requestFollowSolveForced();
}
} // namespace

TimelineScheduler::TimelineScheduler(QObject* parent) : QObject(parent) {}

void TimelineScheduler::setController(RobotSimulationController* ctrl)
{
	m_ctrl = ctrl;
}

void TimelineScheduler::setStudy(const TimelineStudy& study)
{
	m_study = study;
}

void TimelineScheduler::resetEdgeState()
{
	m_firedClipIds.clear();
	m_lastSignalOut.clear();
}

bool TimelineScheduler::onSimTime(double tSec)
{
	if (!m_ctrl)
	{
		return true;
	}
	for (const TimelineTrack& tr : m_study.tracks)
	{
		if (tr.type == TimelineTrackType::Material)
		{
			continue;
		}
		for (const TimelineClip& c : tr.clips)
		{
			const QString key = clipKey(tr.id, c.id);
			const bool inside = (tSec + 1e-9 >= c.tStart) && (tSec < c.tEnd || std::abs(c.tEnd - c.tStart) < 1e-9);
			if (!inside || m_firedClipIds.contains(key))
			{
				continue;
			}
			// 关键帧与瞬时触发：进入即火一次
			if (tSec + 1e-9 >= c.tStart)
			{
				m_firedClipIds.insert(key);
				fireClipEnter(tr, c);
			}
		}
	}
	applySignalWindows(tSec);
	applyMaterialAt(tSec);
	return tSec + 1e-6 >= m_study.durationSec;
}

void TimelineScheduler::fireClipEnter(const TimelineTrack& track, const TimelineClip& clip)
{
	QString err;
	switch (track.type)
	{
	case TimelineTrackType::Robot:
	{
		const bool ok = m_ctrl->onStartRobotProgram(track.instanceIndex, clip.programId, &err);
		if (!ok)
		{
			emit warningMessage(err.isEmpty() ? QStringLiteral("timeline robot start failed") : err);
		}
		break;
	}
	case TimelineTrackType::Device:
		if (!m_ctrl->onGoNamedPose(track.ownerId, clip.poseId, clip.durationSec > 0 ? clip.durationSec : 1.0, &err))
		{
			emit warningMessage(err.isEmpty() ? QStringLiteral("timeline device pose failed") : err);
		}
		break;
	case TimelineTrackType::Signal:
	case TimelineTrackType::Material:
	default:
		break;
	}
}

void TimelineScheduler::applySignalWindows(double tSec)
{
	if (!m_ctrl)
	{
		return;
	}
	for (const TimelineTrack& tr : m_study.tracks)
	{
		if (tr.type != TimelineTrackType::Signal ||
			tr.signalDirection.compare(QStringLiteral("out"), Qt::CaseInsensitive) != 0)
		{
			continue;
		}
		bool want = false;
		for (const TimelineClip& c : tr.clips)
		{
			if (tSec + 1e-9 >= c.tStart && tSec < c.tEnd)
			{
				want = c.signalValue;
				break;
			}
		}
		const QString key = signalKey(tr.ownerId, tr.signal);
		if (m_lastSignalOut.contains(key) && m_lastSignalOut.value(key) == want)
		{
			continue;
		}
		m_lastSignalOut.insert(key, want);
		NamedSignalIoSink* sink = m_ctrl->simulationIoSinkForOwner(tr.ownerId);
		RobotIo::NamedSignalTable* table = m_ctrl->namedSignalTableForOwner(tr.ownerId);
		if (!sink || !table)
		{
			continue;
		}
		const RobotIo::SignalDef* def = table->findByName(tr.signal.toStdString());
		if (!def || def->kind != RobotIo::SignalKind::DO)
		{
			continue;
		}
		sink->setDigitalOutput(def->port, want);
	}
}

void TimelineScheduler::applyMaterialAt(double tSec)
{
	IRobotDocumentHost* doc = (m_ctrl && m_ctrl->host()) ? m_ctrl->host()->document() : nullptr;
	if (!doc)
	{
		return;
	}
	for (const TimelineTrack& tr : m_study.tracks)
	{
		if (tr.type != TimelineTrackType::Material || tr.objectId.isEmpty() || tr.clips.isEmpty())
		{
			continue;
		}
		QVector<TimelineClip> kfs = tr.clips;
		std::sort(kfs.begin(), kfs.end(),
				  [](const TimelineClip& a, const TimelineClip& b) { return a.tStart < b.tStart; });
		if (tSec <= kfs.front().tStart)
		{
			applyMaterialState(doc, tr.objectId, kfs.front().visible, kfs.front().pose);
			continue;
		}
		if (tSec >= kfs.back().tStart)
		{
			applyMaterialState(doc, tr.objectId, kfs.back().visible, kfs.back().pose);
			continue;
		}
		for (int i = 0; i + 1 < kfs.size(); ++i)
		{
			if (tSec >= kfs[i].tStart && tSec <= kfs[i + 1].tStart)
			{
				const double span = std::max(1e-9, kfs[i + 1].tStart - kfs[i].tStart);
				const double u = (tSec - kfs[i].tStart) / span;
				applyMaterialState(doc, tr.objectId, kfs[i].visible, lerpPose(kfs[i].pose, kfs[i + 1].pose, u));
				break;
			}
		}
	}
}
