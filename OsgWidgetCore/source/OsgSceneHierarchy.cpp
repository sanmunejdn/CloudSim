#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

#include "OsgScene.h"

#include <osg/PositionAttitudeTransform>
#include <osg/Quat>
#include <osg/Vec3>

bool OsgScene::isBackendDescendantOf(const std::string& backendId, const std::string& ancestorId) const
{
	if (backendId.empty() || ancestorId.empty())
	{
		return false;
	}
	std::string cur = backendId;
	for (int depth = 0; depth < 2048 && !cur.empty(); ++depth)
	{
		if (cur == ancestorId)
		{
			return true;
		}
		const auto it = m_backendParentIds.find(cur);
		if (it == m_backendParentIds.end() || it->second.empty())
		{
			return false;
		}
		cur = it->second;
	}
	return false;
}

void OsgScene::cacheSelectionPoseFromSelectedTransform()
{
	if (!m_selectedTransform.valid())
	{
		return;
	}
	m_lastSelectionPos = m_selectedTransform->getPosition();
	m_lastSelectionAtt = m_selectedTransform->getAttitude();
	m_hasLastSelectionPose = true;
}

void OsgScene::syncActiveBackendRootFromSelectedTransform()
{
	if (!m_selectedTransform.valid())
	{
		return;
	}
	if (m_activeBackendId.empty())
	{
		return;
	}
	const osg::Vec3f curPos = m_selectedTransform->getPosition();
	const osg::Quat curAtt = m_selectedTransform->getAttitude();
	if (!m_hasLastSelectionPose)
	{
		m_lastSelectionPos = curPos;
		m_lastSelectionAtt = curAtt;
		m_hasLastSelectionPose = true;
		if (m_activeBackendOuterPat.valid())
		{
			m_activeBackendOuterPat->setPosition(curPos);
			m_activeBackendOuterPat->setAttitude(curAtt);
		}
		return;
	}

	const osg::Vec3f prevPos = m_lastSelectionPos;
	const osg::Quat prevAtt = m_lastSelectionAtt;
	const osg::Quat deltaAtt = curAtt * prevAtt.inverse();

	for (auto& kv : m_backendObjectRoots)
	{
		if (!kv.second.valid())
		{
			continue;
		}
		if (!isBackendDescendantOf(kv.first, m_activeBackendId))
		{
			continue;
		}
		if (kv.first == m_activeBackendId)
		{
			kv.second->setPosition(curPos);
			kv.second->setAttitude(curAtt);
			continue;
		}
		const osg::Vec3f oldPos = kv.second->getPosition();
		const osg::Quat oldAtt = kv.second->getAttitude();
		const osg::Vec3f newPos = curPos + (deltaAtt * (oldPos - prevPos));
		kv.second->setPosition(newPos);
		kv.second->setAttitude(deltaAtt * oldAtt);
	}
	m_lastSelectionPos = curPos;
	m_lastSelectionAtt = curAtt;
	m_hasLastSelectionPose = true;
}
