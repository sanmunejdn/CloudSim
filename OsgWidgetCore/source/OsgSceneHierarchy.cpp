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

#include <osg/Matrixd>
#include <osg/MatrixTransform>
#include <osg/Quat>
#include <osg/Vec3>
#include <osg/Vec3d>

namespace
{
osg::Matrixd outerLocalFromPosQuat(const osg::Vec3f& pos, const osg::Quat& q)
{
	return osg::Matrixd::translate(osg::Vec3d(pos.x(), pos.y(), pos.z())) * osg::Matrixd::rotate(q);
}
} // namespace

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

bool OsgScene::backendOuterPatIsUnderOuterPatInSceneGraph(const std::string& childBackendId, const std::string& ancestorBackendId) const
{
	if (childBackendId.empty() || ancestorBackendId.empty() || childBackendId == ancestorBackendId)
	{
		return false;
	}
	const auto cIt = m_backendObjectRoots.find(childBackendId);
	const auto aIt = m_backendObjectRoots.find(ancestorBackendId);
	if (cIt == m_backendObjectRoots.end() || aIt == m_backendObjectRoots.end() || !cIt->second.valid() || !aIt->second.valid())
	{
		return false;
	}
	osg::MatrixTransform* const ancestorMt = aIt->second.get();
	osg::Node* p = cIt->second->getNumParents() > 0 ? cIt->second->getParent(0) : nullptr;
	while (p)
	{
		if (p == ancestorMt)
		{
			return true;
		}
		if (p == m_backendObjectsGroup.get())
		{
			return false;
		}
		p = p->getNumParents() > 0 ? p->getParent(0) : nullptr;
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
			m_activeBackendOuterPat->setMatrix(outerLocalFromPosQuat(curPos, curAtt));
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
			kv.second->setMatrix(outerLocalFromPosQuat(curPos, curAtt));
			continue;
		}
		// When the outer PAT is already nested under the active object's PAT in OSG, the parent transform
		// carries the child; do not apply the legacy flat-sibling delta (would double-move / corrupt pose).
		if (backendOuterPatIsUnderOuterPatInSceneGraph(kv.first, m_activeBackendId))
		{
			continue;
		}
		osg::Vec3d ot;
		osg::Quat oq;
		osg::Vec3d os;
		osg::Quat oso;
		kv.second->getMatrix().decompose(ot, oq, os, oso);
		const osg::Vec3f oldPos(static_cast<float>(ot.x()), static_cast<float>(ot.y()), static_cast<float>(ot.z()));
		const osg::Quat oldAtt = oq;
		const osg::Vec3f newPos = curPos + (deltaAtt * (oldPos - prevPos));
		kv.second->setMatrix(outerLocalFromPosQuat(newPos, deltaAtt * oldAtt));
	}
	m_lastSelectionPos = curPos;
	m_lastSelectionAtt = curAtt;
	m_hasLastSelectionPose = true;
}
