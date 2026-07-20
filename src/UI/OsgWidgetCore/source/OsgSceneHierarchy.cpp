/// @file OsgSceneHierarchy.cpp
/// @brief OsgSceneHierarchy 实现

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

#include "ObjectGizmoFrame.h"
#include "OsgScene.h"

#include <osg/MatrixTransform>
#include <osg/Matrixd>
#include <osg/Quat>
#include <osg/Vec3>
#include <osg/Vec3d>

void OsgScene::cacheSelectionGizmoPose()
{
	ObjectGizmoFrame f;
	if (!readActiveObjectGizmoFrame(f))
	{
		return;
	}
	m_lastGizmoCenterPlusPose = f.centerPlusPose();
	m_lastGizmoAttitude = f.attitude();
	m_hasLastSelectionPose = true;
}

void OsgScene::syncActiveBackendRootFromObjectFrame(const ObjectGizmoFrame& cur, bool dragging)
{
	if (m_activeBackendId.empty() || !m_activeBackendOuterPat.valid())
	{
		return;
	}
	if (dragging)
	{
		m_activeBackendOuterPat->setMatrix(ObjectGizmoFrame::outerLocalMatrix(cur.centerPlusPose(), cur.attitude()));
		m_lastGizmoCenterPlusPose = cur.centerPlusPose();
		m_lastGizmoAttitude = cur.attitude();
		m_hasLastSelectionPose = true;
		return;
	}
	if (!m_hasLastSelectionPose)
	{
		m_lastGizmoCenterPlusPose = cur.centerPlusPose();
		m_lastGizmoAttitude = cur.attitude();
		m_hasLastSelectionPose = true;
		m_activeBackendOuterPat->setMatrix(ObjectGizmoFrame::outerLocalMatrix(cur.centerPlusPose(), cur.attitude()));
		return;
	}

	const osg::Vec3f prevPos = m_lastGizmoCenterPlusPose;
	const osg::Quat prevAtt = m_lastGizmoAttitude;
	const osg::Quat deltaAtt = cur.attitude() * prevAtt.inverse();
	const osg::Vec3f curPos = cur.centerPlusPose();
	const osg::Quat curAtt = cur.attitude();

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
			kv.second->setMatrix(ObjectGizmoFrame::outerLocalMatrix(curPos, curAtt));
			continue;
		}
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
		kv.second->setMatrix(ObjectGizmoFrame::outerLocalMatrix(newPos, deltaAtt * oldAtt));
	}
	m_lastGizmoCenterPlusPose = curPos;
	m_lastGizmoAttitude = curAtt;
	m_hasLastSelectionPose = true;
}

void OsgScene::setBackendLogicalParent(const std::string& backendId, const std::string& parentBackendId)
{
	if (backendId.empty())
	{
		return;
	}
	if (parentBackendId.empty() || parentBackendId == backendId)
	{
		m_backendParentIds.erase(backendId);
		return;
	}
	m_backendParentIds[backendId] = parentBackendId;
}

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

bool OsgScene::backendOuterPatIsUnderOuterPatInSceneGraph(const std::string& childBackendId,
														  const std::string& ancestorBackendId) const
{
	if (childBackendId.empty() || ancestorBackendId.empty() || childBackendId == ancestorBackendId)
	{
		return false;
	}
	const auto cIt = m_backendObjectRoots.find(childBackendId);
	const auto aIt = m_backendObjectRoots.find(ancestorBackendId);
	if (cIt == m_backendObjectRoots.end() || aIt == m_backendObjectRoots.end() || !cIt->second.valid() ||
		!aIt->second.valid())
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
