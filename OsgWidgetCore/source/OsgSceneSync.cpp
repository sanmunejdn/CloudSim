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

#include <algorithm>
#include <cmath>

#include "BackendDataBase.h"
#include "MeshBackendData.h"
#include "PointCloudBackendData.h"

#include <osg/Node>
#include <osg/PositionAttitudeTransform>

osg::Vec3f OsgScene::computePointCloudCenterFromXyz(const std::vector<float>& xyz) const
{
	if (xyz.size() < 3U || (xyz.size() % 3U) != 0U)
	{
		return osg::Vec3f(0.0f, 0.0f, 0.0f);
	}
	float minx = xyz[0], maxx = xyz[0];
	float miny = xyz[1], maxy = xyz[1];
	float minz = xyz[2], maxz = xyz[2];
	for (std::size_t i = 0; i + 2 < xyz.size(); i += 3U)
	{
		const float x = xyz[i], y = xyz[i + 1], z = xyz[i + 2];
		minx = std::min(minx, x);
		maxx = std::max(maxx, x);
		miny = std::min(miny, y);
		maxy = std::max(maxy, y);
		minz = std::min(minz, z);
		maxz = std::max(maxz, z);
	}
	return osg::Vec3f(0.5f * (minx + maxx), 0.5f * (miny + maxy), 0.5f * (minz + maxz));
}

float OsgScene::computePointCloudDiagonalFromXyz(const std::vector<float>& xyz) const
{
	if (xyz.size() < 3U || (xyz.size() % 3U) != 0U)
	{
		return 1.0f;
	}
	float minx = xyz[0], maxx = xyz[0];
	float miny = xyz[1], maxy = xyz[1];
	float minz = xyz[2], maxz = xyz[2];
	for (std::size_t i = 0; i + 2 < xyz.size(); i += 3U)
	{
		const float x = xyz[i], y = xyz[i + 1], z = xyz[i + 2];
		minx = std::min(minx, x);
		maxx = std::max(maxx, x);
		miny = std::min(miny, y);
		maxy = std::max(maxy, y);
		minz = std::min(minz, z);
		maxz = std::max(maxz, z);
	}
	const float dx = maxx - minx;
	const float dy = maxy - miny;
	const float dz = maxz - minz;
	return std::max(1.0f, std::sqrt(dx * dx + dy * dy + dz * dz));
}

osg::Vec3f OsgScene::computeMeshCenterFromSoup(const std::vector<float>& soup) const
{
	if (soup.size() < 3U || (soup.size() % 3U) != 0U)
	{
		return osg::Vec3f(0.0f, 0.0f, 0.0f);
	}
	float minx = soup[0], maxx = soup[0], miny = soup[1], maxy = soup[1], minz = soup[2], maxz = soup[2];
	for (std::size_t i = 0; i + 2 < soup.size(); i += 3U)
	{
		const float x = soup[i], y = soup[i + 1], z = soup[i + 2];
		minx = std::min(minx, x);
		maxx = std::max(maxx, x);
		miny = std::min(miny, y);
		maxy = std::max(maxy, y);
		minz = std::min(minz, z);
		maxz = std::max(maxz, z);
	}
	return osg::Vec3f(0.5f * (minx + maxx), 0.5f * (miny + maxy), 0.5f * (minz + maxz));
}

float OsgScene::computeMeshDiagonalFromSoup(const std::vector<float>& soup) const
{
	if (soup.size() < 3U || (soup.size() % 3U) != 0U)
	{
		return 1.0f;
	}
	float minx = soup[0], maxx = soup[0], miny = soup[1], maxy = soup[1], minz = soup[2], maxz = soup[2];
	for (std::size_t i = 0; i + 2 < soup.size(); i += 3U)
	{
		const float x = soup[i], y = soup[i + 1], z = soup[i + 2];
		minx = std::min(minx, x);
		maxx = std::max(maxx, x);
		miny = std::min(miny, y);
		maxy = std::max(maxy, y);
		minz = std::min(minz, z);
		maxz = std::max(maxz, z);
	}
	const float dx = maxx - minx;
	const float dy = maxy - miny;
	const float dz = maxz - minz;
	return std::max(1.0f, std::sqrt(dx * dx + dy * dy + dz * dz));
}

void OsgScene::syncGizmoAndPickFromPointCloudBackend(const PointCloudBackendData& data)
{
	const std::string id = data.id();
	m_activeBackendId = id;
	auto it = m_backendObjectRoots.find(id);
	if (it != m_backendObjectRoots.end() && it->second.valid())
	{
		m_activeBackendOuterPat = it->second;
	}
	else
	{
		m_activeBackendOuterPat = nullptr;
	}
	auto cIt = m_backendModelCenters.find(id);
	if (cIt != m_backendModelCenters.end())
	{
		m_modelCenter = cIt->second;
	}
	else
	{
		m_modelCenter = computePointCloudCenterFromXyz(data.pointPositionsXyz());
		m_backendModelCenters[id] = m_modelCenter;
	}
	m_activeModelDiagonal = computePointCloudDiagonalFromXyz(data.pointPositionsXyz());
	if (m_selectedTransform.valid())
	{
		if (it != m_backendObjectRoots.end() && it->second.valid())
		{
			osg::PositionAttitudeTransform* outer = it->second.get();
			m_selectedTransform->setPosition(outer->getPosition());
			m_selectedTransform->setAttitude(outer->getAttitude());
		}
		else
		{
			const BackendVec3 p = data.pose();
			const BackendVec3 r = data.rotation();
			const osg::Vec3f pose(static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z));
			m_selectedTransform->setPosition(m_modelCenter + pose);
			m_selectedTransform->setAttitude(eulerDegToQuat(osg::Vec3f(static_cast<float>(r.x), static_cast<float>(r.y), static_cast<float>(r.z))));
		}
	}
	syncActiveBackendRootFromSelectedTransform();
	osg::Node* pickNode = nullptr;
	if (m_activeBackendOuterPat.valid() && m_activeBackendOuterPat->getNumChildren() > 0)
	{
		auto* inner = dynamic_cast<osg::PositionAttitudeTransform*>(m_activeBackendOuterPat->getChild(0));
		if (inner && inner->getNumChildren() > 0)
		{
			pickNode = inner->getChild(0);
		}
	}
	if (pickNode)
	{
		cachePickablePointsFromNode(pickNode);
	}
}

void OsgScene::syncGizmoAndPickFromMeshBackend(const MeshBackendData& data)
{
	const std::string id = data.id();
	m_activeBackendId = id;
	auto it = m_backendObjectRoots.find(id);
	if (it != m_backendObjectRoots.end() && it->second.valid())
	{
		m_activeBackendOuterPat = it->second;
	}
	else
	{
		m_activeBackendOuterPat = nullptr;
	}
	auto cIt = m_backendModelCenters.find(id);
	if (cIt != m_backendModelCenters.end())
	{
		m_modelCenter = cIt->second;
	}
	else
	{
		m_modelCenter = computeMeshCenterFromSoup(data.triangleSoup());
		m_backendModelCenters[id] = m_modelCenter;
	}
	m_activeModelDiagonal = computeMeshDiagonalFromSoup(data.triangleSoup());
	if (m_selectedTransform.valid())
	{
		if (it != m_backendObjectRoots.end() && it->second.valid())
		{
			osg::PositionAttitudeTransform* outer = it->second.get();
			m_selectedTransform->setPosition(outer->getPosition());
			m_selectedTransform->setAttitude(outer->getAttitude());
		}
		else
		{
			const BackendVec3 p = data.pose();
			const BackendVec3 r = data.rotation();
			const osg::Vec3f pose(static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z));
			m_selectedTransform->setPosition(m_modelCenter + pose);
			m_selectedTransform->setAttitude(eulerDegToQuat(osg::Vec3f(static_cast<float>(r.x), static_cast<float>(r.y), static_cast<float>(r.z))));
		}
	}
	syncActiveBackendRootFromSelectedTransform();
	osg::Node* pickNode = nullptr;
	if (m_activeBackendOuterPat.valid() && m_activeBackendOuterPat->getNumChildren() > 0)
	{
		auto* inner = dynamic_cast<osg::PositionAttitudeTransform*>(m_activeBackendOuterPat->getChild(0));
		if (inner && inner->getNumChildren() > 0)
		{
			pickNode = inner->getChild(0);
		}
	}
	if (pickNode)
	{
		cachePickablePointsFromNode(pickNode);
	}
}
