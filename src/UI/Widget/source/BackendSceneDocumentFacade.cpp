#include "BackendSceneDocumentFacade.h"

#include "BackendDataManager.h"
#include "BackendFollowReverseIndex.h"
#include "IBackendSceneBridge.h"
#include "IRobotBackendPoseSink.h"
#include "MeshBackendData.h"
#include "OsgWidget.h"
#include "PointCloudBackendData.h"

BackendSceneEntity::BackendSceneEntity(std::string backendId, IBackendSceneBridge* bridge, BackendDataManager* mgr,
	BackendFollowReverseIndex* followIndex)
	: m_id(std::move(backendId))
	, m_bridge(bridge)
	, m_mgr(mgr)
	, m_followIndex(followIndex)
{
}

bool BackendSceneEntity::valid() const
{
	return m_bridge && m_mgr && !m_id.empty();
}

void BackendSceneEntity::setVisible(bool visible)
{
	if (!valid())
	{
		return;
	}
	m_bridge->setBackendObjectVisible(m_id, visible);
}

void BackendSceneEntity::show()
{
	setVisible(true);
}

void BackendSceneEntity::hide()
{
	setVisible(false);
}

bool BackendSceneEntity::setWorldMatrixColumnMajor(const std::array<double, 16>& columnMajor4x4)
{
	if (!valid())
	{
		return false;
	}
	m_bridge->setBackendRootWorldMatrixColumnMajor(m_id, columnMajor4x4);
	return true;
}

bool BackendSceneEntity::getWorldMatrixColumnMajor(std::array<double, 16>& outColumnMajor4x4) const
{
	if (!valid())
	{
		return false;
	}
	return m_bridge->getBackendRootWorldMatrixColumnMajor(m_id, outColumnMajor4x4);
}

void BackendSceneEntity::removeVisual()
{
	if (!valid())
	{
		return;
	}
	m_bridge->removeBackendObjectVisual(m_id);
}

bool BackendSceneEntity::hasSceneBranch() const
{
	if (!valid())
	{
		return false;
	}
	return m_bridge->hasBackendObjectBranch(m_id);
}

bool BackendSceneEntity::tryGetModelCenterMm(double& outCx, double& outCy, double& outCz) const
{
	if (!valid())
	{
		return false;
	}
	return m_bridge->tryGetBackendModelCenterMm(m_id, outCx, outCy, outCz);
}

void BackendSceneEntity::syncOuterPatFromBackend(const BackendDataBase& data)
{
	if (!valid())
	{
		return;
	}
	m_bridge->syncOuterPatFromBackend(data);
}

void BackendSceneEntity::setParentBackend(const std::string& parentBackendId)
{
	if (!valid())
	{
		return;
	}
	m_bridge->setBackendParent(m_id, parentBackendId);
}

std::vector<std::string> BackendSceneEntity::childBackendIds() const
{
	if (!m_mgr || m_id.empty())
	{
		return {};
	}
	return m_mgr->childrenOf(m_id);
}

std::vector<std::string> BackendSceneEntity::followerBackendIds() const
{
	if (!m_mgr || !m_followIndex || m_id.empty())
	{
		return {};
	}
	return m_followIndex->followersOf(*m_mgr, m_id);
}

BackendSceneDocumentFacade::BackendSceneDocumentFacade(BackendDataManager& mgr, IBackendSceneBridge& bridge,
	BackendFollowReverseIndex& followIndex, OsgWidget* osgWidget)
	: m_mgr(&mgr)
	, m_bridge(&bridge)
	, m_followIndex(&followIndex)
	, m_poseSink(static_cast<IRobotBackendPoseSink*>(osgWidget))
{
}

BackendSceneEntity BackendSceneDocumentFacade::entity(const std::string& backendId) const
{
	return BackendSceneEntity(backendId, m_bridge, m_mgr, m_followIndex);
}

void BackendSceneDocumentFacade::setBackendsVisible(const std::vector<std::string>& backendIds, bool visible)
{
	if (!m_bridge)
	{
		return;
	}
	for (const std::string& id : backendIds)
	{
		if (!id.empty())
		{
			m_bridge->setBackendObjectVisible(id, visible);
		}
	}
}

IRobotBackendPoseSink* BackendSceneDocumentFacade::poseSink() const
{
	return m_poseSink;
}

void BackendSceneDocumentFacade::ensureSelectionVisualForBackend(const BackendDataBase& data, const bool urdfLinkMesh)
{
	OsgWidget* osg = static_cast<OsgWidget*>(m_poseSink);
	if (!osg)
	{
		return;
	}
	const std::string idStd = data.id();
	osg->syncSelectionForBackendId(idStd);
	if (const auto* pc = dynamic_cast<const PointCloudBackendData*>(&data))
	{
		if (!pc->pointPositionsXyz().empty())
		{
			if (entity(idStd).hasSceneBranch())
			{
				osg->syncSelectionFromBackend(*pc);
			}
			else
			{
				QString geomErr;
				osg->loadPointCloudFromBackendData(*pc, &geomErr, false);
			}
		}
		return;
	}
	if (const auto* mesh = dynamic_cast<const MeshBackendData*>(&data))
	{
		if (!mesh->triangleSoup().empty())
		{
			if (entity(idStd).hasSceneBranch())
			{
				osg->syncSelectionFromBackend(*mesh);
			}
			else
			{
				QString geomErr;
				if (urdfLinkMesh)
				{
					osg->loadMeshFromBackendData(*mesh, &geomErr, false, true, true);
				}
				else
				{
					osg->loadMeshFromBackendData(*mesh, &geomErr, false);
				}
			}
		}
		else
		{
			osg->syncSelectionForBackendId(idStd);
		}
		return;
	}
	osg->syncSelectionForBackendId(idStd);
}
