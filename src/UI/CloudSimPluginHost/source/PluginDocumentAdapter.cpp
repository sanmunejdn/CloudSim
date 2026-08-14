/// @file PluginDocumentAdapter.cpp
/// @brief PluginDocumentAdapter 实现

#include "PluginDocumentAdapter.h"

#include "BackendDataBase.h"
#include "BackendDataManager.h"
#include "DocumentHost.h"
#include "DocumentPointCloudOps.h"
#include "IDataService.h"
#include "PluginSceneBridgeAdapter.h"

#include <QTabWidget>

PluginDocumentAdapter::PluginDocumentAdapter(cloudsim::host::DocumentHost* host) : m_host(host)
{
	if (m_host)
	{
		m_sceneBridge = std::make_unique<PluginSceneBridgeAdapter>(m_host);
	}
}

std::string PluginDocumentAdapter::documentLabel() const
{
	if (!m_host || !m_host->parentWidget())
	{
		return std::string();
	}
	const auto* tabs = qobject_cast<QTabWidget*>(m_host->parentWidget());
	if (!tabs)
	{
		return std::string();
	}
	const int idx = tabs->indexOf(m_host);
	if (idx < 0)
	{
		return std::string();
	}
	return tabs->tabText(idx).toStdString();
}

std::size_t PluginDocumentAdapter::backendObjectCount() const
{
	if (!m_host)
	{
		return 0U;
	}
	return m_host->listObjects().size();
}

std::vector<std::string> PluginDocumentAdapter::backendIds() const
{
	std::vector<std::string> ids;
	if (!m_host)
	{
		return ids;
	}
	const auto list = m_host->listObjects();
	ids.reserve(list.size());
	for (const auto& obj : list)
	{
		if (obj)
		{
			ids.push_back(obj->id());
		}
	}
	return ids;
}

bool PluginDocumentAdapter::containsBackend(const std::string& backendId) const
{
	return m_host && m_host->data().isValid(QString::fromStdString(backendId));
}

std::string PluginDocumentAdapter::backendDisplayName(const std::string& backendId) const
{
	if (!m_host)
	{
		return std::string();
	}
	return m_host->data().displayName(QString::fromStdString(backendId)).toStdString();
}

std::string PluginDocumentAdapter::backendClassName(const std::string& backendId) const
{
	if (!m_host)
	{
		return std::string();
	}
	return m_host->data().className(QString::fromStdString(backendId)).toStdString();
}

std::string PluginDocumentAdapter::documentId() const
{
	if (!m_host)
	{
		return std::string();
	}
	return m_host->documentId().toStdString();
}

bool PluginDocumentAdapter::removeBackendObject(const std::string& backendIdUtf8, std::string* outError)
{
	if (!m_host || backendIdUtf8.empty())
	{
		if (outError)
		{
			*outError = "invalid document or backend id";
		}
		return false;
	}
	QString err;
	const bool ok = m_host->data().unregisterSubtree(QString::fromStdString(backendIdUtf8), &err);
	if (!ok && outError)
	{
		*outError = err.toStdString();
	}
	return ok;
}

IPluginSceneBridge* PluginDocumentAdapter::sceneBridge()
{
	return m_sceneBridge.get();
}

const IPluginSceneBridge* PluginDocumentAdapter::sceneBridge() const
{
	return m_sceneBridge.get();
}

bool PluginDocumentAdapter::queryPointCloudInfo(const std::string& backendIdUtf8, PluginPointCloudInfo& out) const
{
	return document_point_cloud_ops::queryPointCloudInfo(m_host, backendIdUtf8, out);
}

bool PluginDocumentAdapter::measurePointCloud(const std::string& backendIdUtf8, PluginPointCloudMeasure& out) const
{
	return document_point_cloud_ops::measurePointCloud(m_host, backendIdUtf8, out);
}

bool PluginDocumentAdapter::exportMeshToPly(const std::string& backendIdUtf8, const std::string& pathUtf8,
											std::string* outError) const
{
	if (document_point_cloud_ops::exportMeshToPly(m_host, backendIdUtf8, pathUtf8, outError))
	{
		return true;
	}
	// 点云 backend 无三角网格，回退到顶点 PLY 导出
	return document_point_cloud_ops::exportPointCloudToPly(m_host, backendIdUtf8, pathUtf8, outError);
}

bool PluginDocumentAdapter::getWorldPoseMm(const std::string& backendIdUtf8, WorldPoseMm* out) const
{
	if (!m_host || !out || backendIdUtf8.empty())
		return false;
	const cloudsim::core::PoseDto pose = m_host->data().worldPoseMm(QString::fromStdString(backendIdUtf8));
	out->xMm = pose.positionMm.x;
	out->yMm = pose.positionMm.y;
	out->zMm = pose.positionMm.z;
	out->rxDeg = pose.eulerDeg.x;
	out->ryDeg = pose.eulerDeg.y;
	out->rzDeg = pose.eulerDeg.z;
	return true;
}

bool PluginDocumentAdapter::applyWorldPoseMm(const std::string& backendIdUtf8, const WorldPoseMm& pose,
											 std::string* outError)
{
	if (!m_host || backendIdUtf8.empty())
	{
		if (outError)
			*outError = "invalid document or backend id";
		return false;
	}
	cloudsim::core::PoseDto dto;
	dto.positionMm.x = pose.xMm;
	dto.positionMm.y = pose.yMm;
	dto.positionMm.z = pose.zMm;
	dto.eulerDeg.x = pose.rxDeg;
	dto.eulerDeg.y = pose.ryDeg;
	dto.eulerDeg.z = pose.rzDeg;
	QString err;
	if (!m_host->data().applyWorldPoseMm(QString::fromStdString(backendIdUtf8), dto, &err))
	{
		if (outError)
			*outError = err.toStdString();
		return false;
	}
	return true;
}
