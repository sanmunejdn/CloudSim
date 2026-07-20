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
	return m_host->backend().listData().size();
}

std::vector<std::string> PluginDocumentAdapter::backendIds() const
{
	std::vector<std::string> ids;
	if (!m_host)
	{
		return ids;
	}
	const auto list = m_host->backend().listData();
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
	return m_host && m_host->backend().contains(backendId);
}

std::string PluginDocumentAdapter::backendDisplayName(const std::string& backendId) const
{
	if (!m_host)
	{
		return std::string();
	}
	const auto obj = m_host->backend().getData(backendId);
	return obj ? obj->name() : std::string();
}

std::string PluginDocumentAdapter::backendClassName(const std::string& backendId) const
{
	if (!m_host)
	{
		return std::string();
	}
	const auto obj = m_host->backend().getData(backendId);
	return obj ? obj->className() : std::string();
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
