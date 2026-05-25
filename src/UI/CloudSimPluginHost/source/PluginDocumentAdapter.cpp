#include "PluginDocumentAdapter.h"

#include "BackendDataBase.h"
#include "BackendDataManager.h"
#include "DocumentPage.h"
#include "DocumentPointCloudOps.h"
#include "IDataService.h"
#include "PluginSceneBridgeAdapter.h"

#include <QTabWidget>

PluginDocumentAdapter::PluginDocumentAdapter(DocumentPage* page)
	: m_page(page)
{
	if (m_page)
	{
		m_sceneBridge = std::make_unique<PluginSceneBridgeAdapter>(m_page);
	}
}

std::string PluginDocumentAdapter::documentLabel() const
{
	if (!m_page || !m_page->parentWidget())
	{
		return std::string();
	}
	const auto* tabs = qobject_cast<QTabWidget*>(m_page->parentWidget());
	if (!tabs)
	{
		return std::string();
	}
	const int idx = tabs->indexOf(m_page);
	if (idx < 0)
	{
		return std::string();
	}
	return tabs->tabText(idx).toStdString();
}

std::size_t PluginDocumentAdapter::backendObjectCount() const
{
	if (!m_page)
	{
		return 0U;
	}
	return m_page->backend().listData().size();
}

std::vector<std::string> PluginDocumentAdapter::backendIds() const
{
	std::vector<std::string> ids;
	if (!m_page)
	{
		return ids;
	}
	const auto list = m_page->backend().listData();
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
	return m_page && m_page->backend().contains(backendId);
}

std::string PluginDocumentAdapter::backendDisplayName(const std::string& backendId) const
{
	if (!m_page)
	{
		return std::string();
	}
	const auto obj = m_page->backend().getData(backendId);
	return obj ? obj->name() : std::string();
}

std::string PluginDocumentAdapter::backendClassName(const std::string& backendId) const
{
	if (!m_page)
	{
		return std::string();
	}
	const auto obj = m_page->backend().getData(backendId);
	return obj ? obj->className() : std::string();
}

std::string PluginDocumentAdapter::documentId() const
{
	if (!m_page)
	{
		return std::string();
	}
	return m_page->documentId().toStdString();
}

bool PluginDocumentAdapter::removeBackendObject(const std::string& backendIdUtf8, std::string* outError)
{
	if (!m_page || backendIdUtf8.empty())
	{
		if (outError)
		{
			*outError = "invalid document or backend id";
		}
		return false;
	}
	QString err;
	const bool ok = m_page->data().unregisterSubtree(QString::fromStdString(backendIdUtf8), &err);
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
	return document_point_cloud_ops::queryPointCloudInfo(m_page, backendIdUtf8, out);
}

bool PluginDocumentAdapter::measurePointCloud(const std::string& backendIdUtf8, PluginPointCloudMeasure& out) const
{
	return document_point_cloud_ops::measurePointCloud(m_page, backendIdUtf8, out);
}

bool PluginDocumentAdapter::exportMeshToPly(
	const std::string& backendIdUtf8,
	const std::string& pathUtf8,
	std::string* outError) const
{
	return document_point_cloud_ops::exportMeshToPly(m_page, backendIdUtf8, pathUtf8, outError);
}
