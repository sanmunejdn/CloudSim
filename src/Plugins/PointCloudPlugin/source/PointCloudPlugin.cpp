#include "PointCloudPlugin.h"

#include "PointCloudDockWidget.h"
#include "CloudSimPluginVersion.h"
#include "IPluginHostContext.h"
#include "IPluginPointCloudHost.h"

#include <QMenu>

QString PointCloudPlugin::pluginId() const
{
	return QStringLiteral("com.cloudsim.pointcloud");
}

QString PointCloudPlugin::displayName() const
{
	return QStringLiteral("Point Cloud");
}

bool PointCloudPlugin::initialize(IPluginHostContext* host)
{
	if (!host)
	{
		return false;
	}
	if (host->hostVersion() < 0x00010200)
	{
		host->logError(QStringLiteral("PointCloudPlugin requires host 1.2.0+"));
		return false;
	}
	if (!host->pointCloudHost())
	{
		host->logError(QStringLiteral("Point cloud host API unavailable"));
		return false;
	}
	m_host = host;

	if (!host->sidePanelTabParent())
	{
		return false;
	}
	m_dockWidget = new PointCloudDockWidget(host, nullptr);
	const char* tabTitle = host->useChinese() ? "点云" : "Point Cloud";
	if (host->registerSidePanelTab(tabTitle, m_dockWidget) < 0)
	{
		return false;
	}

	host->onActiveDocumentChanged([this](IPluginDocument*) {
		if (m_dockWidget)
		{
			m_dockWidget->refreshDocumentLabel();
			m_dockWidget->refreshPointCloudList();
		}
	});

	host->onLanguageChanged([this](const bool) {
		applyLanguage();
	});

	registerMenus();
	applyLanguage();
	host->logInfo(host->useChinese() ? QStringLiteral("点云插件已加载。") : QStringLiteral("PointCloud plugin initialized."));
	return true;
}

void PointCloudPlugin::shutdown()
{
	if (m_host && m_dockWidget)
	{
		m_host->unregisterSidePanelTab(m_dockWidget);
	}
	m_dockWidget = nullptr;
	m_host = nullptr;
	m_pointCloudMenu = nullptr;
	m_importAction = nullptr;
	m_downsampleAction = nullptr;
	m_reconstructAction = nullptr;
	m_exportMeshAction = nullptr;
	m_refreshAction = nullptr;
	m_simplifyAction = nullptr;
	m_smoothAction = nullptr;
}

void PointCloudPlugin::registerMenus()
{
	if (!m_host)
	{
		return;
	}
	m_pointCloudMenu = m_host->registerMenuPath({ QStringLiteral("Tools"), QStringLiteral("Point Cloud") });
	if (!m_pointCloudMenu)
	{
		return;
	}
	m_importAction = m_host->registerAction(
		m_pointCloudMenu, QStringLiteral("Import Point Cloud..."), [this]() { importPointCloud(); });
	m_downsampleAction = m_host->registerAction(
		m_pointCloudMenu, QStringLiteral("Downsample Voxel"), [this]() { downsampleVoxelOnSelection(); });
	m_reconstructAction = m_host->registerAction(
		m_pointCloudMenu, QStringLiteral("Reconstruct Poisson Auto"), [this]() { reconstructPoissonOnSelection(); });
	m_exportMeshAction = m_host->registerAction(
		m_pointCloudMenu, QStringLiteral("Export Mesh PLY..."), [this]() { exportMeshOnSelection(); });
	m_refreshAction = m_host->registerAction(m_pointCloudMenu, QStringLiteral("Refresh List"), [this]() {
		if (m_dockWidget)
		{
			m_dockWidget->refreshPointCloudList();
		}
	});
	m_simplifyAction = m_host->registerAction(
		m_pointCloudMenu, QStringLiteral("Simplify Mesh"), [this]() { simplifyMeshOnSelection(); });
	m_smoothAction = m_host->registerAction(
		m_pointCloudMenu, QStringLiteral("Smooth Mesh"), [this]() { smoothMeshOnSelection(); });
}

void PointCloudPlugin::applyLanguage()
{
	if (!m_host)
	{
		return;
	}
	const bool zh = m_host->useChinese();
	if (m_dockWidget)
	{
		m_dockWidget->applyLanguage();
	}
	if (m_dockWidget)
	{
		m_host->setSidePanelTabTitle(m_dockWidget, zh ? "点云" : "Point Cloud");
	}
	if (m_pointCloudMenu)
	{
		m_pointCloudMenu->setTitle(zh ? QStringLiteral("点云") : QStringLiteral("Point Cloud"));
	}
	if (m_importAction)
	{
		m_importAction->setText(zh ? QStringLiteral("导入点云…") : QStringLiteral("Import Point Cloud..."));
	}
	if (m_downsampleAction)
	{
		m_downsampleAction->setText(zh ? QStringLiteral("体素下采样") : QStringLiteral("Downsample Voxel"));
	}
	if (m_reconstructAction)
	{
		m_reconstructAction->setText(
			zh ? QStringLiteral("Poisson Auto 重建") : QStringLiteral("Reconstruct Poisson Auto"));
	}
	if (m_exportMeshAction)
	{
		m_exportMeshAction->setText(zh ? QStringLiteral("导出网格 PLY…") : QStringLiteral("Export Mesh PLY..."));
	}
	if (m_refreshAction)
	{
		m_refreshAction->setText(zh ? QStringLiteral("刷新列表") : QStringLiteral("Refresh List"));
	}
	if (m_simplifyAction)
	{
		m_simplifyAction->setText(zh ? QStringLiteral("网格简化") : QStringLiteral("Simplify Mesh"));
	}
	if (m_smoothAction)
	{
		m_smoothAction->setText(zh ? QStringLiteral("网格平滑") : QStringLiteral("Smooth Mesh"));
	}
}

void PointCloudPlugin::importPointCloud()
{
	if (m_dockWidget)
	{
		m_dockWidget->triggerImport();
	}
}

void PointCloudPlugin::downsampleVoxelOnSelection()
{
	if (m_dockWidget)
	{
		m_dockWidget->triggerVoxelDownsample();
	}
}

void PointCloudPlugin::reconstructPoissonOnSelection()
{
	if (m_dockWidget)
	{
		m_dockWidget->triggerPoissonReconstruct();
	}
}

void PointCloudPlugin::exportMeshOnSelection()
{
	if (m_dockWidget)
	{
		m_dockWidget->triggerExportMesh();
	}
}

void PointCloudPlugin::simplifyMeshOnSelection()
{
	if (m_dockWidget)
	{
		m_dockWidget->triggerMeshSimplify();
	}
}

void PointCloudPlugin::smoothMeshOnSelection()
{
	if (m_dockWidget)
	{
		m_dockWidget->triggerMeshSmoothLaplacian();
	}
}
