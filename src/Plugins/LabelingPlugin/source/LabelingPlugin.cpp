/// @file LabelingPlugin.cpp
/// @brief LabelingPlugin 实现

#include "LabelingPlugin.h"

#include "CloudSimPluginVersion.h"
#include "IPluginDocument.h"
#include "IPluginDocumentContext.h"
#include "IPluginHostContext.h"
#include "IPluginLabelingHost.h"
#include "IPluginUiContext.h"
#include "LabelingAnnotWidget.h"
#include "LabelingTrainWidget.h"

#include <QAction>
#include <QMenu>
#include <QTabWidget>

QString LabelingPlugin::pluginId() const
{
	return QStringLiteral("com.cloudsim.labeling");
}

QString LabelingPlugin::displayName() const
{
	return QStringLiteral("Labeling");
}

bool LabelingPlugin::initialize(IPluginHostContext* host)
{
	if (!host)
	{
		return false;
	}
	if (host->hostVersion() < 0x00013800U)
	{
		host->logError(host->useChinese() ? QStringLiteral("分割标注插件需要宿主 1.56.0+")
										  : QStringLiteral("LabelingPlugin requires host 1.56.0+"));
		return false;
	}
	IPluginUiContext* uiCtx = host->uiContext();
	IPluginDocumentContext* docCtx = host->documentContext();
	if (!uiCtx || !docCtx)
	{
		host->logError(QStringLiteral("LabelingPlugin: narrow context unavailable"));
		return false;
	}
	// labelingHost 暂无独立窄上下文，仍走聚合
	if (!host->labelingHost())
	{
		host->logError(host->useChinese() ? QStringLiteral("标注宿主 API 不可用")
										  : QStringLiteral("Labeling host API unavailable"));
		return false;
	}
	if (!uiCtx->sidePanelTabParent())
	{
		return false;
	}

	m_host = host;
	m_tabWidget = new QTabWidget(nullptr);
	m_annotWidget = new LabelingAnnotWidget(host, m_tabWidget);
	m_trainWidget = new LabelingTrainWidget(host, m_tabWidget);
	m_tabWidget->addTab(m_annotWidget, host->useChinese() ? QStringLiteral("分割标注") : QStringLiteral("Annotation"));
	m_tabWidget->addTab(m_trainWidget, host->useChinese() ? QStringLiteral("模型训练") : QStringLiteral("Training"));

	const char* panelTitle = host->useChinese() ? "分割标注" : "Labeling";
	if (uiCtx->registerSidePanelTab(panelTitle, m_tabWidget) < 0)
	{
		return false;
	}

	QObject::connect(m_annotWidget, &LabelingAnnotWidget::datasetExported, m_trainWidget,
					 &LabelingTrainWidget::setDatasetRoot);

	docCtx->onActiveDocumentChanged(
		[this](IPluginDocument*)
		{
			if (m_annotWidget)
			{
				m_annotWidget->refreshBackendList();
			}
		});

	host->onLanguageChanged([this](const bool) { applyLanguage(); });

	registerMenus();
	applyLanguage();
	host->logInfo(host->useChinese() ? QStringLiteral("分割标注插件已加载。")
									 : QStringLiteral("Labeling plugin initialized."));
	return true;
}

void LabelingPlugin::shutdown()
{
	if (m_host && m_tabWidget)
	{
		if (IPluginUiContext* uiCtx = m_host->uiContext())
		{
			uiCtx->unregisterSidePanelTab(m_tabWidget);
		}
	}
	m_annotWidget = nullptr;
	m_trainWidget = nullptr;
	m_tabWidget = nullptr;
	m_host = nullptr;
	m_labelingMenu = nullptr;
	m_openAnnotAction = nullptr;
	m_openTrainAction = nullptr;
}

void LabelingPlugin::registerMenus()
{
	if (!m_host)
	{
		return;
	}
	IPluginUiContext* uiCtx = m_host->uiContext();
	if (!uiCtx)
	{
		return;
	}
	m_labelingMenu = uiCtx->registerMenuPath({QStringLiteral("Tools"), QStringLiteral("Labeling")});
	if (!m_labelingMenu)
	{
		return;
	}
	m_openAnnotAction = uiCtx->registerAction(m_labelingMenu, QStringLiteral("Open Annotation Tab"),
											  [this]()
											  {
												  if (m_tabWidget)
												  {
													  m_tabWidget->setCurrentIndex(0);
												  }
											  });
	m_openTrainAction = uiCtx->registerAction(m_labelingMenu, QStringLiteral("Open Training Tab"),
											  [this]()
											  {
												  if (m_tabWidget)
												  {
													  m_tabWidget->setCurrentIndex(1);
												  }
											  });
}

void LabelingPlugin::applyLanguage()
{
	if (!m_host || !m_tabWidget)
	{
		return;
	}
	const bool zh = m_host->useChinese();
	m_tabWidget->setTabText(0, zh ? QStringLiteral("分割标注") : QStringLiteral("Annotation"));
	m_tabWidget->setTabText(1, zh ? QStringLiteral("模型训练") : QStringLiteral("Training"));
	if (IPluginUiContext* uiCtx = m_host->uiContext())
	{
		uiCtx->setSidePanelTabTitle(m_tabWidget, zh ? "分割标注" : "Labeling");
	}
	if (m_annotWidget)
	{
		m_annotWidget->applyLanguage();
	}
	if (m_trainWidget)
	{
		m_trainWidget->applyLanguage();
	}
	if (m_labelingMenu)
	{
		m_labelingMenu->setTitle(zh ? QStringLiteral("分割标注") : QStringLiteral("Labeling"));
	}
	if (m_openAnnotAction)
	{
		m_openAnnotAction->setText(zh ? QStringLiteral("打开标注页") : QStringLiteral("Open Annotation Tab"));
	}
	if (m_openTrainAction)
	{
		m_openTrainAction->setText(zh ? QStringLiteral("打开训练页") : QStringLiteral("Open Training Tab"));
	}
}
