/// @file ProcessFlowPlugin.cpp
/// @brief 工艺流程仿真插件实现

#include "ProcessFlowPlugin.h"

#include "BackendTypeIds.h"
#include "IPluginDocument.h"
#include "IPluginHostContext.h"
#include "IProcessFlowAiBridge.h"
#include "ProcessFlowAiBridge.h"
#include "ProcessFlowCanvasWidget.h"
#include "ProcessFlowJobSetPanel.h"
#include "ProcessFlowPageWidget.h"
#include "ProcessFlowPaletteWidget.h"
#include "ProcessFlowPropertyPanel.h"
#include "ProcessFlowReportPanel.h"
#include "ProcessFlowSimController.h"
#include "ProcessFlowSimSideWidget.h"

#include <QAction>
#include <QColor>
#include <QFile>
#include <QFileDialog>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLatin1String>
#include <QMenu>
#include <QMessageBox>
#include <QPointF>
#include <QTextStream>

ProcessFlowPlugin::~ProcessFlowPlugin() = default;

QString ProcessFlowPlugin::pluginId() const
{
	return QStringLiteral("com.cloudsim.processflow");
}

QString ProcessFlowPlugin::displayName() const
{
	return QStringLiteral("Process Flow");
}

bool ProcessFlowPlugin::initialize(IPluginHostContext* host)
{
	if (!host)
	{
		return false;
	}
	if (host->hostVersion() < 0x00012400U)
	{
		host->logError(QStringLiteral("ProcessFlowPlugin requires host 1.20.0+"));
		return false;
	}
	m_host = host;
	m_sim = new ProcessFlowSimController(this);
	m_sim->setHost(host);

	m_palette = new ProcessFlowPaletteWidget(nullptr);
	m_simSide = new ProcessFlowSimSideWidget(nullptr);
	connect(m_palette, &ProcessFlowPaletteWidget::addNodeRequested, this, &ProcessFlowPlugin::addNodeToActiveCanvas);
	if (ProcessFlowPropertyPanel* panel = m_palette->propertyPanel())
	{
		connect(panel, &ProcessFlowPropertyPanel::propsEdited, this,
				[this](int nodeId, const ProcessFlowNodeProps& props)
				{
					ProcessFlowPageWidget* page = ensurePageForActiveDocument();
					if (page && page->canvas())
					{
						page->canvas()->setNodeProps(nodeId, props);
					}
				});
	}
	bindSimUi();

	m_aiBridge = std::make_unique<ProcessFlowAiBridge>(this);
	host->setProcessFlowAiBridge(m_aiBridge.get());

	host->onActiveDocumentChanged(
		[this](IPluginDocument*)
		{
			if (!m_inProcessFlow || !m_host)
			{
				return;
			}
			if (m_host->currentWorkspaceMode() != pluginId())
			{
				softExitProcessFlow();
				return;
			}
			if (ProcessFlowPageWidget* page = ensurePageForActiveDocument())
			{
				m_host->setCentralAlternateWidget(page);
				m_host->showCentralAlternate();
				bindCanvasSelection(page);
			}
		});
	host->onWorkspaceModeClaimed(
		[this](const QString& modeId)
		{
			if (modeId == pluginId())
			{
				return;
			}
			softExitProcessFlow();
		});
	host->onLanguageChanged([this](bool) { applyLanguage(); });
	host->onProjectAboutToSave([this](const QString& documentId, QJsonObject& root)
							   { onProjectAboutToSave(documentId, root); });
	host->onProjectLoaded([this](const QString& documentId, const QJsonObject& root)
						  { onProjectLoaded(documentId, root); });

	host->registerWorkspaceMode(pluginId(), QStringLiteral("工艺流程"), QStringLiteral("Process Flow"),
								[this]() { enterProcessFlow(); });
	applyLanguage();
	host->logInfo(host->useChinese() ? QStringLiteral("工艺流程插件已加载。")
									 : QStringLiteral("Process Flow plugin initialized."));
	return true;
}

void ProcessFlowPlugin::bindSimUi()
{
	if (!m_simSide || !m_simSide->reportPanel() || !m_sim)
	{
		return;
	}
	ProcessFlowReportPanel* report = m_simSide->reportPanel();
	connect(report, &ProcessFlowReportPanel::runClicked, this, &ProcessFlowPlugin::runSimulation);
	connect(report, &ProcessFlowReportPanel::optimizeClicked, this,
			[this, report]()
			{
				if (!m_inProcessFlow || !m_sim)
					return;
				ProcessFlowPageWidget* page = ensurePageForActiveDocument();
				if (!page || !page->canvas())
					return;
				report->applyConfigTo(&m_sim->config());
				m_sim->optimizeThenStart(page->canvas());
			});
	connect(report, &ProcessFlowReportPanel::compareClicked, this,
			[this, report]()
			{
				if (!m_inProcessFlow || !m_sim)
					return;
				ProcessFlowPageWidget* page = ensurePageForActiveDocument();
				if (!page || !page->canvas())
					return;
				report->applyConfigTo(&m_sim->config());
				m_sim->compare(page->canvas(), report->comparePolicies());
			});
	connect(report, &ProcessFlowReportPanel::stopClicked, m_sim, &ProcessFlowSimController::stop);
	connect(report, &ProcessFlowReportPanel::exportJsonClicked, this, &ProcessFlowPlugin::exportSimJson);
	connect(report, &ProcessFlowReportPanel::exportCsvClicked, this, &ProcessFlowPlugin::exportSimCsv);
	connect(report, &ProcessFlowReportPanel::playbackLoadRequested, this,
			[this]()
			{
				ProcessFlowPageWidget* page = ensurePageForActiveDocument();
				if (page && page->canvas() && m_sim)
					page->canvas()->setPlaybackTrace(m_sim->lastResult());
			});
	connect(report, &ProcessFlowReportPanel::playbackTimeChanged, this,
			[this](double t)
			{
				ProcessFlowPageWidget* page = ensurePageForActiveDocument();
				if (page && page->canvas())
					page->canvas()->setPlaybackTime(t);
			});
	connect(m_sim, &ProcessFlowSimController::started, this, [report]() { report->setRunning(true); });
	connect(m_sim, &ProcessFlowSimController::resultReady, this,
			[this, report](const SimStatistics& stats)
			{
				report->setStatistics(stats);
				ProcessFlowPageWidget* page = ensurePageForActiveDocument();
				if (page && page->canvas())
					page->canvas()->setPlaybackTrace(stats);
			});
	connect(m_sim, &ProcessFlowSimController::compareReady, this,
			[report](const QVector<PolicyCompareRow>& rows, const QVector<SimStatistics>& perPolicy)
			{ report->setCompareResult(rows, perPolicy); });
	connect(m_sim, &ProcessFlowSimController::finished, this,
			[this, report](bool ok, const QString& message)
			{
				report->setRunning(false);
				if (!m_host)
					return;
				if (ok)
					m_host->logInfo(m_host->useChinese() ? QStringLiteral("仿真完成。")
														 : QStringLiteral("Simulation finished."));
				else
					m_host->logWarn(message);
			});

	if (ProcessFlowJobSetPanel* js = m_simSide->jobSetPanel())
	{
		connect(js, &ProcessFlowJobSetPanel::jobSetChanged, this,
				[this, js]()
				{
					ProcessFlowPageWidget* page = ensurePageForActiveDocument();
					if (page && page->canvas())
						page->canvas()->setJobSetJson(js->toJson());
					if (m_host)
						m_host->markActiveDocumentModified();
					m_flowDirty = true;
				});
	}
}

void ProcessFlowPlugin::runSimulation()
{
	if (!m_inProcessFlow || !m_sim || !m_simSide || !m_simSide->reportPanel())
	{
		if (m_host)
		{
			m_host->logWarn(m_host->useChinese() ? QStringLiteral("请先进入工艺流程。")
												 : QStringLiteral("Enter process flow first."));
		}
		return;
	}
	ProcessFlowPageWidget* page = ensurePageForActiveDocument();
	if (!page || !page->canvas())
	{
		return;
	}
	ProcessFlowReportPanel* report = m_simSide->reportPanel();
	report->applyConfigTo(&m_sim->config());
	m_sim->start(page->canvas());
}

void ProcessFlowPlugin::exportSimJson()
{
	if (!m_sim || !m_host)
	{
		return;
	}
	const QString path = QFileDialog::getSaveFileName(nullptr, QStringLiteral("Export JSON"), QString(),
													  QStringLiteral("JSON (*.json)"));
	if (path.isEmpty())
	{
		return;
	}
	QFile f(path);
	if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate))
	{
		m_host->logError(QStringLiteral("cannot write file"));
		return;
	}
	QJsonObject root;
	root.insert(QStringLiteral("statistics"), m_sim->lastResult().toJson());
	root.insert(QStringLiteral("operationTrace"), m_sim->lastResult().trace.toJsonArray());
	f.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
}

void ProcessFlowPlugin::exportSimCsv()
{
	if (!m_sim || !m_host)
	{
		return;
	}
	const QString path = QFileDialog::getSaveFileName(nullptr, QStringLiteral("Export CSV"), QString(),
													  QStringLiteral("CSV (*.csv)"));
	if (path.isEmpty())
	{
		return;
	}
	QFile f(path);
	if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text))
	{
		m_host->logError(QStringLiteral("cannot write file"));
		return;
	}
	QTextStream ts(&f);
	ts.setCodec("UTF-8");
	ts << m_sim->lastResult().toCsv();
}

void ProcessFlowPlugin::shutdown()
{
	if (m_host)
	{
		m_host->setProcessFlowAiBridge(nullptr);
	}
	m_aiBridge.reset();
	if (m_sim)
	{
		m_sim->stop();
	}
	// 关窗路径禁止走 exitProcessFlow（会弹框 + returnToMainWorkspace 广播）
	softExitProcessFlow();
	if (m_host && m_host->currentWorkspaceMode() == pluginId())
	{
		m_host->claimWorkspaceMode(QString());
	}
	m_pagesByDocId.clear();
	if (m_palette)
	{
		delete m_palette.data();
		m_palette = nullptr;
	}
	if (m_simSide)
	{
		delete m_simSide.data();
		m_simSide = nullptr;
	}
	m_host = nullptr;
	m_menu = nullptr;
	m_enterAction = nullptr;
	m_exitAction = nullptr;
	m_runAction = nullptr;
	m_stopAction = nullptr;
}

void ProcessFlowPlugin::registerMenus()
{
	// 模式切换改由宿主顶栏分段 / 设置→模式切换；仿真启停在工艺流程侧栏
}

void ProcessFlowPlugin::applyLanguage()
{
	if (!m_host)
	{
		return;
	}
	const bool zh = m_host->useChinese();
	if (m_palette)
	{
		m_palette->applyLanguage(zh);
	}
	if (m_simSide)
	{
		m_simSide->applyLanguage(zh);
	}
	for (auto it = m_pagesByDocId.begin(); it != m_pagesByDocId.end(); ++it)
	{
		if (it.value())
		{
			it.value()->applyLanguage(zh);
		}
	}
}

void ProcessFlowPlugin::enterProcessFlow()
{
	if (!m_host)
	{
		return;
	}
	if (!m_host->activeDocument())
	{
		m_host->logWarn(m_host->useChinese() ? QStringLiteral("无活动文档，无法进入工艺流程。")
											 : QStringLiteral("No active document."));
		return;
	}
	ProcessFlowPageWidget* page = ensurePageForActiveDocument();
	if (!page)
	{
		return;
	}
	m_host->claimWorkspaceMode(pluginId());
	m_host->setModeToolBar(nullptr);
	m_host->setCentralAlternateWidget(page);
	m_host->enterAlternateSideUi(m_palette, m_simSide);
	m_host->showCentralAlternate();
	bindCanvasSelection(page);
	if (m_simSide && m_simSide->jobSetPanel())
	{
		m_simSide->jobSetPanel()->setCanvas(page->canvas());
		m_simSide->jobSetPanel()->loadFromJson(page->canvas()->jobSetJson());
	}
	m_inProcessFlow = true;
}

void ProcessFlowPlugin::softExitProcessFlow()
{
	if (!m_inProcessFlow)
	{
		return;
	}
	if (m_sim && m_sim->isRunning())
	{
		m_sim->stop();
	}
	m_inProcessFlow = false;
	if (m_palette && m_palette->propertyPanel())
	{
		m_palette->propertyPanel()->clearSelection();
	}
	if (!m_host)
	{
		return;
	}
	m_host->setModeToolBar(nullptr);
	m_host->setCentralAlternateWidget(nullptr);
	m_host->exitAlternateSideUi();
}

void ProcessFlowPlugin::exitProcessFlow()
{
	if (!m_host)
	{
		return;
	}
	if (m_flowDirty || m_host->isActiveDocumentModified())
	{
		const QMessageBox::StandardButton btn = QMessageBox::question(
			nullptr, m_host->useChinese() ? QStringLiteral("未保存的工艺流程") : QStringLiteral("Unsaved process flow"),
			m_host->useChinese() ? QStringLiteral("流程图已修改。退出前请先保存工程，或选择放弃修改。\n是否仍要退出？")
								 : QStringLiteral("Flow graph changed. Exit anyway (save project first to keep changes)?"),
			QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
		if (btn != QMessageBox::Yes)
			return;
	}
	if (m_sim && m_sim->isRunning())
	{
		m_sim->stop();
	}
	m_inProcessFlow = false;
	m_flowDirty = false;
	m_host->returnToMainWorkspace();
	if (m_palette && m_palette->propertyPanel())
	{
		m_palette->propertyPanel()->clearSelection();
	}
}

ProcessFlowPageWidget* ProcessFlowPlugin::ensurePageForDocument(const QString& documentId)
{
	if (documentId.isEmpty())
	{
		return nullptr;
	}
	QPointer<ProcessFlowPageWidget>& slot = m_pagesByDocId[documentId];
	if (!slot)
	{
		slot = new ProcessFlowPageWidget(nullptr);
		if (m_host)
		{
			slot->applyLanguage(m_host->useChinese());
		}
		bindCanvasSelection(slot.data());
		if (slot->canvas() && m_sim)
		{
			connect(slot->canvas(), &ProcessFlowCanvasWidget::graphChanged, this,
					[this](int, int)
					{
						m_flowDirty = true;
						if (m_host)
							m_host->markActiveDocumentModified();
						if (!m_sim)
						{
							return;
						}
						if (m_sim->isRunning())
						{
							m_sim->stop();
						}
						m_sim->clearResult();
						if (m_simSide && m_simSide->reportPanel())
						{
							m_simSide->reportPanel()->clearStatistics();
						}
					});
		}
	}
	return slot.data();
}

ProcessFlowPageWidget* ProcessFlowPlugin::ensurePageForActiveDocument()
{
	if (!m_host)
	{
		return nullptr;
	}
	IPluginDocument* doc = m_host->activeDocument();
	if (!doc)
	{
		return nullptr;
	}
	return ensurePageForDocument(QString::fromStdString(doc->documentId()));
}

void ProcessFlowPlugin::bindCanvasSelection(ProcessFlowPageWidget* page)
{
	if (!page || !page->canvas() || !m_palette || !m_palette->propertyPanel())
	{
		return;
	}
	ProcessFlowCanvasWidget* canvas = page->canvas();
	ProcessFlowPropertyPanel* panel = m_palette->propertyPanel();
	disconnect(canvas, &ProcessFlowCanvasWidget::nodeSelected, panel, nullptr);
	connect(canvas, &ProcessFlowCanvasWidget::nodeSelected, panel,
			[canvas, panel](int id, const QString&)
			{
				if (id < 0)
				{
					panel->clearSelection();
					return;
				}
				ProcessFlowNodeProps props;
				if (canvas->nodeProps(id, &props))
				{
					panel->setNodeProps(id, props);
				}
			});
	const int sel = canvas->selectedNodeId();
	if (sel >= 0)
	{
		ProcessFlowNodeProps props;
		if (canvas->nodeProps(sel, &props))
		{
			panel->setNodeProps(sel, props);
		}
	}
	else
	{
		panel->clearSelection();
	}
}

void ProcessFlowPlugin::addNodeToActiveCanvas(const QString& kind, const QString& title, const QString& subtitle,
											 const QColor& color)
{
	if (!m_inProcessFlow)
	{
		return;
	}
	ProcessFlowPageWidget* page = ensurePageForActiveDocument();
	if (!page || !page->canvas())
	{
		return;
	}
	ProcessFlowCanvasWidget* canvas = page->canvas();
	const QPointF pos(40.0 + canvas->nodeCount() * 36.0, 40.0 + canvas->nodeCount() * 24.0);
	canvas->addNode(title, subtitle, color, pos, kind);
	bindCanvasSelection(page);
}

void ProcessFlowPlugin::onProjectAboutToSave(const QString& documentId, QJsonObject& root)
{
	auto it = m_pagesByDocId.constFind(documentId);
	if (it == m_pagesByDocId.cend() || !it.value() || !it.value()->canvas())
	{
		return;
	}
	ProcessFlowCanvasWidget* canvas = it.value()->canvas();
	if (canvas->nodeCount() <= 0 && canvas->edgeCount() <= 0)
	{
		return;
	}
	root.insert(QLatin1String(backend_type::kProjectKeyProcessFlow), canvas->toJson());
	m_flowDirty = false;
}

void ProcessFlowPlugin::onProjectLoaded(const QString& documentId, const QJsonObject& root)
{
	if (!root.contains(QLatin1String(backend_type::kProjectKeyProcessFlow)))
	{
		return;
	}
	const QJsonObject flow = root.value(QLatin1String(backend_type::kProjectKeyProcessFlow)).toObject();
	ProcessFlowPageWidget* page = ensurePageForDocument(documentId);
	if (!page || !page->canvas())
	{
		return;
	}
	page->canvas()->fromJson(flow);
	if (m_simSide && m_simSide->jobSetPanel())
	{
		m_simSide->jobSetPanel()->setCanvas(page->canvas());
		m_simSide->jobSetPanel()->loadFromJson(page->canvas()->jobSetJson());
	}
}

bool ProcessFlowPlugin::ensureProcessFlowForAi(QString* outError)
{
	if (!m_host)
	{
		if (outError)
			*outError = QStringLiteral("宿主不可用。");
		return false;
	}
	if (!m_host->activeDocument())
	{
		if (outError)
			*outError = QStringLiteral("无活动文档，无法进入工艺流程。");
		return false;
	}
	if (!m_inProcessFlow)
		enterProcessFlow();
	if (!m_inProcessFlow)
	{
		if (outError)
			*outError = QStringLiteral("进入工艺流程失败。");
		return false;
	}
	return true;
}

ProcessFlowCanvasWidget* ProcessFlowPlugin::activeCanvasForAi() const
{
	if (!m_host || !m_host->activeDocument())
		return nullptr;
	const QString docId = QString::fromStdString(m_host->activeDocument()->documentId());
	const auto it = m_pagesByDocId.constFind(docId);
	if (it == m_pagesByDocId.cend() || !it.value())
		return nullptr;
	return it.value()->canvas();
}

void ProcessFlowPlugin::syncJobSetPanelFromCanvas()
{
	ProcessFlowCanvasWidget* canvas = activeCanvasForAi();
	if (!canvas || !m_simSide || !m_simSide->jobSetPanel())
		return;
	m_simSide->jobSetPanel()->setCanvas(canvas);
	m_simSide->jobSetPanel()->loadFromJson(canvas->jobSetJson());
}

void ProcessFlowPlugin::applyAiSimStatistics(const SimStatistics& stats)
{
	if (m_sim)
	{
		m_sim->clearResult();
		// 同步结果供导出；不经异步 controller
		m_sim->config().policy = QStringLiteral("fifo");
	}
	if (m_simSide && m_simSide->reportPanel())
		m_simSide->reportPanel()->setStatistics(stats);
	if (ProcessFlowCanvasWidget* canvas = activeCanvasForAi())
		canvas->setPlaybackTrace(stats);
}

void ProcessFlowPlugin::applyAiCompareRows(const QVector<PolicyCompareRow>& rows)
{
	if (m_simSide && m_simSide->reportPanel())
		m_simSide->reportPanel()->setCompareRows(rows);
}
