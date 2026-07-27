/// @file EngineeringDrawingPlugin.cpp
/// @brief 工程图插件二期：轴测/剖视/第三角/标注/拖视图/导出

#include "EngineeringDrawingPlugin.h"

#include "BackendTypeIds.h"
#include "DrawingPageWidget.h"
#include "DrawingRibbonBar.h"
#include "DrawingSheetCanvasWidget.h"
#include "DrawingSidePanel.h"
#include "IPluginDocument.h"
#include "IPluginGeometryHost.h"
#include "IPluginHostContext.h"
#include "PluginGeometryTypes.h"

#include <QAction>
#include <QFileDialog>
#include <QJsonObject>
#include <QLatin1String>
#include <QMenu>
#include <QMessageBox>
#include <QPushButton>
#include <QStringList>

namespace
{
QVector<DrawingSheetCanvasWidget::Polyline2d> toPolylinesFlippedY(const std::vector<std::vector<float>>& xys)
{
	QVector<DrawingSheetCanvasWidget::Polyline2d> out;
	out.reserve(static_cast<int>(xys.size()));
	for (const std::vector<float>& xy : xys)
	{
		DrawingSheetCanvasWidget::Polyline2d poly;
		poly.points.reserve(static_cast<int>(xy.size() / 2));
		for (std::size_t i = 0; i + 1 < xy.size(); i += 2)
			poly.points.push_back(QPointF(xy[i], -xy[i + 1]));
		if (poly.points.size() >= 2)
			out.push_back(poly);
	}
	return out;
}
} // namespace

EngineeringDrawingPlugin::~EngineeringDrawingPlugin() = default;

QString EngineeringDrawingPlugin::pluginId() const
{
	return QStringLiteral("com.cloudsim.drawing");
}

QString EngineeringDrawingPlugin::displayName() const
{
	return QStringLiteral("Engineering Drawing");
}

bool EngineeringDrawingPlugin::initialize(IPluginHostContext* host)
{
	if (!host)
		return false;
	if (host->hostVersion() < 0x00012200U)
	{
		host->logError(QStringLiteral("EngineeringDrawingPlugin requires host 1.34.0+"));
		return false;
	}
	m_host = host;
	m_side = new DrawingSidePanel(nullptr);
	QObject::connect(m_side, &DrawingSidePanel::selectionChanged, this, [this](const QString& backendId) {
		if (!m_inDrawing || backendId.isEmpty())
			return;
		refreshViewPreviews(backendId);
	});
	QObject::connect(m_side, &DrawingSidePanel::viewTemplateActivated, this, [this](const QString& kind) {
		DrawingPageWidget* page = ensurePageForActiveDocument();
		if (!page || !page->canvas())
			return;
		QPointF pos(40, 40);
		if (!page->canvas()->views().isEmpty())
		{
			const QRectF last = page->canvas()->views().constLast().frame;
			pos = QPointF(last.right() + 40.0, last.top());
		}
		page->canvas()->addCatalogViewAt(kind, pos);
	});

	host->onActiveDocumentChanged(
		[this](IPluginDocument*)
		{
			if (!m_inDrawing || !m_host)
				return;
			if (m_host->currentWorkspaceMode() != pluginId())
			{
				softExitDrawing();
				return;
			}
			if (DrawingPageWidget* page = ensurePageForActiveDocument())
			{
				m_host->setCentralAlternateWidget(page);
				m_host->showCentralAlternate();
				bindPage(page);
				refreshBackendList();
			}
		});
	host->onWorkspaceModeClaimed(
		[this](const QString& modeId)
		{
			if (modeId == pluginId())
				return;
			softExitDrawing();
		});
	host->onLanguageChanged([this](bool) { applyLanguage(); });
	host->onProjectAboutToSave([this](const QString& documentId, QJsonObject& root)
							   { onProjectAboutToSave(documentId, root); });
	host->onProjectLoaded([this](const QString& documentId, const QJsonObject& root)
						  { onProjectLoaded(documentId, root); });

	host->registerWorkspaceMode(pluginId(), QStringLiteral("工程图"), QStringLiteral("Drawing"),
								[this]() { enterDrawing(); });
	applyLanguage();
	host->logInfo(host->useChinese() ? QStringLiteral("工程图插件已加载。")
									 : QStringLiteral("Engineering Drawing plugin initialized."));
	return true;
}

void EngineeringDrawingPlugin::shutdown()
{
	softExitDrawing();
	if (m_host)
	{
		if (m_host->currentWorkspaceMode() == pluginId())
			m_host->claimWorkspaceMode(QString());
		m_host->setModeToolBar(nullptr);
		m_host->setCentralAlternateWidget(nullptr);
	}
	m_pagesByDocId.clear();
	delete m_ribbon;
	m_ribbon = nullptr;
	delete m_side;
	m_side = nullptr;
	m_host = nullptr;
}

void EngineeringDrawingPlugin::registerMenus()
{
	// 模式切换改由宿主顶栏分段 / 设置→模式切换，不再注册顶层菜单
}

void EngineeringDrawingPlugin::applyLanguage()
{
	if (!m_host)
		return;
	const bool zh = m_host->useChinese();
	if (m_side)
		m_side->applyLanguage(zh);
	if (m_ribbon)
		m_ribbon->applyLanguage(zh);
	if (m_inDrawing && m_host && m_side)
		m_host->enterAlternateSideUi(m_side, nullptr);
	for (auto it = m_pagesByDocId.begin(); it != m_pagesByDocId.end(); ++it)
	{
		if (it.value())
			it.value()->applyLanguage(zh);
	}
}

void EngineeringDrawingPlugin::ensureRibbon()
{
	if (m_ribbon)
		return;
	m_ribbon = new DrawingRibbonBar(nullptr);
	m_ribbon->applyLanguage(m_host && m_host->useChinese());
	QObject::connect(m_ribbon, &DrawingRibbonBar::toolRequested, this,
					 &EngineeringDrawingPlugin::applyToolToActiveCanvas);
}

void EngineeringDrawingPlugin::applyToolToActiveCanvas(DrawingCanvasTool tool)
{
	DrawingPageWidget* page = ensurePageForActiveDocument();
	if (page && page->canvas())
		page->canvas()->setTool(tool);
}

void EngineeringDrawingPlugin::enterDrawing()
{
	if (!m_host)
		return;
	if (!m_host->activeDocument())
	{
		m_host->logWarn(m_host->useChinese() ? QStringLiteral("请先打开文档。") : QStringLiteral("Open a document first."));
		return;
	}
	DrawingPageWidget* page = ensurePageForActiveDocument();
	if (!page)
		return;
	ensureRibbon();
	m_host->claimWorkspaceMode(pluginId());
	m_host->setModeToolBar(m_ribbon);
	m_host->setCentralAlternateWidget(page);
	// 右侧只保留宿主 AI 助手，不挂图纸说明面板
	m_host->enterAlternateSideUi(m_side, nullptr);
	m_host->showCentralAlternate();
	bindPage(page);
	refreshBackendList();
	m_inDrawing = true;
}

void EngineeringDrawingPlugin::exitDrawing()
{
	if (!m_host)
		return;
	m_inDrawing = false;
	m_host->returnToMainWorkspace();
}

void EngineeringDrawingPlugin::softExitDrawing()
{
	// 卸掉本模式 Ribbon/中央页/侧栏，避免切到几何等模式时残留工程图 UI
	if (!m_inDrawing)
		return;
	m_inDrawing = false;
	if (!m_host)
		return;
	m_host->setModeToolBar(nullptr);
	m_host->setCentralAlternateWidget(nullptr);
	m_host->exitAlternateSideUi();
}

DrawingPageWidget* EngineeringDrawingPlugin::ensurePageForDocument(const QString& documentId)
{
	if (documentId.isEmpty())
		return nullptr;
	auto it = m_pagesByDocId.find(documentId);
	if (it != m_pagesByDocId.end() && it.value())
		return it.value();
	auto* page = new DrawingPageWidget(nullptr);
	page->applyLanguage(m_host && m_host->useChinese());
	m_pagesByDocId.insert(documentId, page);
	bindPage(page);
	return page;
}

DrawingPageWidget* EngineeringDrawingPlugin::ensurePageForActiveDocument()
{
	if (!m_host || !m_host->activeDocument())
		return nullptr;
	return ensurePageForDocument(QString::fromStdString(m_host->activeDocument()->documentId()));
}

void EngineeringDrawingPlugin::bindPage(DrawingPageWidget* page)
{
	if (!page)
		return;
	QObject::disconnect(page, nullptr, this, nullptr);
	QObject::connect(page, &DrawingPageWidget::generateRequested, this, &EngineeringDrawingPlugin::generateViews);
	QObject::connect(page, &DrawingPageWidget::exportSvgRequested, this, [this, page]() {
		if (!page->canvas())
			return;
		const QString path = QFileDialog::getSaveFileName(page, QStringLiteral("导出 SVG"), QString(),
														  QStringLiteral("SVG (*.svg)"));
		if (path.isEmpty())
			return;
		if (!page->canvas()->exportSvg(path))
			QMessageBox::warning(page, QStringLiteral("导出失败"), QStringLiteral("无法写入 SVG。"));
	});
	QObject::connect(page, &DrawingPageWidget::exportDxfRequested, this, [this, page]() {
		if (!page->canvas())
			return;
		const QString path = QFileDialog::getSaveFileName(page, QStringLiteral("导出 DXF"), QString(),
														  QStringLiteral("DXF (*.dxf)"));
		if (path.isEmpty())
			return;
		if (!page->canvas()->exportDxf(path))
			QMessageBox::warning(page, QStringLiteral("导出失败"), QStringLiteral("无法写入 DXF。"));
	});
}

void EngineeringDrawingPlugin::refreshBackendList()
{
	if (!m_host || !m_side)
		return;
	IPluginGeometryHost* geo = m_host->geometryHost();
	IPluginDocument* doc = m_host->activeDocument();
	QStringList names;
	QStringList ids;
	if (geo && doc)
	{
		std::vector<PluginGeometryBackendEntry> backends;
		QString err;
		if (geo->listComputableBackends(doc, backends, &err))
		{
			for (const PluginGeometryBackendEntry& e : backends)
			{
				names.append(QString::fromStdString(e.displayName.empty() ? e.backendId : e.displayName));
				ids.append(QString::fromStdString(e.backendId));
			}
		}
		else if (!err.isEmpty())
		{
			m_host->logWarn(err);
		}
	}
	m_side->setBackends(names, ids);
	if (!ids.isEmpty())
		refreshViewPreviews(ids.first());
}

void EngineeringDrawingPlugin::applyHlrResultToUi(DrawingPageWidget* page, const QString& backendId, bool thirdAngle,
												 bool layoutSheet, const PluginDrawingHlrResult& result)
{
	QVector<DrawingSheetCanvasWidget::Polyline2d> frontVis, frontHid, topVis, topHid, rightVis, rightHid;
	QVector<DrawingSheetCanvasWidget::Polyline2d> isoVis, isoHid, sectionVis, sectionHid;
	bool hasIso = false;
	bool hasSection = false;
	for (const PluginDrawingHlrViewResult& v : result.views)
	{
		const QString id = QString::fromStdString(v.viewId);
		if (id == QLatin1String("front"))
		{
			frontVis = toPolylinesFlippedY(v.visibleXy);
			frontHid = toPolylinesFlippedY(v.hiddenXy);
		}
		else if (id == QLatin1String("top"))
		{
			topVis = toPolylinesFlippedY(v.visibleXy);
			topHid = toPolylinesFlippedY(v.hiddenXy);
		}
		else if (id == QLatin1String("right"))
		{
			rightVis = toPolylinesFlippedY(v.visibleXy);
			rightHid = toPolylinesFlippedY(v.hiddenXy);
		}
		else if (id == QLatin1String("iso"))
		{
			isoVis = toPolylinesFlippedY(v.visibleXy);
			isoHid = toPolylinesFlippedY(v.hiddenXy);
			hasIso = true;
		}
		else if (id == QLatin1String("section"))
		{
			sectionVis = toPolylinesFlippedY(v.visibleXy);
			sectionHid = toPolylinesFlippedY(v.hiddenXy);
			hasSection = true;
		}
	}

	const bool zh = m_host && m_host->useChinese();
	QVector<DrawingViewTemplate> templates;
	auto pushT = [&](const char* kind, const QString& titleZh, const QString& titleEn,
					 const QVector<DrawingSheetCanvasWidget::Polyline2d>& vis,
					 const QVector<DrawingSheetCanvasWidget::Polyline2d>& hid) {
		DrawingViewTemplate t;
		t.kind = QString::fromLatin1(kind);
		t.title = zh ? titleZh : titleEn;
		t.visible = vis;
		t.hidden = hid;
		if (!vis.isEmpty() || !hid.isEmpty())
			t.thumbnail = renderDrawingViewThumbnail(vis, hid, QSize(148, 108));
		templates.push_back(t);
	};
	pushT("front", QStringLiteral("正视图"), QStringLiteral("Front"), frontVis, frontHid);
	pushT("top", QStringLiteral("俯视图"), QStringLiteral("Top"), topVis, topHid);
	pushT("right", QStringLiteral("右视图"), QStringLiteral("Right"), rightVis, rightHid);
	pushT("iso", QStringLiteral("轴测图"), QStringLiteral("Iso"), isoVis, isoHid);
	if (hasSection)
		pushT("section", QStringLiteral("剖视图"), QStringLiteral("Section"), sectionVis, sectionHid);

	if (m_side)
		m_side->setViewTemplates(templates);
	if (page && page->canvas())
	{
		page->canvas()->setViewCatalog(templates);
		page->canvas()->setBackendId(backendId);
	}

	if (!layoutSheet || !page || !page->canvas())
		return;

	const DrawingProjectionMethod method =
		thirdAngle ? DrawingProjectionMethod::ThirdAngle : DrawingProjectionMethod::FirstAngle;
	const QVector<DrawingSheetCanvasWidget::DrawingView> views = layoutEngineeringViews(
		method, hasIso, hasSection, frontVis, frontHid, topVis, topHid, rightVis, rightHid, isoVis, isoHid, sectionVis,
		sectionHid);
	int polyCount = 0;
	for (const auto& v : views)
		polyCount += v.visible.size() + v.hidden.size();
	page->canvas()->setProjectionMethod(method);
	page->canvas()->setViews(views);
	if (m_host)
	{
		m_host->logInfo(zh ? QStringLiteral("工程图已生成：%1 视图，%2 条折线。").arg(views.size()).arg(polyCount)
						   : QStringLiteral("Drawing generated: %1 views, %2 polylines.")
								 .arg(views.size())
								 .arg(polyCount));
	}
}

void EngineeringDrawingPlugin::refreshViewPreviews(const QString& backendId)
{
	if (!m_host || backendId.isEmpty())
		return;
	IPluginGeometryHost* geo = m_host->geometryHost();
	IPluginDocument* doc = m_host->activeDocument();
	DrawingPageWidget* page = ensurePageForActiveDocument();
	if (!geo || !doc || !page)
		return;

	PluginDrawingProjectParams params;
	params.thirdAngle = page->thirdAngle();
	params.includeIso = true;
	params.includeSection = false;
	geo->projectBrepToEngineeringDrawing(
		doc, backendId.toStdString(), params,
		[this, page, backendId, params](bool ok, const QString& error, const PluginDrawingHlrResult& result) {
			if (!ok)
			{
				if (m_host)
					m_host->logWarn(error.isEmpty() ? QStringLiteral("视角预览失败") : error);
				return;
			}
			applyHlrResultToUi(page, backendId, params.thirdAngle, false, result);
		});
}

void EngineeringDrawingPlugin::generateViews()
{
	if (!m_host)
		return;
	IPluginGeometryHost* geo = m_host->geometryHost();
	IPluginDocument* doc = m_host->activeDocument();
	DrawingPageWidget* page = ensurePageForActiveDocument();
	if (!geo || !doc || !page || !page->canvas())
	{
		m_host->logWarn(QStringLiteral("Cannot generate drawing."));
		return;
	}
	QString backendId = m_side ? m_side->selectedBackendId() : QString();
	if (backendId.isEmpty())
	{
		m_host->logWarn(m_host->useChinese() ? QStringLiteral("请选择模型。") : QStringLiteral("Select a model."));
		return;
	}

	PluginDrawingProjectParams params;
	params.thirdAngle = page->thirdAngle();
	params.includeIso = page->includeIso();
	params.includeSection = page->includeSection();
	params.sectionPlane = page->sectionPlane();

	page->generateButton()->setEnabled(false);
	geo->projectBrepToEngineeringDrawing(
		doc, backendId.toStdString(), params,
		[this, page, backendId, params](bool ok, const QString& error, const PluginDrawingHlrResult& result) {
			if (page && page->generateButton())
				page->generateButton()->setEnabled(true);
			if (!ok)
			{
				if (m_host)
					m_host->logError(error.isEmpty() ? QStringLiteral("Drawing projection failed") : error);
				return;
			}
			applyHlrResultToUi(page, backendId, params.thirdAngle, true, result);
		});
}

void EngineeringDrawingPlugin::onProjectAboutToSave(const QString& documentId, QJsonObject& root)
{
	auto it = m_pagesByDocId.constFind(documentId);
	if (it == m_pagesByDocId.cend() || !it.value() || !it.value()->canvas())
		return;
	DrawingSheetCanvasWidget* canvas = it.value()->canvas();
	if (canvas->isEmpty())
		return;
	root.insert(QLatin1String(backend_type::kProjectKeyEngineeringDrawing), canvas->toJson());
}

void EngineeringDrawingPlugin::onProjectLoaded(const QString& documentId, const QJsonObject& root)
{
	if (!root.contains(QLatin1String(backend_type::kProjectKeyEngineeringDrawing)))
		return;
	const QJsonObject drawing = root.value(QLatin1String(backend_type::kProjectKeyEngineeringDrawing)).toObject();
	DrawingPageWidget* page = ensurePageForDocument(documentId);
	if (!page || !page->canvas())
		return;
	page->canvas()->fromJson(drawing);
}
