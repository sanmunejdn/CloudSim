/// @file EngineeringDrawingPlugin.cpp
/// @brief 工程图插件二期：轴测/剖视/第三角/标注/拖视图/导出

#include "EngineeringDrawingPlugin.h"

#include "BackendTypeIds.h"
#include "DrawingPageWidget.h"
#include "DrawingRibbonBar.h"
#include "DrawingInfoPanel.h"
#include "DimStyleDialog.h"
#include "DrawingSheetCanvasWidget.h"
#include "DrawingSidePanel.h"
#include "IPluginDocument.h"
#include "IPluginGeometryHost.h"
#include "IPluginHostContext.h"
#include "PluginGeometryTypes.h"

#include <QAction>
#include <QDialog>
#include <QFileDialog>
#include <QInputDialog>
#include <QJsonObject>
#include <QLabel>
#include <QLatin1String>
#include <QLineEdit>
#include <QMenu>
#include <QMessageBox>
#include <QPushButton>
#include <QStringList>
#include <QVBoxLayout>

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
	if (host->hostVersion() < 0x00012A00U)
	{
		host->logError(QStringLiteral("EngineeringDrawingPlugin requires host 1.42.0+"));
		return false;
	}
	m_host = host;
	m_side = new DrawingSidePanel(nullptr);
	m_info = new DrawingInfoPanel(nullptr);
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
				if (m_ribbon && page->canvas())
					m_ribbon->syncFromCanvas(page->canvas());
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
	host->onDocumentClosed(
		[this](const QString& documentId)
		{
			auto it = m_pagesByDocId.find(documentId);
			if (it == m_pagesByDocId.end())
				return;
			DrawingPageWidget* page = it.value().data();
			m_pagesByDocId.erase(it);
			if (m_inDrawing && page && m_host && m_host->isShowingCentralAlternate())
				softExitDrawing();
			if (page)
				page->deleteLater();
		});

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
	delete m_info;
	m_info = nullptr;
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
	if (m_info)
		m_info->applyLanguage(zh);
	if (m_ribbon)
		m_ribbon->applyLanguage(zh);
	if (m_inDrawing && m_host && m_side)
		m_host->enterAlternateSideUi(m_side, m_info);
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
	QObject::connect(m_ribbon, &DrawingRibbonBar::generateRequested, this, &EngineeringDrawingPlugin::generateViews);
	QObject::connect(m_ribbon, &DrawingRibbonBar::exportSvgRequested, this, [this]() {
		DrawingPageWidget* page = ensurePageForActiveDocument();
		if (!page || !page->canvas())
			return;
		const QString path = QFileDialog::getSaveFileName(page, QStringLiteral("导出 SVG"), QString(),
														  QStringLiteral("SVG (*.svg)"));
		if (path.isEmpty())
			return;
		if (!page->canvas()->exportSvg(path))
			QMessageBox::warning(page, QStringLiteral("导出失败"), QStringLiteral("无法写入 SVG。"));
	});
	QObject::connect(m_ribbon, &DrawingRibbonBar::exportDxfRequested, this, [this]() {
		DrawingPageWidget* page = ensurePageForActiveDocument();
		if (!page || !page->canvas())
			return;
		const QString path = QFileDialog::getSaveFileName(page, QStringLiteral("导出 DXF"), QString(),
														  QStringLiteral("DXF (*.dxf)"));
		if (path.isEmpty())
			return;
		if (!page->canvas()->exportDxf(path))
			QMessageBox::warning(page, QStringLiteral("导出失败"), QStringLiteral("无法写入 DXF。"));
	});
	QObject::connect(m_ribbon, &DrawingRibbonBar::exportPdfRequested, this, [this]() {
		DrawingPageWidget* page = ensurePageForActiveDocument();
		if (!page || !page->canvas() || !m_ribbon)
			return;
		m_ribbon->applySheetSettings(page->canvas(), false);
		const QString path = QFileDialog::getSaveFileName(page, QStringLiteral("导出 PDF"), QString(),
														  QStringLiteral("PDF (*.pdf)"));
		if (path.isEmpty())
			return;
		if (!page->canvas()->exportPdf(path))
			QMessageBox::warning(page, QStringLiteral("导出失败"), QStringLiteral("无法写入 PDF。"));
	});
	QObject::connect(m_ribbon, &DrawingRibbonBar::importDxfRequested, this, [this]() {
		DrawingPageWidget* page = ensurePageForActiveDocument();
		if (!page || !page->canvas())
			return;
		const QString path = QFileDialog::getOpenFileName(page, QStringLiteral("导入 DXF"), QString(),
														  QStringLiteral("DXF (*.dxf)"));
		if (path.isEmpty())
			return;
		if (!page->canvas()->importDxf(path))
			QMessageBox::warning(page, QStringLiteral("导入失败"), QStringLiteral("未能解析 DXF 直线实体。"));
	});
	QObject::connect(m_ribbon, &DrawingRibbonBar::printPreviewRequested, this, [this]() {
		DrawingPageWidget* page = ensurePageForActiveDocument();
		if (!page || !page->canvas())
			return;
		QDialog dlg(page);
		dlg.setWindowTitle(QStringLiteral("打印预览"));
		auto* lay = new QVBoxLayout(&dlg);
		auto* label = new QLabel(&dlg);
		label->setPixmap(page->canvas()->renderPrintPreview(QSize(900, 640)));
		label->setAlignment(Qt::AlignCenter);
		lay->addWidget(label);
		dlg.resize(940, 700);
		dlg.exec();
	});
	QObject::connect(m_ribbon, &DrawingRibbonBar::createBlockRequested, this, [this]() {
		DrawingPageWidget* page = ensurePageForActiveDocument();
		if (!page || !page->canvas())
			return;
		bool ok = false;
		const QString name = QInputDialog::getText(page, QStringLiteral("建块"), QStringLiteral("块名"),
												   QLineEdit::Normal, QStringLiteral("Block1"), &ok);
		if (ok)
			page->canvas()->createBlockFromSelection(name);
	});
	QObject::connect(m_ribbon, &DrawingRibbonBar::insertBlockRequested, this, [this]() {
		DrawingPageWidget* page = ensurePageForActiveDocument();
		if (!page || !page->canvas() || !m_ribbon)
			return;
		const auto& defs = page->canvas()->blockDefs();
		if (defs.isEmpty())
		{
			QMessageBox::information(page, QStringLiteral("插入块"), QStringLiteral("尚无块定义，请先建块。"));
			return;
		}
		QStringList names;
		QStringList ids;
		for (const auto& d : defs)
		{
			names << (d.name.isEmpty() ? d.id : d.name);
			ids << d.id;
		}
		bool ok = false;
		const QString pick =
			QInputDialog::getItem(page, QStringLiteral("插入块"), QStringLiteral("选择块"), names, 0, false, &ok);
		if (!ok)
			return;
		const int idx = names.indexOf(pick);
		if (idx < 0 || idx >= ids.size())
			return;
		page->canvas()->setPendingInsertBlockId(ids.at(idx));
		page->canvas()->setTool(DrawingCanvasTool::InsertBlock);
		m_ribbon->setActiveTool(DrawingCanvasTool::InsertBlock);
	});
	QObject::connect(m_ribbon, &DrawingRibbonBar::dimStyleDialogRequested, this, [this]() {
		DrawingPageWidget* page = ensurePageForActiveDocument();
		if (!page || !page->canvas())
			return;
		DimStyleDialog dlg(page->canvas(), page);
		dlg.exec();
	});
	QObject::connect(m_ribbon, &DrawingRibbonBar::titleBlockAttrsRequested, this, [this]() {
		DrawingPageWidget* page = ensurePageForActiveDocument();
		if (page && page->canvas())
			page->canvas()->editTitleBlockAttrs();
	});
	QObject::connect(m_ribbon, &DrawingRibbonBar::ctbEnabledChanged, this, [this](bool on) {
		if (DrawingPageWidget* page = ensurePageForActiveDocument())
			if (page->canvas())
			{
				page->canvas()->setCtbEnabled(on);
				page->canvas()->update();
			}
	});
	QObject::connect(m_ribbon, &DrawingRibbonBar::ctbTableEditRequested, this, [this]() {
		if (DrawingPageWidget* page = ensurePageForActiveDocument())
			if (page->canvas())
				page->canvas()->editCtbTable();
	});
	QObject::connect(m_ribbon, &DrawingRibbonBar::recalculateDimsRequested, this, [this]() {
		if (DrawingPageWidget* page = ensurePageForActiveDocument())
			if (page->canvas())
				page->canvas()->recalculateDimensions(page->canvas()->selectedDimIndex() >= 0);
	});
	QObject::connect(m_ribbon, &DrawingRibbonBar::projectionDragLockChanged, this, [this](bool on) {
		if (DrawingPageWidget* page = ensurePageForActiveDocument())
			if (page->canvas())
				page->canvas()->setProjectionDragLock(on);
	});
	QObject::connect(m_ribbon, &DrawingRibbonBar::projectionPinnedChanged, this, [this](bool on) {
		if (DrawingPageWidget* page = ensurePageForActiveDocument())
			if (page->canvas())
				page->canvas()->setProjectionPinned(on);
	});
	QObject::connect(m_ribbon, &DrawingRibbonBar::halfSectionChanged, this, [this](bool on) {
		if (DrawingPageWidget* page = ensurePageForActiveDocument())
			if (page->canvas())
			{
				page->canvas()->setHalfSection(on);
				if (on)
					page->canvas()->applyHalfSectionClip();
				page->canvas()->update();
			}
	});
	QObject::connect(m_ribbon, &DrawingRibbonBar::projectionGuidesVisibleChanged, this, [this](bool on) {
		if (DrawingPageWidget* page = ensurePageForActiveDocument())
			if (page->canvas())
				page->canvas()->setProjectionGuidesVisible(on);
	});
	QObject::connect(m_ribbon, &DrawingRibbonBar::snapFlagsChanged, this, [this](SheetSnapFlags flags) {
		if (DrawingPageWidget* page = ensurePageForActiveDocument())
			if (page->canvas())
				page->canvas()->setSnapFlags(flags);
	});
	QObject::connect(m_ribbon, &DrawingRibbonBar::ltScaleChanged, this, [this](double scale) {
		if (DrawingPageWidget* page = ensurePageForActiveDocument())
			if (page->canvas())
				page->canvas()->setLtScale(scale);
	});
	QObject::connect(m_ribbon, &DrawingRibbonBar::fitWindowRequested, this, [this]() {
		if (DrawingPageWidget* page = ensurePageForActiveDocument())
			if (page->canvas())
				page->canvas()->fitToView();
	});
	QObject::connect(m_ribbon, &DrawingRibbonBar::viewAlignRequested, this, [this](ViewAlignMode mode) {
		if (DrawingPageWidget* page = ensurePageForActiveDocument())
			if (page->canvas())
				page->canvas()->alignSelectedViews(mode);
	});
	QObject::connect(m_ribbon, &DrawingRibbonBar::projectionAlignRequested, this, [this]() {
		if (DrawingPageWidget* page = ensurePageForActiveDocument())
			if (page->canvas())
				page->canvas()->alignProjectionViews();
	});
	QObject::connect(m_ribbon, &DrawingRibbonBar::fitPaperRequested, this, [this]() {
		DrawingPageWidget* page = ensurePageForActiveDocument();
		if (!page || !page->canvas() || !m_ribbon)
			return;
		m_ribbon->applySheetSettings(page->canvas(), false);
		if (page->canvas()->fitViewsToPaper())
			m_ribbon->syncFromCanvas(page->canvas());
	});
	QObject::connect(m_ribbon, &DrawingRibbonBar::gridVisibleChanged, this, [this](bool visible) {
		if (DrawingPageWidget* page = ensurePageForActiveDocument())
			if (page->canvas())
				page->canvas()->setGridVisible(visible);
	});
	QObject::connect(m_ribbon, &DrawingRibbonBar::detailScaleChanged, this, [this](double scale) {
		if (DrawingPageWidget* page = ensurePageForActiveDocument())
			if (page->canvas())
				page->canvas()->setDetailScale(scale);
	});
	QObject::connect(m_ribbon, &DrawingRibbonBar::sheetSettingsChanged, this, [this](bool rescale) {
		DrawingPageWidget* page = ensurePageForActiveDocument();
		if (page && page->canvas() && m_ribbon)
			m_ribbon->applySheetSettings(page->canvas(), rescale);
	});
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
	m_host->enterAlternateSideUi(m_side, m_info);
	m_host->showCentralAlternate();
	bindPage(page);
	if (m_ribbon && page->canvas())
	{
		m_ribbon->syncFromCanvas(page->canvas());
		m_ribbon->applySheetSettings(page->canvas(), false);
	}
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
	if (m_side && page->canvas())
		m_side->bindCanvas(page->canvas());
	if (m_info && page->canvas())
		m_info->bindCanvas(page->canvas());
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
	page->canvas()->setViews(views, true);
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
	params.thirdAngle = m_ribbon && m_ribbon->thirdAngle();
	params.includeIso = true;
	params.includeSection = false;
	// 侧栏预览：开快速预览或复杂件时走网格引擎
	params.coarseView = m_ribbon && m_ribbon->coarseView();
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
	if (!geo || !doc || !page || !page->canvas() || !m_ribbon)
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

	m_ribbon->applySheetSettings(page->canvas(), false);

	PluginDrawingProjectParams params;
	params.thirdAngle = m_ribbon->thirdAngle();
	params.includeIso = m_ribbon->includeIso();
	params.includeSection = m_ribbon->includeSection();
	params.sectionPlane = m_ribbon->sectionPlane();
	params.customSection = m_ribbon->customSection();
	params.coarseView = m_ribbon->coarseView();
	m_ribbon->sectionOriginMm(params.sectionOriginMm);
	m_ribbon->sectionNormal(params.sectionNormal);
	if (!params.customSection && params.includeSection && params.sectionPlane == 0 && page->canvas())
	{
		double origin[3] = {0, 0, 0};
		double normal[3] = {0, 1, 0};
		if (page->canvas()->sectionMarkOriginHint(origin, normal))
		{
			params.customSection = true;
			params.sectionOriginMm[0] = origin[0];
			params.sectionOriginMm[1] = origin[1];
			params.sectionOriginMm[2] = origin[2];
			params.sectionNormal[0] = normal[0];
			params.sectionNormal[1] = normal[1];
			params.sectionNormal[2] = normal[2];
		}
	}

	m_ribbon->generateButton()->setEnabled(false);
	geo->projectBrepToEngineeringDrawing(
		doc, backendId.toStdString(), params,
		[this, page, backendId, params](bool ok, const QString& error, const PluginDrawingHlrResult& result) {
			if (m_ribbon && m_ribbon->generateButton())
				m_ribbon->generateButton()->setEnabled(true);
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
	if (m_inDrawing && m_ribbon && page->canvas() &&
		m_host && m_host->activeDocument() &&
		QString::fromStdString(m_host->activeDocument()->documentId()) == documentId)
		m_ribbon->syncFromCanvas(page->canvas());
}
