/// @file GeometricModelingPlugin.cpp

#include "GeometricModelingPlugin.h"

#include "BackendTypeIds.h"
#include "BodyHistoryCmd.h"
#include "CloudSimGeomPython.h"
#include "CommandStack.h"
#include "GeometricModelingPage.h"
#include "GeometricModelingRibbonBar.h"
#include "GeomodelingI18n.h"
#include "IAiAssistantHost.h"
#include "IPluginDocument.h"
#include "IPluginGeometryHost.h"
#include "IPluginHostContext.h"
#include "ScriptModelIo.h"
#include "SketchConstraintSolver.h"
#include "SketchGeom.h"
#include "SketchTools.h"

#include <QAction>
#include <QCursor>
#include <QFile>
#include <QFileDialog>
#include <QInputDialog>
#include <QIODevice>
#include <QJsonObject>
#include <QLatin1String>
#include <QMenu>
#include <QMessageBox>
#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <utility>
#include <vector>

namespace
{
bool planeApproxEqual(const PluginSketchPlane& a, const PluginSketchPlane& b)
{
	const double nd = a.normal.x * b.normal.x + a.normal.y * b.normal.y + a.normal.z * b.normal.z;
	if (nd < 0.999)
		return false;
	const double dx = a.origin.x - b.origin.x;
	const double dy = a.origin.y - b.origin.y;
	const double dz = a.origin.z - b.origin.z;
	return (dx * dx + dy * dy + dz * dz) < 1e-2;
}
} // namespace

GeometricModelingPlugin::~GeometricModelingPlugin() = default;

QString GeometricModelingPlugin::pluginId() const
{
	return QStringLiteral("com.cloudsim.geomodeling");
}

QString GeometricModelingPlugin::displayName() const
{
	return useChinese() ? QStringLiteral("\u51e0\u4f55\u5efa\u6a21") : QStringLiteral("Geometric Modeling");
}

void GeometricModelingPlugin::hostLogInfo(const QString& msg)
{
	if (m_host)
		m_host->logInfo(msg);
}

void GeometricModelingPlugin::hostLogWarn(const QString& msg)
{
	if (m_host)
		m_host->logWarn(msg);
}

void GeometricModelingPlugin::hostLogError(const QString& msg)
{
	if (m_host)
		m_host->logError(msg);
}

bool GeometricModelingPlugin::initialize(IPluginHostContext* host)
{
	if (!host)
		return false;
	if (host->hostVersion() < 0x00013200U)
	{
		host->logError(QStringLiteral("GeometricModelingPlugin requires host 1.50.0+"));
		return false;
	}
	m_host = host;
	CloudSimGeomPython::setHost(host);

	std::string err;
	if (!SketchConstraintSolver::runEquilateralTriangleSelfTest(&err))
		hostLogError(QStringLiteral("PlaneGCS self-test failed: %1").arg(QString::fromStdString(err)));
	else
		hostLogInfo(QStringLiteral("PlaneGCS equilateral self-test OK."));

	host->onActiveDocumentChanged(
		[this](IPluginDocument*)
		{
			if (!m_inMode || !m_host)
				return;
			if (m_host->currentWorkspaceMode() != pluginId())
			{
				softExitMode();
				return;
			}
			if (GeometricModelingPage* page = ensurePageForActiveDocument())
			{
				m_host->restoreActiveRenderWidget();
				m_host->setCentralAlternateWidget(nullptr);
				m_host->showCentralScene3D();
				m_host->enterAlternateSideUi(page->featureTreePanel(), nullptr);
				applyOriginReferenceVisibility(page);
				if (m_sketch.active())
					page->showLegendOverlay();
				else
					page->hideLegendOverlay();
			}
		});
	host->onWorkspaceModeClaimed(
		[this](const QString& modeId)
		{
			if (modeId == pluginId())
				return;
			softExitMode();
		});
	host->onProjectAboutToSave([this](const QString& id, QJsonObject& root) { onProjectAboutToSave(id, root); });
	host->onProjectLoaded([this](const QString& id, const QJsonObject& root) { onProjectLoaded(id, root); });
	host->onDocumentClosed(
		[this](const QString& documentId)
		{
			GeometricModelingPage* page = m_pages.take(documentId);
			if (!page)
				return;
			if (m_inMode && m_host)
				softExitMode();
			page->deleteLater();
		});
	host->onParametricBodyHistoryChanged(
		[this](const QString& documentId, const QString& backendId)
		{
			GeometricModelingPage* page = m_pages.value(documentId, nullptr);
			if (!page)
				page = ensurePageForActiveDocument();
			if (!page || backendId.isEmpty())
				return;
			page->setActiveBodyId(backendId);
			syncFeaturesFromBody(page);
		});
	host->onLanguageChanged([this](bool) { applyLanguage(); });
	host->registerWorkspaceMode(pluginId(), QStringLiteral("\u51e0\u4f55\u5efa\u6a21"), QStringLiteral("Modeling"),
								[this]() { enterGeometricModeling(); });
	applyLanguage();
	hostLogInfo(i18n(QStringLiteral("Geometric Modeling plugin loaded."), QStringLiteral("\u51e0\u4f55\u5efa\u6a21\u63d2\u4ef6\u5df2\u52a0\u8f7d\u3002")));
	return true;
}

void GeometricModelingPlugin::shutdown()
{
	clearExtrudePreviewUi();
	m_sketch.end();
	// 关窗路径用 softExit，避免 returnToMainWorkspace 再广播其它已 unload 插件的回调
	softExitMode();
	if (m_host)
	{
		if (m_host->currentWorkspaceMode() == pluginId())
			m_host->claimWorkspaceMode(QString());
		m_host->setModeToolBar(nullptr);
	}
	m_inMode = false;
	m_ribbon = nullptr;
	qDeleteAll(m_pages);
	m_pages.clear();
	m_host = nullptr;
}

void GeometricModelingPlugin::registerMenus()
{
	// 模式切换改由宿主顶栏分段 / 设置→模式切换，不再注册顶层菜单
}

bool GeometricModelingPlugin::useChinese() const
{
	return !m_host || m_host->useChinese();
}

QString GeometricModelingPlugin::i18n(const QString& en, const QString& zh) const
{
	return gmTr(useChinese(), en, zh);
}

void GeometricModelingPlugin::applyLanguage()
{
	const bool zh = useChinese();
	if (m_ribbon)
		m_ribbon->applyLanguage(zh);
	m_sketch.setUseChinese(zh);
	for (GeometricModelingPage* page : m_pages)
	{
		if (page)
			page->applyLanguage(zh);
	}
}

void GeometricModelingPlugin::ensureRibbon()
{
	if (m_ribbon)
		return;
	m_ribbon = new GeometricModelingRibbonBar(nullptr);
	m_ribbon->applyLanguage(useChinese());
	connect(m_ribbon, &GeometricModelingRibbonBar::newSketchRequested, this, &GeometricModelingPlugin::onNewSketch);
	connect(m_ribbon, &GeometricModelingRibbonBar::datumPlaneRequested, this, &GeometricModelingPlugin::onDatumPlane);
	connect(m_ribbon, &GeometricModelingRibbonBar::endSketchRequested, this, &GeometricModelingPlugin::onEndSketch);
	connect(m_ribbon, &GeometricModelingRibbonBar::lineToolRequested, this, &GeometricModelingPlugin::onToolLine);
	connect(m_ribbon, &GeometricModelingRibbonBar::arcToolRequested, this, &GeometricModelingPlugin::onToolArc);
	connect(m_ribbon, &GeometricModelingRibbonBar::circleToolRequested, this, &GeometricModelingPlugin::onToolCircle);
	connect(m_ribbon, &GeometricModelingRibbonBar::rectToolRequested, this, &GeometricModelingPlugin::onToolRect);
	connect(m_ribbon, &GeometricModelingRibbonBar::ellipseToolRequested, this, &GeometricModelingPlugin::onToolEllipse);
	connect(m_ribbon, &GeometricModelingRibbonBar::polygonToolRequested, this, &GeometricModelingPlugin::onToolPolygon);
	connect(m_ribbon, &GeometricModelingRibbonBar::slotToolRequested, this, &GeometricModelingPlugin::onToolSlot);
	connect(m_ribbon, &GeometricModelingRibbonBar::splineToolRequested, this, &GeometricModelingPlugin::onToolSpline);
	connect(m_ribbon, &GeometricModelingRibbonBar::dimLengthRequested, this, &GeometricModelingPlugin::onDimLength);
	connect(m_ribbon, &GeometricModelingRibbonBar::dimDistanceRequested, this, &GeometricModelingPlugin::onDimDistance);
	connect(m_ribbon, &GeometricModelingRibbonBar::dimRadiusRequested, this, &GeometricModelingPlugin::onDimRadius);
	connect(m_ribbon, &GeometricModelingRibbonBar::dimAngleRequested, this, &GeometricModelingPlugin::onDimAngle);
	connect(m_ribbon, &GeometricModelingRibbonBar::dimArcRadiusRequested, this, &GeometricModelingPlugin::onDimArcRadius);
	connect(m_ribbon, &GeometricModelingRibbonBar::constructionToolRequested, this,
			&GeometricModelingPlugin::onToggleConstruction);
	connect(m_ribbon, &GeometricModelingRibbonBar::geomHorizontalRequested, this,
			&GeometricModelingPlugin::onGeomHorizontal);
	connect(m_ribbon, &GeometricModelingRibbonBar::geomVerticalRequested, this, &GeometricModelingPlugin::onGeomVertical);
	connect(m_ribbon, &GeometricModelingRibbonBar::geomCoincidentRequested, this,
			&GeometricModelingPlugin::onGeomCoincident);
	connect(m_ribbon, &GeometricModelingRibbonBar::geomParallelRequested, this, &GeometricModelingPlugin::onGeomParallel);
	connect(m_ribbon, &GeometricModelingRibbonBar::geomPerpendicularRequested, this,
			&GeometricModelingPlugin::onGeomPerpendicular);
	connect(m_ribbon, &GeometricModelingRibbonBar::geomEqualLengthRequested, this,
			&GeometricModelingPlugin::onGeomEqualLength);
	connect(m_ribbon, &GeometricModelingRibbonBar::geomTangentRequested, this, &GeometricModelingPlugin::onGeomTangent);
	connect(m_ribbon, &GeometricModelingRibbonBar::geomSymmetricRequested, this,
			&GeometricModelingPlugin::onGeomSymmetric);
	connect(m_ribbon, &GeometricModelingRibbonBar::geomMidpointRequested, this,
			&GeometricModelingPlugin::onGeomMidpoint);
	connect(m_ribbon, &GeometricModelingRibbonBar::geomFixRequested, this, &GeometricModelingPlugin::onGeomFix);
	connect(m_ribbon, &GeometricModelingRibbonBar::geomFixOriginRequested, this,
			&GeometricModelingPlugin::onGeomFixOrigin);
	connect(m_ribbon, &GeometricModelingRibbonBar::trimToolRequested, this, &GeometricModelingPlugin::onTrim);
	connect(m_ribbon, &GeometricModelingRibbonBar::mirrorToolRequested, this, &GeometricModelingPlugin::onMirror);
	connect(m_ribbon, &GeometricModelingRibbonBar::deleteToolRequested, this, &GeometricModelingPlugin::onDelete);
	connect(m_ribbon, &GeometricModelingRibbonBar::projectEdgesRequested, this, &GeometricModelingPlugin::onProjectEdges);
	connect(m_ribbon, &GeometricModelingRibbonBar::convertEntitiesRequested, this,
			&GeometricModelingPlugin::onConvertEntities);
	connect(m_ribbon, &GeometricModelingRibbonBar::offsetRequested, this, &GeometricModelingPlugin::onOffset);
	connect(m_ribbon, &GeometricModelingRibbonBar::solveRequested, this, &GeometricModelingPlugin::onSolve);
	connect(m_ribbon, &GeometricModelingRibbonBar::padRequested, this, &GeometricModelingPlugin::onPad);
	connect(m_ribbon, &GeometricModelingRibbonBar::pocketRequested, this, &GeometricModelingPlugin::onPocket);
	connect(m_ribbon, &GeometricModelingRibbonBar::sweepRequested, this, &GeometricModelingPlugin::onSweep);
	connect(m_ribbon, &GeometricModelingRibbonBar::sweepCutRequested, this, &GeometricModelingPlugin::onSweepCut);
	connect(m_ribbon, &GeometricModelingRibbonBar::filletRequested, this, &GeometricModelingPlugin::onFillet);
	connect(m_ribbon, &GeometricModelingRibbonBar::chamferRequested, this, &GeometricModelingPlugin::onChamfer);
	connect(m_ribbon, &GeometricModelingRibbonBar::revolveRequested, this, &GeometricModelingPlugin::onRevolve);
	connect(m_ribbon, &GeometricModelingRibbonBar::revolveCutRequested, this, &GeometricModelingPlugin::onRevolveCut);
	connect(m_ribbon, &GeometricModelingRibbonBar::linearPatternRequested, this, &GeometricModelingPlugin::onLinearPattern);
	connect(m_ribbon, &GeometricModelingRibbonBar::circularPatternRequested, this, &GeometricModelingPlugin::onCircularPattern);
	connect(m_ribbon, &GeometricModelingRibbonBar::mirror3dRequested, this, &GeometricModelingPlugin::onMirror3d);
	connect(m_ribbon, &GeometricModelingRibbonBar::loftRequested, this, &GeometricModelingPlugin::onLoft);
	connect(m_ribbon, &GeometricModelingRibbonBar::loftCutRequested, this, &GeometricModelingPlugin::onLoftCut);
	connect(m_ribbon, &GeometricModelingRibbonBar::shellRequested, this, &GeometricModelingPlugin::onShell);
	connect(m_ribbon, &GeometricModelingRibbonBar::draftRequested, this, &GeometricModelingPlugin::onDraft);
	connect(m_ribbon, &GeometricModelingRibbonBar::rebuildRequested, this, &GeometricModelingPlugin::onRebuild);
	connect(m_ribbon, &GeometricModelingRibbonBar::undoRequested, this, &GeometricModelingPlugin::onUndo);
	connect(m_ribbon, &GeometricModelingRibbonBar::redoRequested, this, &GeometricModelingPlugin::onRedo);
	connect(m_ribbon, &GeometricModelingRibbonBar::exportHistoryRequested, this, &GeometricModelingPlugin::onExportHistory);
	connect(m_ribbon, &GeometricModelingRibbonBar::importHistoryReplaceRequested, this,
			&GeometricModelingPlugin::onImportHistoryReplace);
	connect(m_ribbon, &GeometricModelingRibbonBar::importHistoryNewRequested, this,
			&GeometricModelingPlugin::onImportHistoryNew);
	connect(m_ribbon, &GeometricModelingRibbonBar::runComposeFileRequested, this,
			&GeometricModelingPlugin::onRunComposeFile);
	connect(m_ribbon, &GeometricModelingRibbonBar::pythonConsoleRequested, this,
			&GeometricModelingPlugin::onPythonConsole);
}

void GeometricModelingPlugin::enterGeometricModeling()
{
	if (!m_host || !m_host->activeDocument())
	{
		QMessageBox::warning(nullptr, i18n(QStringLiteral("Notice"), QStringLiteral("\u63d0\u793a")),
						 i18n(QStringLiteral("Please open a document first."), QStringLiteral("\u8bf7\u5148\u6253\u5f00\u6587\u6863")));
		return;
	}
	GeometricModelingPage* page = ensurePageForActiveDocument();
	if (!page)
		return;
	ensureRibbon();
	m_host->claimWorkspaceMode(pluginId());
	m_host->setModeToolBar(m_ribbon);
	m_host->restoreActiveRenderWidget();
	m_host->setCentralAlternateWidget(nullptr);
	m_host->showCentralScene3D();
	m_host->enterAlternateSideUi(page->featureTreePanel(), nullptr);
	m_inMode = true;
	if (!page->activeBodyId().isEmpty())
		syncFeaturesFromBody(page);
	else
		refreshBodyList(page);
	refreshVisibleSketchOverlays(page);
	applyOriginReferenceVisibility(page);
	hostLogInfo(i18n(QStringLiteral("Entered Geometric Modeling: create a sketch then Pad/Pocket."),
					 QStringLiteral("\u5df2\u8fdb\u5165\u51e0\u4f55\u5efa\u6a21\uff1a\u65b0\u5efa\u8349\u56fe\u540e\u53ef Pad/Pocket\u3002")));
}

void GeometricModelingPlugin::softExitMode()
{
	if (!m_inMode)
		return;
	clearExtrudePreviewUi();
	clearSweepPreviewUi();
	clearSolidFeaturePreviewUi();
	clearOriginReferenceVisibility();
	if (m_host && m_host->geometryHost() && m_host->activeDocument())
	{
		m_host->geometryHost()->cancelOriginSketchPlanePick(m_host->activeDocument());
		m_host->geometryHost()->clearSketchOverlay(m_host->activeDocument());
	}
	m_sketch.end();
	m_inMode = false;
	for (GeometricModelingPage* page : m_pages)
		page->hideLegendOverlay();
	if (!m_host)
		return;
	m_host->setModeToolBar(nullptr);
	m_host->restoreActiveRenderWidget();
	m_host->setCentralAlternateWidget(nullptr);
	m_host->exitAlternateSideUi();
}

void GeometricModelingPlugin::exitGeometricModeling()
{
	if (!m_host)
		return;
	clearExtrudePreviewUi();
	clearSweepPreviewUi();
	clearSolidFeaturePreviewUi();
	clearOriginReferenceVisibility();
	if (m_host->geometryHost() && m_host->activeDocument())
	{
		m_host->geometryHost()->cancelOriginSketchPlanePick(m_host->activeDocument());
		m_host->geometryHost()->clearSketchOverlay(m_host->activeDocument());
	}
	m_sketch.end();
	m_inMode = false;
	for (GeometricModelingPage* page : m_pages)
		page->hideLegendOverlay();
	m_host->returnToMainWorkspace();
	hostLogInfo(i18n(QStringLiteral("Exited Geometric Modeling."), QStringLiteral("\u5df2\u9000\u51fa\u51e0\u4f55\u5efa\u6a21\u3002")));
}

GeometricModelingPage* GeometricModelingPlugin::ensurePageForActiveDocument()
{
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	if (!doc)
		return nullptr;
	const QString id = QString::fromStdString(doc->documentId());
	if (m_pages.contains(id))
		return m_pages.value(id);
	auto* page = new GeometricModelingPage(m_host, nullptr);
	page->applyLanguage(useChinese());
	connect(page, &GeometricModelingPage::lengthEdited, this, &GeometricModelingPlugin::onLengthEdited);
	connect(page, &GeometricModelingPage::namedParamEdited, this, &GeometricModelingPlugin::onNamedParamEdited);
	connect(page, &GeometricModelingPage::featureParamApplyRequested, this,
			&GeometricModelingPlugin::onFeatureParamApply);
	connect(page, &GeometricModelingPage::confirmExtrudeRequested, this, &GeometricModelingPlugin::onConfirmExtrude);
	connect(page, &GeometricModelingPage::cancelExtrudeRequested, this, &GeometricModelingPlugin::onCancelExtrude);
	connect(page, &GeometricModelingPage::confirmSweepRequested, this, &GeometricModelingPlugin::onConfirmSweep);
	connect(page, &GeometricModelingPage::cancelSweepRequested, this, &GeometricModelingPlugin::onCancelSweep);
	connect(page, &GeometricModelingPage::sweepSelectionChanged, this, &GeometricModelingPlugin::refreshSweepPreview);
	connect(page, &GeometricModelingPage::pickSweepProfileRequested, this, &GeometricModelingPlugin::onPickSweepProfile);
	connect(page, &GeometricModelingPage::pickSweepPathRequested, this, &GeometricModelingPlugin::onPickSweepPath);
	connect(page, &GeometricModelingPage::pickSweepEdgePathRequested, this, &GeometricModelingPlugin::onPickSweepEdgePath);
	connect(page, &GeometricModelingPage::featureRollbackRequested, this, &GeometricModelingPlugin::onFeatureRollback);
	connect(page, &GeometricModelingPage::exitRollbackRequested, this, &GeometricModelingPlugin::onExitRollback);
	connect(page, &GeometricModelingPage::featureDeleteRequested, this, &GeometricModelingPlugin::onFeatureDelete);
	connect(page, &GeometricModelingPage::featureEditRequested, this, &GeometricModelingPlugin::onEditFeature);
	connect(page, &GeometricModelingPage::originPlaneSketchRequested, this,
			&GeometricModelingPlugin::onOriginPlaneSketchRequested);
	connect(page, &GeometricModelingPage::fixPointToOriginRequested, this,
			&GeometricModelingPlugin::onFixPointToOriginRequested);
	connect(page, &GeometricModelingPage::sketchVisibilityToggleRequested, this,
			&GeometricModelingPlugin::onToggleSketchVisibility);
	connect(page, &GeometricModelingPage::originVisibilityChanged, this,
			&GeometricModelingPlugin::onOriginVisibilityChanged);
	m_sketch.setBackgroundOverlayProvider(
		[this](std::vector<PluginSketchOverlaySegment>& segs)
		{
			GeometricModelingPage* p = ensurePageForActiveDocument();
			if (!p)
				return;
			appendVisibleSketchOverlays(p, p->activeSketchId(), segs);
		});
	connect(page, &GeometricModelingPage::viewportFeaturePickRequested, this, &GeometricModelingPlugin::onViewportEditPick);
	connect(page, &GeometricModelingPage::pickUpToFaceRequested, this, &GeometricModelingPlugin::onPickUpToFace);
	connect(page, &GeometricModelingPage::pickUpToVertexRequested, this, &GeometricModelingPlugin::onPickUpToVertex);
	connect(page, &GeometricModelingPage::activeBodyChanged, this, &GeometricModelingPlugin::onActiveBodyChanged);
	connect(page, &GeometricModelingPage::extrudeOptionsChanged, this, &GeometricModelingPlugin::onExtrudeOptionsChanged);
	connect(page, &GeometricModelingPage::mirrorConfirmRequested, this, &GeometricModelingPlugin::onMirrorConfirm);
	connect(page, &GeometricModelingPage::mirrorCancelRequested, this, &GeometricModelingPlugin::onMirrorCancel);
	connect(page, &GeometricModelingPage::mirrorPickAxisRequested, this,
			[this]()
			{
				m_sketch.setMirrorPickingAxis(true);
				refreshMirrorPanel(ensurePageForActiveDocument());
			});
	connect(page, &GeometricModelingPage::mirrorPickEntitiesRequested, this,
			[this]()
			{
				m_sketch.setMirrorPickingAxis(false);
				refreshMirrorPanel(ensurePageForActiveDocument());
			});
	connect(page, &GeometricModelingPage::mirrorClearEntitiesRequested, this,
			[this]()
			{
				m_sketch.clearMirrorTargets();
				refreshMirrorPanel(ensurePageForActiveDocument());
			});
	connect(page, &GeometricModelingPage::mirrorRemoveEntityRequested, this,
			[this](int eid)
			{
				m_sketch.removeMirrorTarget(eid);
				refreshMirrorPanel(ensurePageForActiveDocument());
			});
	connect(page, &GeometricModelingPage::pickFilletEdgeRequested, this, &GeometricModelingPlugin::onPickFilletEdge);
	connect(page, &GeometricModelingPage::filletConfirmRequested, this, &GeometricModelingPlugin::onConfirmFillet);
	connect(page, &GeometricModelingPage::filletCancelRequested, this, &GeometricModelingPlugin::onCancelFillet);
	connect(page, &GeometricModelingPage::pickChamferEdgeRequested, this, &GeometricModelingPlugin::onPickChamferEdge);
	connect(page, &GeometricModelingPage::chamferConfirmRequested, this, &GeometricModelingPlugin::onConfirmChamfer);
	connect(page, &GeometricModelingPage::chamferCancelRequested, this, &GeometricModelingPlugin::onCancelChamfer);
	connect(page, &GeometricModelingPage::revolveConfirmRequested, this, &GeometricModelingPlugin::onConfirmRevolve);
	connect(page, &GeometricModelingPage::revolveCancelRequested, this, &GeometricModelingPlugin::onCancelRevolve);
	connect(page, &GeometricModelingPage::revolveSelectionChanged, this, &GeometricModelingPlugin::refreshRevolvePreview);
	connect(page, &GeometricModelingPage::pickRevolveAxisRequested, this, &GeometricModelingPlugin::onPickRevolveAxis);
	connect(page, &GeometricModelingPage::patternConfirmRequested, this, &GeometricModelingPlugin::onConfirmPattern);
	connect(page, &GeometricModelingPage::patternCancelRequested, this, &GeometricModelingPlugin::onCancelPattern);
	connect(page, &GeometricModelingPage::patternOptionsChanged, this, &GeometricModelingPlugin::refreshPatternPreview);
	connect(page, &GeometricModelingPage::circularPatternConfirmRequested, this,
			&GeometricModelingPlugin::onConfirmCircularPattern);
	connect(page, &GeometricModelingPage::circularPatternCancelRequested, this,
			&GeometricModelingPlugin::onCancelCircularPattern);
	connect(page, &GeometricModelingPage::circularPatternOptionsChanged, this,
			&GeometricModelingPlugin::refreshCircularPatternPreview);
	connect(page, &GeometricModelingPage::pickCircularPatternAxisRequested, this,
			&GeometricModelingPlugin::onPickCircularPatternAxis);
	connect(page, &GeometricModelingPage::mirror3dConfirmRequested, this, &GeometricModelingPlugin::onConfirmMirror3d);
	connect(page, &GeometricModelingPage::mirror3dCancelRequested, this, &GeometricModelingPlugin::onCancelMirror3d);
	connect(page, &GeometricModelingPage::mirror3dOptionsChanged, this, &GeometricModelingPlugin::refreshMirror3dPreview);
	connect(page, &GeometricModelingPage::loftConfirmRequested, this, &GeometricModelingPlugin::onConfirmLoft);
	connect(page, &GeometricModelingPage::loftCancelRequested, this, &GeometricModelingPlugin::onCancelLoft);
	connect(page, &GeometricModelingPage::loftSelectionChanged, this, &GeometricModelingPlugin::refreshLoftPreview);
	connect(page, &GeometricModelingPage::shellPickFaceRequested, this, &GeometricModelingPlugin::onPickShellFace);
	connect(page, &GeometricModelingPage::shellConfirmRequested, this, &GeometricModelingPlugin::onConfirmShell);
	connect(page, &GeometricModelingPage::shellCancelRequested, this, &GeometricModelingPlugin::onCancelShell);
	connect(page, &GeometricModelingPage::draftPickFaceRequested, this, &GeometricModelingPlugin::onPickDraftFace);
	connect(page, &GeometricModelingPage::draftPickNeutralRequested, this, &GeometricModelingPlugin::onPickDraftNeutral);
	connect(page, &GeometricModelingPage::draftConfirmRequested, this, &GeometricModelingPlugin::onConfirmDraft);
	connect(page, &GeometricModelingPage::draftCancelRequested, this, &GeometricModelingPlugin::onCancelDraft);
	m_pages.insert(id, page);
	return page;
}

void GeometricModelingPlugin::syncFeaturesFromBody(GeometricModelingPage* page)
{
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IPluginGeometryHost* geo = m_host ? m_host->geometryHost() : nullptr;
	if (!doc || !geo || !page || page->activeBodyId().isEmpty())
		return;
	QByteArray hist;
	QString err;
	if (!geo->queryParametricBodyHistoryJson(doc, page->activeBodyId().toStdString(), hist, &err))
	{
		hostLogWarn(err.isEmpty() ? QStringLiteral("\u65e0\u6cd5\u8bfb\u53d6 Body \u5386\u53f2") : err);
		return;
	}
	const std::vector<GeomodelingFeature> datums = page->features().extractDatumPlanes();
	if (!page->features().fromParametricHistoryJson(hist))
	{
		hostLogWarn(QStringLiteral("\u65e0\u6cd5\u89e3\u6790 Body \u5386\u53f2"));
		for (const GeomodelingFeature& d : datums)
			page->features().appendPreserved(d);
		return;
	}
	for (const GeomodelingFeature& d : datums)
		page->features().appendPreserved(d);
	reevaluateDatumPlanes(page);
	page->refreshFeatureTree();
	refreshBodyList(page);
	refreshVisibleSketchOverlays(page);
}

void GeometricModelingPlugin::refreshBodyList(GeometricModelingPage* page)
{
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IPluginGeometryHost* geo = m_host ? m_host->geometryHost() : nullptr;
	if (!doc || !geo || !page)
		return;
	std::vector<std::string> ids;
	QString err;
	if (!geo->listParametricBodyIds(doc, ids, &err))
	{
		if (!err.isEmpty())
			hostLogWarn(err);
		return;
	}
	QStringList qids;
	qids.reserve(static_cast<int>(ids.size()));
	for (const std::string& id : ids)
		qids.append(QString::fromStdString(id));
	page->setBodyIdList(qids);
}

bool GeometricModelingPlugin::pushFeatureHistory(GeometricModelingPage* page, const QByteArray& beforeHist)
{
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IPluginGeometryHost* geo = m_host ? m_host->geometryHost() : nullptr;
	if (!doc || !geo || !page || page->activeBodyId().isEmpty())
		return false;
	const QByteArray afterHist = page->features().toParametricHistoryJson();
	bool ok = false;
	geo->setParametricBodyHistoryJson(
		doc, page->activeBodyId().toStdString(), afterHist,
		[this, page, doc, beforeHist, afterHist, &ok](bool success, const QString& err, const PluginGeometryJobResult&)
		{
			if (!success)
			{
				hostLogError(err);
				return;
			}
			syncFeaturesFromBody(page);
			page->commands().execute(std::make_unique<BodyHistoryCmd>(
				m_host, doc, page->activeBodyId(), beforeHist, afterHist,
				[this, page]() { syncFeaturesFromBody(page); }, true));
			ok = true;
		});
	return ok;
}

void GeometricModelingPlugin::persistActiveSketchDocument(GeometricModelingPage* page)
{
	if (!page || !m_sketch.active())
		return;
	const QString sid = page->activeSketchId();
	if (sid.isEmpty())
		return;
	std::vector<float> profile;
	std::string exportErr;
	(void)m_sketch.exportClosedProfile(profile, &exportErr);
	if (profile.size() < 12)
	{
		profile.clear();
		exportErr.clear();
		(void)m_sketch.document().exportOpenPathXyz(m_sketch.plane(), profile, &exportErr);
	}
	page->features().setProfile(sid, profile);
	page->features().setSketchDocument(sid, m_sketch.sketchDocumentJson());
	page->refreshFeatureTree();
}

void GeometricModelingPlugin::appendVisibleSketchOverlays(GeometricModelingPage* page, const QString& excludeSketchId,
														  std::vector<PluginSketchOverlaySegment>& out) const
{
	if (!page)
		return;
	for (const GeomodelingFeature& f : page->features().features())
	{
		if (f.kind != GeomodelingFeatureKind::Sketch || !f.visible || f.suppressed)
			continue;
		if (!excludeSketchId.isEmpty() && f.id == excludeSketchId)
			continue;

		// 完整草图文档优先；AI/Host 仅写 profile 时用折线兜底，否则「显示」无任何几何
		if (!f.sketchDocumentUtf8.isEmpty())
		{
			SketchDocument2d sk;
			if (sk.fromJsonUtf8(f.sketchDocumentUtf8))
			{
				std::vector<PluginSketchOverlaySegment> segs;
				sk.tessellateOverlay(f.plane, segs);
				out.insert(out.end(), segs.begin(), segs.end());
				continue;
			}
		}

		auto appendPolyline = [&out](const std::vector<float>& xyz)
		{
			if (xyz.size() < 6)
				return;
			PluginSketchOverlaySegment seg;
			seg.construction = false;
			seg.lineWidthPx = 2.f;
			seg.rgba[0] = 0.2f;
			seg.rgba[1] = 0.55f;
			seg.rgba[2] = 0.95f;
			seg.rgba[3] = 1.f;
			seg.xyzMm = xyz;
			out.push_back(std::move(seg));
		};
		appendPolyline(f.profileXyzMm);
		for (const auto& hole : f.profileHolesXyzMm)
			appendPolyline(hole);
	}

	// 用户基准面：半透明矩形边框（世界 mm）
	constexpr float kHalf = 40.f;
	for (const GeomodelingFeature& f : page->features().features())
	{
		if ((f.kind != GeomodelingFeatureKind::DatumPlane && f.kind != GeomodelingFeatureKind::DatumPlaneAngle) ||
			f.suppressed || !f.visible)
			continue;
		if (!f.plane.isPlanar)
			continue;
		const auto& pl = f.plane;
		auto at = [&](float u, float v) -> PluginPoint3d
		{
			return {pl.origin.x + pl.axisX.x * u + pl.axisY.x * v, pl.origin.y + pl.axisX.y * u + pl.axisY.y * v,
					pl.origin.z + pl.axisX.z * u + pl.axisY.z * v};
		};
		const PluginPoint3d corners[5] = {at(-kHalf, -kHalf), at(kHalf, -kHalf), at(kHalf, kHalf), at(-kHalf, kHalf),
										  at(-kHalf, -kHalf)};
		PluginSketchOverlaySegment seg;
		seg.construction = true;
		seg.lineWidthPx = 1.5f;
		seg.rgba[0] = 0.15f;
		seg.rgba[1] = 0.75f;
		seg.rgba[2] = 0.55f;
		seg.rgba[3] = 0.95f;
		for (const PluginPoint3d& c : corners)
		{
			seg.xyzMm.push_back(static_cast<float>(c.x));
			seg.xyzMm.push_back(static_cast<float>(c.y));
			seg.xyzMm.push_back(static_cast<float>(c.z));
		}
		out.push_back(std::move(seg));
	}
}

void GeometricModelingPlugin::refreshVisibleSketchOverlays(GeometricModelingPage* page)
{
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IPluginGeometryHost* geo = m_host ? m_host->geometryHost() : nullptr;
	if (!doc || !geo || !page)
		return;
	// 草图编辑中：只刷当前 background provider 叠加
	if (m_sketch.active())
	{
		m_sketch.refreshOverlay();
		return;
	}

	std::vector<PluginSketchOverlaySegment> all;
	appendVisibleSketchOverlays(page, QString(), all);
	if (all.empty())
		geo->clearSketchOverlay(doc);
	else
		geo->setSketchOverlay(doc, all);
}

void GeometricModelingPlugin::onToggleSketchVisibility(const QString& featureId)
{
	GeometricModelingPage* page = ensurePageForActiveDocument();
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IPluginGeometryHost* geo = m_host ? m_host->geometryHost() : nullptr;
	if (!page || featureId.isEmpty())
		return;
	GeomodelingFeature* f = page->features().find(featureId);
	if (!f || f->kind != GeomodelingFeatureKind::Sketch)
		return;

	const bool next = !f->visible;
	page->features().setVisible(featureId, next);
	page->refreshFeatureTree();
	refreshVisibleSketchOverlays(page);

	if (doc && geo && !page->activeBodyId().isEmpty())
	{
		QByteArray beforeHist;
		QString qerr;
		if (geo->queryParametricBodyHistoryJson(doc, page->activeBodyId().toStdString(), beforeHist, &qerr))
			pushFeatureHistory(page, beforeHist);
	}
	hostLogInfo(next ? i18n(QStringLiteral("Sketch shown."), QStringLiteral("\u8349\u56fe\u5df2\u663e\u793a\u3002"))
					 : i18n(QStringLiteral("Sketch hidden."), QStringLiteral("\u8349\u56fe\u5df2\u9690\u85cf\u3002")));
}

void GeometricModelingPlugin::applyOriginReferenceVisibility(GeometricModelingPage* page)
{
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IPluginGeometryHost* geo = m_host ? m_host->geometryHost() : nullptr;
	if (!doc || !geo || !page || !m_inMode)
		return;
	PluginOriginReferenceVisibility vis;
	vis.originPoint = page->originPointVisible();
	vis.planeXY = page->originPlaneXyVisible();
	vis.planeXZ = page->originPlaneXzVisible();
	vis.planeYZ = page->originPlaneYzVisible();
	geo->setOriginReferenceVisibility(doc, vis);
}

void GeometricModelingPlugin::clearOriginReferenceVisibility()
{
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IPluginGeometryHost* geo = m_host ? m_host->geometryHost() : nullptr;
	if (!doc || !geo)
		return;
	PluginOriginReferenceVisibility vis;
	vis.originPoint = false;
	vis.planeXY = false;
	vis.planeXZ = false;
	vis.planeYZ = false;
	geo->setOriginReferenceVisibility(doc, vis);
}

void GeometricModelingPlugin::onOriginVisibilityChanged()
{
	applyOriginReferenceVisibility(ensurePageForActiveDocument());
}

void GeometricModelingPlugin::updateDofUi(GeometricModelingPage* page)
{
	if (!page)
		return;
	if (m_sketch.lastHasConflict())
		hostLogWarn(QStringLiteral("\u7ea6\u675f\u51b2\u7a81"));
	else if (m_sketch.lastHasRedundant())
		hostLogWarn(QStringLiteral("\u5b58\u5728\u5197\u4f59\u7ea6\u675f"));
}

void GeometricModelingPlugin::onNewSketch()
{
	IPluginDocument* doc = m_host->activeDocument();
	IPluginGeometryHost* geo = m_host->geometryHost();
	GeometricModelingPage* page = ensurePageForActiveDocument();
	if (!doc || !geo || !page)
		return;
	clearExtrudePreviewUi();
	std::vector<PluginSupportPlaneCandidate> extras;
	for (const GeomodelingFeature& f : page->features().features())
	{
		if ((f.kind != GeomodelingFeatureKind::DatumPlane && f.kind != GeomodelingFeatureKind::DatumPlaneAngle) ||
			f.suppressed || !f.visible || !f.plane.isPlanar)
			continue;
		PluginSupportPlaneCandidate c;
		c.plane = f.plane;
		c.tagUtf8 = f.id.toStdString();
		c.halfExtentMm = 40.f;
		extras.push_back(c);
	}
	hostLogInfo(i18n(QStringLiteral("Click a datum, origin plane or planar face to start sketch (Esc to cancel)"),
					 QStringLiteral("\u8bf7\u70b9\u9009\u53c2\u8003\u9762\u3001\u57fa\u9762\u6216\u6a21\u578b\u5e73\u9762\u5f00\u59cb\u8349\u56fe\uff08Esc \u53d6\u6d88\uff09")));
	geo->pickSketchSupportPlane(
		doc, extras,
		[this, page, geo, doc](bool ok, const QString& err, PluginOriginPlaneKind /*kind*/,
							   const PluginSketchPlane& plane, const QString& tag)
		{
			if (!ok)
			{
				hostLogInfo(err.isEmpty() ? QStringLiteral("\u5df2\u53d6\u6d88") : err);
				return;
			}
			QString datumId;
			if (!tag.isEmpty() && !tag.startsWith(QLatin1String("face:")) && !tag.startsWith(QLatin1String("origin:")))
				datumId = tag;
			beginSketchOnPlane(page, geo, doc, plane, datumId);
		});
}

namespace
{
PluginSketchPlane offsetPlaneAlongNormal(const PluginSketchPlane& src, double offsetMm)
{
	PluginSketchPlane p = src;
	p.origin.x += static_cast<float>(src.normal.x * offsetMm);
	p.origin.y += static_cast<float>(src.normal.y * offsetMm);
	p.origin.z += static_cast<float>(src.normal.z * offsetMm);
	return p;
}

bool planeFromThreePoints(const PluginPoint3d& a, const PluginPoint3d& b, const PluginPoint3d& c, PluginSketchPlane& out)
{
	const double abx = b.x - a.x, aby = b.y - a.y, abz = b.z - a.z;
	const double acx = c.x - a.x, acy = c.y - a.y, acz = c.z - a.z;
	double nx = aby * acz - abz * acy;
	double ny = abz * acx - abx * acz;
	double nz = abx * acy - aby * acx;
	const double nlen = std::sqrt(nx * nx + ny * ny + nz * nz);
	if (nlen < 1e-9)
		return false;
	nx /= nlen;
	ny /= nlen;
	nz /= nlen;
	double xx = abx, xy = aby, xz = abz;
	const double xlen = std::sqrt(xx * xx + xy * xy + xz * xz);
	if (xlen < 1e-9)
		return false;
	xx /= xlen;
	xy /= xlen;
	xz /= xlen;
	const double yx = ny * xz - nz * xy;
	const double yy = nz * xx - nx * xz;
	const double yz = nx * xy - ny * xx;
	out.isPlanar = true;
	out.origin = a;
	out.axisX = {static_cast<float>(xx), static_cast<float>(xy), static_cast<float>(xz)};
	out.axisY = {static_cast<float>(yx), static_cast<float>(yy), static_cast<float>(yz)};
	out.normal = {static_cast<float>(nx), static_cast<float>(ny), static_cast<float>(nz)};
	return true;
}

void rotateVec(double ax, double ay, double az, double ux, double uy, double uz, double angRad, double& ox, double& oy,
			   double& oz)
{
	const double c = std::cos(angRad);
	const double s = std::sin(angRad);
	const double dot = ax * ux + ay * uy + az * uz;
	ox = ux * c + (ay * uz - az * uy) * s + ax * dot * (1.0 - c);
	oy = uy * c + (az * ux - ax * uz) * s + ay * dot * (1.0 - c);
	oz = uz * c + (ax * uy - ay * ux) * s + az * dot * (1.0 - c);
}

PluginSketchPlane rotatePlaneAroundAxis(const PluginSketchPlane& src, const PluginPoint3d& axisO,
										const PluginPoint3d& axisD, double angleDeg)
{
	double alen = std::sqrt(axisD.x * axisD.x + axisD.y * axisD.y + axisD.z * axisD.z);
	if (alen < 1e-9)
		return src;
	const double ax = axisD.x / alen, ay = axisD.y / alen, az = axisD.z / alen;
	const double ang = angleDeg * 3.14159265358979323846 / 180.0;
	PluginSketchPlane p = src;
	double nx, ny, nz, xx, xy, xz, yx, yy, yz;
	rotateVec(ax, ay, az, src.normal.x, src.normal.y, src.normal.z, ang, nx, ny, nz);
	rotateVec(ax, ay, az, src.axisX.x, src.axisX.y, src.axisX.z, ang, xx, xy, xz);
	rotateVec(ax, ay, az, src.axisY.x, src.axisY.y, src.axisY.z, ang, yx, yy, yz);
	p.normal = {static_cast<float>(nx), static_cast<float>(ny), static_cast<float>(nz)};
	p.axisX = {static_cast<float>(xx), static_cast<float>(xy), static_cast<float>(xz)};
	p.axisY = {static_cast<float>(yx), static_cast<float>(yy), static_cast<float>(yz)};
	p.origin = axisO;
	p.isPlanar = true;
	return p;
}

bool isUserDatumKind(GeomodelingFeatureKind k)
{
	return k == GeomodelingFeatureKind::DatumPlane || k == GeomodelingFeatureKind::DatumPlaneAngle;
}
} // namespace

void GeometricModelingPlugin::onDatumPlane()
{
	GeometricModelingPage* page = ensurePageForActiveDocument();
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IPluginGeometryHost* geo = m_host ? m_host->geometryHost() : nullptr;
	if (!page || !doc || !geo)
		return;

	QStringList modes;
	modes << i18n(QStringLiteral("Offset from face"), QStringLiteral("\u7b49\u8ddd\u9762"))
		  << i18n(QStringLiteral("Three points"), QStringLiteral("\u4e09\u70b9"))
		  << i18n(QStringLiteral("Angle from face"), QStringLiteral("\u6210\u89d2\u9762"));
	bool ok = false;
	const QString mode = QInputDialog::getItem(
		nullptr, i18n(QStringLiteral("Datum Plane"), QStringLiteral("\u57fa\u51c6\u9762")),
		i18n(QStringLiteral("Creation method:"), QStringLiteral("\u521b\u5efa\u65b9\u5f0f\uff1a")), modes, 0, false, &ok);
	if (!ok)
		return;

	if (mode == modes[0])
	{
		hostLogInfo(i18n(QStringLiteral("Pick an origin plane or planar face to offset."),
						 QStringLiteral("\u8bf7\u70b9\u9009\u539f\u70b9\u57fa\u9762\u6216\u6a21\u578b\u5e73\u9762\u4f5c\u7b49\u8ddd\u6e90\u3002")));
		geo->pickSketchSupportPlane(
			doc, {},
			[this, page, geo, doc](bool okPick, const QString& err, PluginOriginPlaneKind kind,
								   const PluginSketchPlane& facePlane, const QString& tag)
			{
				if (!okPick)
				{
					if (!err.isEmpty())
						hostLogInfo(err);
					return;
				}
				if (!facePlane.isPlanar)
				{
					hostLogWarn(i18n(QStringLiteral("Face is not planar."), QStringLiteral("\u9762\u4e0d\u662f\u5e73\u9762\u3002")));
					return;
				}
				bool okDist = false;
				const double dist = QInputDialog::getDouble(
					nullptr, i18n(QStringLiteral("Offset"), QStringLiteral("\u7b49\u8ddd")),
					i18n(QStringLiteral("Offset distance (mm):"), QStringLiteral("\u504f\u79fb\u8ddd\u79bb\uff08mm\uff09\uff1a")),
					10.0, -1e6, 1e6, 2, &okDist);
				if (!okDist)
					return;
				const PluginSketchPlane plane = offsetPlaneAlongNormal(facePlane, dist);
				GeomodelingDatumSourceKind srcKind = GeomodelingDatumSourceKind::None;
				int originIdx = 0;
				QString faceBackend;
				int faceIndex = -1;
				if (tag.startsWith(QLatin1String("origin:")))
				{
					srcKind = GeomodelingDatumSourceKind::OriginPlane;
					originIdx = tag.section(QLatin1Char(':'), 1).toInt();
					if (originIdx < 0 || originIdx > 2)
						originIdx = static_cast<int>(kind);
				}
				else if (tag.startsWith(QLatin1String("face:")))
				{
					srcKind = GeomodelingDatumSourceKind::Face;
					const QStringList parts = tag.split(QLatin1Char(':'));
					if (parts.size() >= 3)
					{
						faceBackend = parts[1];
						faceIndex = parts[2].toInt();
					}
				}
				else if (static_cast<int>(kind) >= 0 && static_cast<int>(kind) <= 2 && tag.isEmpty())
				{
					srcKind = GeomodelingDatumSourceKind::OriginPlane;
					originIdx = static_cast<int>(kind);
				}
				const QString id =
					(srcKind == GeomodelingDatumSourceKind::None)
						? page->features().addDatumPlane(plane)
						: page->features().addDatumPlaneOffset(plane, srcKind, dist, originIdx, faceBackend, faceIndex);
				page->refreshFeatureTree();
				refreshVisibleSketchOverlays(page);
				hostLogInfo(i18n(QStringLiteral("Datum plane created: %1"), QStringLiteral("\u5df2\u521b\u5efa\u57fa\u51c6\u9762\uff1a%1"))
								.arg(id));
				(void)geo;
				(void)doc;
			});
		return;
	}

	if (mode == modes[2])
	{
		hostLogInfo(i18n(QStringLiteral("Pick a planar face, then a hinge edge."),
						 QStringLiteral("\u8bf7\u5148\u70b9\u9009\u53c2\u8003\u5e73\u9762\uff0c\u518d\u70b9\u9009\u94fe\u94fe\u8f74\u8fb9\u3002")));
		PluginGeometryElementPickRequest reqFace;
		reqFace.kind = PluginGeometryElementKind::Face;
		geo->pickStepElementFromViewport(
			doc, reqFace,
			[this, page, geo, doc](bool okFace, const QString& errFace, const PluginGeometryStepRef& faceRef)
			{
				if (!okFace)
				{
					if (!errFace.isEmpty())
						hostLogInfo(errFace);
					return;
				}
				PluginSketchPlane facePlane;
				QString planeErr;
				if (!geo->queryFaceSketchPlane(doc, faceRef, facePlane, &planeErr) || !facePlane.isPlanar)
				{
					hostLogWarn(planeErr.isEmpty()
									? i18n(QStringLiteral("Face is not planar."), QStringLiteral("\u9762\u4e0d\u662f\u5e73\u9762\u3002"))
									: planeErr);
					return;
				}
				hostLogInfo(i18n(QStringLiteral("Pick hinge edge."), QStringLiteral("\u8bf7\u70b9\u9009\u94fe\u94fe\u8f74\u8fb9\u3002")));
				PluginGeometryElementPickRequest reqEdge;
				reqEdge.kind = PluginGeometryElementKind::Edge;
				reqEdge.backendIdUtf8 = faceRef.backendIdUtf8;
				geo->pickStepElementFromViewport(
					doc, reqEdge,
					[this, page, facePlane, faceRef](bool okEdge, const QString& errEdge,
													 const PluginGeometryStepRef& edgeRef)
					{
						if (!okEdge)
						{
							if (!errEdge.isEmpty())
								hostLogInfo(errEdge);
							return;
						}
						bool okAng = false;
						const double ang = QInputDialog::getDouble(
							nullptr, i18n(QStringLiteral("Angle"), QStringLiteral("\u6210\u89d2")),
							i18n(QStringLiteral("Angle (deg):"), QStringLiteral("\u89d2\u5ea6\uff08\u5ea6\uff09\uff1a")), 30.0,
							-179.0, 179.0, 2, &okAng);
						if (!okAng)
							return;
						// 铰链：命中点为原点；边两端定轴向，失败再退回面内 axisX
						PluginPoint3d hingeO = facePlane.origin;
						if (edgeRef.hasHitPoint)
							hingeO = edgeRef.hitWorldMm;
						PluginPoint3d hingeD = facePlane.axisX;
						if (edgeRef.hasEdgeEnds)
						{
							const double dx = static_cast<double>(edgeRef.edgeEndBMm.x) - edgeRef.edgeEndAMm.x;
							const double dy = static_cast<double>(edgeRef.edgeEndBMm.y) - edgeRef.edgeEndAMm.y;
							const double dz = static_cast<double>(edgeRef.edgeEndBMm.z) - edgeRef.edgeEndAMm.z;
							const double len = std::sqrt(dx * dx + dy * dy + dz * dz);
							if (len > 1e-9)
							{
								hingeD = {static_cast<float>(dx / len), static_cast<float>(dy / len),
										  static_cast<float>(dz / len)};
							}
						}
						const PluginSketchPlane plane = rotatePlaneAroundAxis(facePlane, hingeO, hingeD, ang);
						const QString id =
							page->features().addDatumPlaneAngle(plane, ang, hingeO, hingeD);
						page->refreshFeatureTree();
						refreshVisibleSketchOverlays(page);
						hostLogInfo(i18n(QStringLiteral("Angled datum plane created: %1"),
										 QStringLiteral("\u5df2\u521b\u5efa\u6210\u89d2\u57fa\u51c6\u9762\uff1a%1"))
										.arg(id));
						(void)faceRef;
					});
			});
		return;
	}

	// 三点：依次拾取三个顶点
	hostLogInfo(i18n(QStringLiteral("Pick first vertex."), QStringLiteral("\u8bf7\u70b9\u9009\u7b2c\u4e00\u4e2a\u9876\u70b9\u3002")));
	auto pts = std::make_shared<std::vector<PluginPoint3d>>();
	auto pickNext = std::make_shared<std::function<void()>>();
	*pickNext = [this, page, geo, doc, pts, pickNext]()
	{
		PluginGeometryElementPickRequest req;
		req.kind = PluginGeometryElementKind::Vertex;
		geo->pickStepElementFromViewport(
			doc, req,
			[this, page, pts, pickNext](bool okPick, const QString& err, const PluginGeometryStepRef& ref)
			{
				if (!okPick)
				{
					if (!err.isEmpty())
						hostLogInfo(err);
					return;
				}
				if (!ref.hasHitPoint)
				{
					hostLogWarn(i18n(QStringLiteral("Vertex pick has no hit point."),
									 QStringLiteral("\u9876\u70b9\u62fe\u53d6\u672a\u8fd4\u56de\u5750\u6807\u3002")));
					return;
				}
				pts->push_back(ref.hitWorldMm);
				if (pts->size() < 3)
				{
					hostLogInfo(i18n(QStringLiteral("Pick vertex %1/3."), QStringLiteral("\u8bf7\u70b9\u9009\u9876\u70b9 %1/3\u3002"))
									.arg(static_cast<int>(pts->size()) + 1));
					(*pickNext)();
					return;
				}
				PluginSketchPlane plane;
				if (!planeFromThreePoints((*pts)[0], (*pts)[1], (*pts)[2], plane))
				{
					hostLogWarn(i18n(QStringLiteral("Three points are collinear."),
									 QStringLiteral("\u4e09\u70b9\u5171\u7ebf\uff0c\u65e0\u6cd5\u6784\u9020\u5e73\u9762\u3002")));
					return;
				}
				const QString id = page->features().addDatumPlane(plane);
				page->refreshFeatureTree();
				refreshVisibleSketchOverlays(page);
				hostLogInfo(i18n(QStringLiteral("Datum plane created: %1"), QStringLiteral("\u5df2\u521b\u5efa\u57fa\u51c6\u9762\uff1a%1"))
								.arg(id));
			});
	};
	(*pickNext)();
}

namespace
{
PluginSketchPlane makeOriginPluginPlaneLocal(int index)
{
	PluginSketchPlane p;
	p.isPlanar = true;
	p.origin = {0, 0, 0};
	switch (index)
	{
	case 1: // XZ
		p.axisX = {1, 0, 0};
		p.axisY = {0, 0, 1};
		p.normal = {0, 1, 0};
		break;
	case 2: // YZ
		p.axisX = {0, 1, 0};
		p.axisY = {0, 0, 1};
		p.normal = {1, 0, 0};
		break;
	default: // XY
		p.axisX = {1, 0, 0};
		p.axisY = {0, 1, 0};
		p.normal = {0, 0, 1};
		break;
	}
	return p;
}
} // namespace

void GeometricModelingPlugin::beginSketchOnPlane(GeometricModelingPage* page, IPluginGeometryHost* geo,
												 IPluginDocument* doc, const PluginSketchPlane& plane,
												 const QString& datumPlaneId)
{
	if (!page || !geo || !doc)
		return;

	QString sketchId;
	QByteArray existingJson;
	for (const auto& f : page->features().features())
	{
		if (f.kind != GeomodelingFeatureKind::Sketch)
			continue;
		if (!datumPlaneId.isEmpty())
		{
			if (f.datumPlaneId != datumPlaneId)
				continue;
		}
		else if (!planeApproxEqual(f.plane, plane))
			continue;
		if (f.sketchDocumentUtf8.isEmpty())
			continue;
		sketchId = f.id;
		existingJson = f.sketchDocumentUtf8;
		break;
	}

	QString beginErr;
	const bool loaded = !existingJson.isEmpty();
	auto afterBegin = [this, page]()
	{
		m_sketch.setChangeNotifier(
			[this, page]()
			{
				updateDofUi(page);
				hostLogInfo(m_sketch.statusText());
				persistActiveSketchDocument(page);
				if (m_sketch.toolKind() == SketchToolKind::Mirror)
					refreshMirrorPanel(page);
				const int eid = m_sketch.selectedEntityId();
				if (eid >= 0)
				{
					std::vector<std::pair<QString, double>> rows;
					if (m_sketch.readNamedParams(eid, rows))
						page->showNamedParams(m_sketch.entityDisplayName(eid), rows, true);
				}
			});
	};
	if (loaded)
	{
		page->setActiveSketchId(sketchId);
		if (!m_sketch.beginWithDocument(geo, doc, plane, existingJson, &beginErr))
		{
			hostLogError(beginErr);
			return;
		}
	}
	else
	{
		sketchId = page->features().addSketch(plane, QString(), datumPlaneId);
		page->setActiveSketchId(sketchId);
		if (!m_sketch.begin(geo, doc, plane, &beginErr))
		{
			hostLogError(beginErr);
			return;
		}
	}
	afterBegin();

	page->refreshFeatureTree();
	page->showLegendOverlay();
	updateDofUi(page);
	hostLogInfo(loaded ? QStringLiteral("\u8349\u56fe\u5df2\u4ece\u7279\u5f81\u52a0\u8f7d")
					   : QStringLiteral("\u5df2\u5728\u57fa\u51c6\u9762\u5f00\u59cb\u8349\u56fe\uff0c\u53f3\u952e\u786e\u8ba4\uff0cESC \u53d6\u6d88"));
}

bool GeometricModelingPlugin::resolveDatumSourcePlane(GeometricModelingPage* page, const GeomodelingFeature& datum,
													  PluginSketchPlane& out, QString* err)
{
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IPluginGeometryHost* geo = m_host ? m_host->geometryHost() : nullptr;
	if (!doc || !geo)
	{
		if (err)
			*err = QStringLiteral("geometry host unavailable");
		return false;
	}
	if (datum.datumSourceKind == GeomodelingDatumSourceKind::OriginPlane)
	{
		out = makeOriginPluginPlaneLocal(datum.datumOriginPlaneIndex);
		return out.isPlanar;
	}
	if (datum.datumSourceKind == GeomodelingDatumSourceKind::Face)
	{
		if (datum.datumFaceBackendId.isEmpty() || datum.datumFaceIndex < 0)
		{
			if (err)
				*err = i18n(QStringLiteral("Datum face reference incomplete."),
							QStringLiteral("\u57fa\u51c6\u9762\u6e90\u9762\u5f15\u7528\u4e0d\u5b8c\u6574\u3002"));
			return false;
		}
		PluginGeometryStepRef ref;
		ref.backendIdUtf8 = datum.datumFaceBackendId.toStdString();
		ref.faceIndex = datum.datumFaceIndex;
		QString planeErr;
		if (!geo->queryFaceSketchPlane(doc, ref, out, &planeErr) || !out.isPlanar)
		{
			if (err)
				*err = planeErr.isEmpty() ? i18n(QStringLiteral("Cannot resolve datum source face."),
												 QStringLiteral("\u65e0\u6cd5\u89e3\u6790\u57fa\u51c6\u9762\u6e90\u9762\u3002"))
										  : planeErr;
			return false;
		}
		(void)page;
		return true;
	}
	if (err)
		*err = i18n(QStringLiteral("Datum has no associative source."),
					QStringLiteral("\u8be5\u57fa\u51c6\u9762\u65e0\u5173\u8054\u6e90\u3002"));
	return false;
}

void GeometricModelingPlugin::syncSketchesBoundToDatum(GeometricModelingPage* page, const QString& datumId,
													   const PluginSketchPlane& plane)
{
	if (!page || datumId.isEmpty())
		return;
	std::vector<QString> ids;
	for (const GeomodelingFeature& f : page->features().features())
	{
		if (f.kind == GeomodelingFeatureKind::Sketch && f.datumPlaneId == datumId)
			ids.push_back(f.id);
	}
	for (const QString& id : ids)
	{
		GeomodelingFeature* f = page->features().find(id);
		if (!f)
			continue;
		f->plane = plane;
		if (f->sketchDocumentUtf8.isEmpty())
			continue;
		SketchDocument2d sk;
		if (!sk.fromJsonUtf8(f->sketchDocumentUtf8))
			continue;
		std::vector<std::vector<float>> loops;
		std::string exportErr;
		if (!sk.exportClosedProfilesXyz(plane, loops, &exportErr) || loops.empty())
			continue;
		f->profileXyzMm = loops.front();
		f->profileHolesXyzMm.clear();
		for (std::size_t i = 1; i < loops.size(); ++i)
			f->profileHolesXyzMm.push_back(std::move(loops[i]));
	}
}

void GeometricModelingPlugin::reevaluateDatumPlanes(GeometricModelingPage* page)
{
	if (!page)
		return;
	std::vector<QString> ids;
	for (const GeomodelingFeature& f : page->features().features())
	{
		if ((f.kind == GeomodelingFeatureKind::DatumPlane || f.kind == GeomodelingFeatureKind::DatumPlaneAngle) &&
			f.datumSourceKind != GeomodelingDatumSourceKind::None)
			ids.push_back(f.id);
	}
	for (const QString& id : ids)
	{
		GeomodelingFeature* f = page->features().find(id);
		if (!f)
			continue;
		PluginSketchPlane src;
		QString err;
		if (!resolveDatumSourcePlane(page, *f, src, &err))
		{
			hostLogWarn(err.isEmpty() ? i18n(QStringLiteral("Datum source lost: %1"),
											 QStringLiteral("\u57fa\u51c6\u9762\u6e90\u5931\u6548\uff1a%1"))
											.arg(f->id)
									  : err);
			continue;
		}
		if (f->kind == GeomodelingFeatureKind::DatumPlaneAngle)
			f->plane = rotatePlaneAroundAxis(offsetPlaneAlongNormal(src, f->datumOffsetMm), f->datumHingeOrigin,
											 f->datumHingeDir, f->datumAngleDeg);
		else
			f->plane = offsetPlaneAlongNormal(src, f->datumOffsetMm);
		syncSketchesBoundToDatum(page, f->id, f->plane);
	}
}

void GeometricModelingPlugin::onOriginPlaneSketchRequested(int planeIndex)
{
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IPluginGeometryHost* geo = m_host ? m_host->geometryHost() : nullptr;
	GeometricModelingPage* page = ensurePageForActiveDocument();
	if (!doc || !geo || !page)
		return;
	if (geo)
		geo->cancelOriginSketchPlanePick(doc);
	clearExtrudePreviewUi();
	clearSweepPreviewUi();
	if (m_sketch.active())
	{
		persistActiveSketchDocument(page);
		m_sketch.end();
		page->hideLegendOverlay();
	}
	beginSketchOnPlane(page, geo, doc, makeOriginPluginPlaneLocal(planeIndex));
}

void GeometricModelingPlugin::onFixPointToOriginRequested()
{
	GeometricModelingPage* page = ensurePageForActiveDocument();
	if (!page)
		return;
	if (!m_sketch.active())
	{
		hostLogInfo(i18n(QStringLiteral("Start or edit a sketch first, then fix a point to origin."),
						 QStringLiteral("\u8bf7\u5148\u5f00\u59cb\u6216\u7f16\u8f91\u8349\u56fe\uff0c\u518d\u5c06\u70b9\u56fa\u5b9a\u5230\u539f\u70b9\u3002")));
		return;
	}
	setActiveTool(SketchToolKind::GeomFixOrigin);
	hostLogInfo(i18n(QStringLiteral("Click a sketch point to fix at origin (0,0)."),
					 QStringLiteral("\u8bf7\u70b9\u9009\u8349\u56fe\u70b9\uff0c\u56fa\u5b9a\u5230\u539f\u70b9 (0,0)\u3002")));
}

void GeometricModelingPlugin::onEndSketch()
{
	GeometricModelingPage* page = ensurePageForActiveDocument();
	if (m_host && m_host->geometryHost() && m_host->activeDocument())
		m_host->geometryHost()->cancelOriginSketchPlanePick(m_host->activeDocument());
	persistActiveSketchDocument(page);
	m_sketch.end();
	if (page)
	{
		page->hideLegendOverlay();
		page->setSideToolPanel(SideToolPanel::None);
		page->refreshFeatureTree();
	}
	rebuildDownstreamAfterSketch(page);
	refreshVisibleSketchOverlays(page);
	hostLogInfo(i18n(QStringLiteral("Sketch editing ended"), QStringLiteral("\u8349\u56fe\u7f16\u8f91\u5df2\u7ed3\u675f")));
}

void GeometricModelingPlugin::refreshMirrorPanel(GeometricModelingPage* page)
{
	if (!page)
		return;
	const int axisId = m_sketch.mirrorAxisId();
	const QString axisText =
		axisId >= 0 ? m_sketch.entityDisplayName(axisId) : QStringLiteral("\u672a\u9009\u62e9\u955c\u50cf\u8f74");
	std::vector<std::pair<int, QString>> ents;
	ents.reserve(m_sketch.mirrorTargetIds().size());
	for (int id : m_sketch.mirrorTargetIds())
		ents.emplace_back(id, m_sketch.entityDisplayName(id));
	page->updateMirrorPanel(axisId, axisText, ents, m_sketch.mirrorPickingAxis(),
							axisId >= 0 && !ents.empty());
}

void GeometricModelingPlugin::onMirrorConfirm()
{
	GeometricModelingPage* page = ensurePageForActiveDocument();
	if (!page || !m_sketch.active())
		return;
	QString err;
	if (!m_sketch.confirmMirror(&err))
	{
		hostLogWarn(err);
		return;
	}
	hostLogInfo(err);
	persistActiveSketchDocument(page);
	refreshMirrorPanel(page);
}

void GeometricModelingPlugin::onMirrorCancel()
{
	GeometricModelingPage* page = ensurePageForActiveDocument();
	if (!page)
		return;
	m_sketch.resetMirrorSelection();
	page->setSideToolPanel(SideToolPanel::None);
	setActiveTool(SketchToolKind::Line);
	hostLogInfo(QStringLiteral("\u5df2\u53d6\u6d88"));
}

void GeometricModelingPlugin::setActiveTool(SketchToolKind kind)
{
	if (!m_sketch.active())
	{
		hostLogInfo(i18n(QStringLiteral("Enter a sketch first"), QStringLiteral("\u8bf7\u5148\u8fdb\u5165\u8349\u56fe")));
		return;
	}
	m_sketch.setTool(kind);
	GeometricModelingPage* page = ensurePageForActiveDocument();
	if (!page || m_previewActive)
		return;
	if (kind == SketchToolKind::Mirror)
	{
		page->setSideToolPanel(SideToolPanel::Mirror);
		refreshMirrorPanel(page);
	}
	else if (kind == SketchToolKind::Trim)
	{
		page->setSideToolPanel(SideToolPanel::Trim);
		page->setTrimHint(i18n(QStringLiteral("Click a line segment that intersects others to trim."), QStringLiteral("\u70b9\u9009\u4e0e\u5176\u5b83\u76f4\u7ebf\u76f8\u4ea4\u7684\u7ebf\u6bb5\u8fdb\u884c\u88c1\u526a\u3002")));
	}
	else
	{
		page->setSideToolPanel(SideToolPanel::None);
	}
}

void GeometricModelingPlugin::onToolLine() { setActiveTool(SketchToolKind::Line); }
void GeometricModelingPlugin::onToolArc() { setActiveTool(SketchToolKind::Arc); }
void GeometricModelingPlugin::onToolCircle() { setActiveTool(SketchToolKind::Circle); }
void GeometricModelingPlugin::onToolRect() { setActiveTool(SketchToolKind::Rectangle); }
void GeometricModelingPlugin::onToolEllipse() { setActiveTool(SketchToolKind::Ellipse); }
void GeometricModelingPlugin::onToolPolygon()
{
	if (!m_sketch.active())
	{
		hostLogWarn(i18n(QStringLiteral("Open a sketch first."), QStringLiteral("\u8bf7\u5148\u8fdb\u5165\u8349\u56fe\u7f16\u8f91\u3002")));
		return;
	}
	bool ok = false;
	const int sides = QInputDialog::getInt(
		nullptr, i18n(QStringLiteral("Polygon"), QStringLiteral("\u591a\u8fb9\u5f62")),
		i18n(QStringLiteral("Number of sides (3-24):"), QStringLiteral("\u8fb9\u6570\uff083-24\uff09\uff1a")),
		m_sketch.polygonSides(), PolygonSketchTool::kMinSides, PolygonSketchTool::kMaxSides, 1, &ok);
	if (!ok)
		return;
	m_sketch.setPolygonSides(sides);
	setActiveTool(SketchToolKind::Polygon);
}
void GeometricModelingPlugin::onToolSlot() { setActiveTool(SketchToolKind::Slot); }
void GeometricModelingPlugin::onToolSpline() { setActiveTool(SketchToolKind::Spline); }
void GeometricModelingPlugin::onDimLength() { setActiveTool(SketchToolKind::DimLength); }
void GeometricModelingPlugin::onDimDistance() { setActiveTool(SketchToolKind::DimDistance); }
void GeometricModelingPlugin::onDimRadius() { setActiveTool(SketchToolKind::DimRadius); }
void GeometricModelingPlugin::onDimAngle() { setActiveTool(SketchToolKind::DimAngle); }
void GeometricModelingPlugin::onDimArcRadius() { setActiveTool(SketchToolKind::DimArcRadius); }
void GeometricModelingPlugin::onToggleConstruction() { setActiveTool(SketchToolKind::ToggleConstruction); }
void GeometricModelingPlugin::onGeomHorizontal() { setActiveTool(SketchToolKind::GeomHorizontal); }
void GeometricModelingPlugin::onGeomVertical() { setActiveTool(SketchToolKind::GeomVertical); }
void GeometricModelingPlugin::onGeomCoincident() { setActiveTool(SketchToolKind::GeomCoincident); }
void GeometricModelingPlugin::onGeomParallel() { setActiveTool(SketchToolKind::GeomParallel); }
void GeometricModelingPlugin::onGeomPerpendicular() { setActiveTool(SketchToolKind::GeomPerpendicular); }
void GeometricModelingPlugin::onGeomEqualLength() { setActiveTool(SketchToolKind::GeomEqualLength); }
void GeometricModelingPlugin::onGeomFix() { setActiveTool(SketchToolKind::GeomFix); }
void GeometricModelingPlugin::onGeomFixOrigin() { setActiveTool(SketchToolKind::GeomFixOrigin); }
void GeometricModelingPlugin::onTrim() { setActiveTool(SketchToolKind::Trim); }
void GeometricModelingPlugin::onMirror() { setActiveTool(SketchToolKind::Mirror); }
void GeometricModelingPlugin::onDelete() { setActiveTool(SketchToolKind::Delete); }

void GeometricModelingPlugin::onSolve()
{
	GeometricModelingPage* page = ensurePageForActiveDocument();
	if (!page)
		return;
	if (!m_sketch.active())
	{
		hostLogInfo(i18n(QStringLiteral("Enter a sketch first"), QStringLiteral("\u8bf7\u5148\u8fdb\u5165\u8349\u56fe")));
		return;
	}
	std::string err;
	if (!m_sketch.solveNow(&err))
	{
		const QString msg = QString::fromStdString(err.empty() ? "\u6c42\u89e3\u5931\u8d25" : err);
		updateDofUi(page);
		hostLogError(msg);
		return;
	}
	m_sketch.refreshOverlay();
	updateDofUi(page);
	persistActiveSketchDocument(page);
	hostLogInfo(m_sketch.statusText() + QStringLiteral(" | \u5df2\u6c42\u89e3"));
}

void GeometricModelingPlugin::onPad() { beginExtrudePreview(false); }
void GeometricModelingPlugin::onPocket() { beginExtrudePreview(true); }
void GeometricModelingPlugin::onSweep() { beginSweepPanel(false); }
void GeometricModelingPlugin::onSweepCut() { beginSweepPanel(true); }

void GeometricModelingPlugin::clearExtrudePreviewUi()
{
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IPluginGeometryHost* geo = m_host ? m_host->geometryHost() : nullptr;
	if (geo && doc)
		geo->clearSketchExtrudePreview(doc);
	m_previewActive = false;
	m_previewPocket = false;
	m_editExtrudeMode = false;
	m_previewProfile.clear();
	m_previewHoles.clear();
	m_previewProfileSegments.clear();
	if (GeometricModelingPage* page = ensurePageForActiveDocument())
	{
		if (!m_sweepPreviewActive)
			page->setExtrudePreviewUi(false);
		page->setEditingFeatureId(QString());
		page->clearUpToFacePlane();
		page->clearUpToVertex();
		page->setExtrudeOperationMode(false);
	}
}

void GeometricModelingPlugin::clearSweepPreviewUi()
{
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IPluginGeometryHost* geo = m_host ? m_host->geometryHost() : nullptr;
	if (geo && doc)
		geo->clearSketchExtrudePreview(doc);
	m_sweepPreviewActive = false;
	m_sweepCut = false;
	m_editSweepMode = false;
	m_sweepProfile.clear();
	m_sweepProfileSegments.clear();
	m_sweepPath.clear();
	m_sweepPathSegments.clear();
	m_sweepPathFromEdge = false;
	if (GeometricModelingPage* page = ensurePageForActiveDocument())
	{
		page->setSweepPreviewUi(false, false);
		if (!m_editExtrudeMode)
			page->setEditingFeatureId(QString());
	}
}

bool GeometricModelingPlugin::loadSketchPolyline(const GeomodelingFeature& sketch, bool asPath,
												 std::vector<float>& out, QString* err) const
{
	out.clear();
	if (asPath)
	{
		if (sketch.pathXyzMm.size() >= 6)
		{
			out = sketch.pathXyzMm;
			return true;
		}
		if (sketch.profileXyzMm.size() >= 6)
		{
			out = sketch.profileXyzMm;
			return true;
		}
	}
	else if (sketch.profileXyzMm.size() >= 12)
	{
		out = sketch.profileXyzMm;
		return true;
	}

	SketchDocument2d doc;
	if (sketch.sketchDocumentUtf8.isEmpty() || !doc.fromJsonUtf8(sketch.sketchDocumentUtf8))
	{
		if (err)
			*err = i18n(QStringLiteral("Sketch document missing."), QStringLiteral("\u8349\u56fe\u6587\u6863\u7f3a\u5931\u3002"));
		return false;
	}
	std::string exportErr;
	const bool ok = asPath ? doc.exportOpenPathXyz(sketch.plane, out, &exportErr)
						   : doc.exportClosedProfileXyz(sketch.plane, out, &exportErr);
	if (!ok || (asPath ? out.size() < 6 : out.size() < 12))
	{
		if (err)
			*err = QString::fromStdString(exportErr.empty()
											  ? (asPath ? "open path export failed" : "closed profile export failed")
											  : exportErr);
		return false;
	}
	return true;
}

bool GeometricModelingPlugin::loadSketchPathSegments(const GeomodelingFeature& sketch,
													 std::vector<PluginSketchSweepPathSegment>& out,
													 QString* err) const
{
	out.clear();
	SketchDocument2d doc;
	if (sketch.sketchDocumentUtf8.isEmpty() || !doc.fromJsonUtf8(sketch.sketchDocumentUtf8))
	{
		if (err)
			*err = i18n(QStringLiteral("Sketch document missing."), QStringLiteral("\u8349\u56fe\u6587\u6863\u7f3a\u5931\u3002"));
		return false;
	}
	std::string exportErr;
	if (!doc.exportOpenPathSegments(sketch.plane, out, &exportErr) || out.empty())
	{
		if (err)
			*err = QString::fromStdString(exportErr.empty() ? "open path segments failed" : exportErr);
		return false;
	}
	return true;
}

bool GeometricModelingPlugin::validateSweepPathAtProfileCenter(const std::vector<float>& profileXyzMm,
															   const std::vector<PluginSketchSweepPathSegment>& pathSegs,
															   const std::vector<float>& pathXyzMm, QString* err) const
{
	if (profileXyzMm.size() < 9 || (profileXyzMm.size() % 3) != 0)
	{
		if (err)
			*err = i18n(QStringLiteral("Invalid sweep profile."), QStringLiteral("\u626b\u63cf\u8f6e\u5ed3\u65e0\u6548\u3002"));
		return false;
	}

	double cx = 0.0, cy = 0.0, cz = 0.0;
	std::size_t n = profileXyzMm.size() / 3;
	// 闭合折线末点若与首点重合则不计入中心
	if (n >= 2)
	{
		const double dx = profileXyzMm[0] - profileXyzMm[(n - 1) * 3];
		const double dy = profileXyzMm[1] - profileXyzMm[(n - 1) * 3 + 1];
		const double dz = profileXyzMm[2] - profileXyzMm[(n - 1) * 3 + 2];
		if (dx * dx + dy * dy + dz * dz < 1e-12)
			--n;
	}
	if (n < 3)
	{
		if (err)
			*err = i18n(QStringLiteral("Invalid sweep profile."), QStringLiteral("\u626b\u63cf\u8f6e\u5ed3\u65e0\u6548\u3002"));
		return false;
	}
	for (std::size_t i = 0; i < n; ++i)
	{
		cx += profileXyzMm[i * 3];
		cy += profileXyzMm[i * 3 + 1];
		cz += profileXyzMm[i * 3 + 2];
	}
	cx /= static_cast<double>(n);
	cy /= static_cast<double>(n);
	cz /= static_cast<double>(n);

	double x0 = 0.0, y0 = 0.0, z0 = 0.0, x1 = 0.0, y1 = 0.0, z1 = 0.0;
	bool haveEnds = false;
	if (!pathSegs.empty())
	{
		x0 = pathSegs.front().ax;
		y0 = pathSegs.front().ay;
		z0 = pathSegs.front().az;
		x1 = pathSegs.back().bx;
		y1 = pathSegs.back().by;
		z1 = pathSegs.back().bz;
		haveEnds = true;
	}
	else if (pathXyzMm.size() >= 6 && (pathXyzMm.size() % 3) == 0)
	{
		const std::size_t pn = pathXyzMm.size() / 3;
		x0 = pathXyzMm[0];
		y0 = pathXyzMm[1];
		z0 = pathXyzMm[2];
		x1 = pathXyzMm[(pn - 1) * 3];
		y1 = pathXyzMm[(pn - 1) * 3 + 1];
		z1 = pathXyzMm[(pn - 1) * 3 + 2];
		haveEnds = true;
	}
	if (!haveEnds)
	{
		if (err)
			*err = i18n(QStringLiteral("Invalid sweep path."), QStringLiteral("\u626b\u63cf\u8def\u5f84\u65e0\u6548\u3002"));
		return false;
	}

	auto dist2 = [&](double x, double y, double z) {
		const double dx = x - cx, dy = y - cy, dz = z - cz;
		return dx * dx + dy * dy + dz * dz;
	};
	// 与轮廓尺寸相关的容差，避免浮点/导出误差误报
	double diag2 = 0.0;
	for (std::size_t i = 0; i < n; ++i)
	{
		const double dx = profileXyzMm[i * 3] - cx;
		const double dy = profileXyzMm[i * 3 + 1] - cy;
		const double dz = profileXyzMm[i * 3 + 2] - cz;
		diag2 = std::max(diag2, dx * dx + dy * dy + dz * dz);
	}
	const double tol = std::max(0.05, 1e-3 * std::sqrt(std::max(diag2, 1.0)));
	const double tol2 = tol * tol;
	if (dist2(x0, y0, z0) > tol2 && dist2(x1, y1, z1) > tol2)
	{
		if (err)
			*err = i18n(QStringLiteral("One path endpoint must be at the profile sketch geometric center."),
						QStringLiteral("\u8def\u5f84\u7aef\u70b9\u5fc5\u987b\u843d\u5728\u8f6e\u5ed3\u8349\u56fe\u51e0\u4f55\u4e2d\u5fc3\uff0c\u5426\u5219\u626b\u63cf\u7ed3\u679c\u4e0d\u6b63\u786e\u3002"));
		return false;
	}
	return true;
}

void GeometricModelingPlugin::beginSweepPanel(bool cut)
{
	GeometricModelingPage* page = ensurePageForActiveDocument();
	if (!page)
		return;
	clearExtrudePreviewUi();
	clearSweepPreviewUi();

	int sketchCount = 0;
	for (const auto& f : page->features().features())
	{
		if (f.kind == GeomodelingFeatureKind::Sketch)
			++sketchCount;
	}
	if (sketchCount < 1)
	{
		hostLogWarn(i18n(QStringLiteral("Sweep needs a profile sketch (path sketch or model edge optional)."),
						 QStringLiteral("\u626b\u63cf\u9700\u8981\u8f6e\u5ed3\u8349\u56fe\uff08\u8def\u5f84\u8349\u56fe\u6216\u6a21\u578b\u8fb9\u53ef\u9009\uff09\u3002")));
		return;
	}
	if (cut && page->activeBodyId().isEmpty())
	{
		hostLogWarn(i18n(QStringLiteral("SweepCut requires an active Parametric Body."),
						 QStringLiteral("\u626b\u63cf\u5207\u9664\u9700\u8981\u6d3b\u52a8 Parametric Body\u3002")));
		return;
	}

	m_sweepCut = cut;
	m_sweepPreviewActive = true;
	m_editSweepMode = false;
	m_sweepPathFromEdge = false;
	page->setEditingFeatureId(QString());
	page->fillSweepSketchCombos();
	page->setSweepPreviewUi(true, cut);
	refreshSweepPreview();
	hostLogInfo(cut ? i18n(QStringLiteral("Select profile and path sketches for Sweep Cut."),
						   QStringLiteral("\u8bf7\u9009\u62e9\u8f6e\u5ed3\u4e0e\u8def\u5f84\u8349\u56fe\u8fdb\u884c\u626b\u63cf\u5207\u9664\u3002"))
					: i18n(QStringLiteral("Select profile and path sketches for Sweep."),
						   QStringLiteral("\u8bf7\u9009\u62e9\u8f6e\u5ed3\u4e0e\u8def\u5f84\u8349\u56fe\u8fdb\u884c\u626b\u63cf\u51f8\u53f0\u3002")));
}

void GeometricModelingPlugin::refreshSweepPreview()
{
	GeometricModelingPage* page = ensurePageForActiveDocument();
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IPluginGeometryHost* geo = m_host ? m_host->geometryHost() : nullptr;
	if (!page || !doc || !geo || !m_sweepPreviewActive)
		return;

	const QString profileId = page->sweepProfileSketchId();
	const QString pathId = page->sweepPathSketchId();
	if (profileId.isEmpty() || (!m_sweepPathFromEdge && (pathId.isEmpty() || profileId == pathId)))
	{
		geo->clearSketchExtrudePreview(doc);
		const QString msg = i18n(QStringLiteral("Select profile sketch (and path sketch or model edge)."),
								 QStringLiteral("\u8bf7\u9009\u62e9\u8f6e\u5ed3\u8349\u56fe\uff08\u4ee5\u53ca\u8def\u5f84\u8349\u56fe\u6216\u6a21\u578b\u8fb9\u8def\u5f84\uff09\u3002"));
		hostLogWarn(msg);
		page->setSweepStatus(msg);
		return;
	}
	const GeomodelingFeature* profileSk = page->features().find(profileId);
	const GeomodelingFeature* pathSk = m_sweepPathFromEdge ? nullptr : page->features().find(pathId);
	if (!profileSk || (!m_sweepPathFromEdge && !pathSk))
		return;

	QString err;
	std::vector<float> profile;
	std::vector<float> path;
	std::vector<PluginSketchSweepPathSegment> pathSegs;
	if (!loadSketchPolyline(*profileSk, false, profile, &err))
	{
		hostLogWarn(err);
		page->setSweepStatus(err);
		geo->clearSketchExtrudePreview(doc);
		return;
	}
	if (m_sweepPathFromEdge)
	{
		path = m_sweepPath;
		pathSegs.clear();
	}
	else if (!loadSketchPathSegments(*pathSk, pathSegs, &err))
	{
		hostLogWarn(err);
		page->setSweepStatus(err);
		geo->clearSketchExtrudePreview(doc);
		return;
	}
	if (!m_sweepPathFromEdge && !loadSketchPolyline(*pathSk, true, path, &err))
	{
		path.clear();
		for (const auto& s : pathSegs)
		{
			if (path.empty())
			{
				path.push_back(s.ax);
				path.push_back(s.ay);
				path.push_back(s.az);
			}
			path.push_back(s.bx);
			path.push_back(s.by);
			path.push_back(s.bz);
		}
	}
	if (!validateSweepPathAtProfileCenter(profile, pathSegs, path, &err))
	{
		hostLogWarn(err);
		page->setSweepStatus(err);
		geo->clearSketchExtrudePreview(doc);
		return;
	}

	m_sweepProfile = profile;
	{
		std::vector<PluginSketchSweepPathSegment> segs;
		SketchDocument2d doc;
		if (!profileSk->sketchDocumentUtf8.isEmpty() && doc.fromJsonUtf8(profileSk->sketchDocumentUtf8))
			(void)doc.exportClosedProfileSegments(profileSk->plane, segs, nullptr);
		else if (!profileSk->profileSegments.empty())
		{
			for (const auto& s : profileSk->profileSegments)
			{
				PluginSketchSweepPathSegment p;
				p.kind = static_cast<PluginSketchSweepPathSegKind>(s.kind);
				p.ax = s.ax;
				p.ay = s.ay;
				p.az = s.az;
				p.bx = s.bx;
				p.by = s.by;
				p.bz = s.bz;
				p.mx = s.mx;
				p.my = s.my;
				p.mz = s.mz;
				segs.push_back(p);
			}
		}
		m_sweepProfileSegments = std::move(segs);
	}
	m_sweepPath = path;
	m_sweepPathSegments = pathSegs;

	PluginSketchSweepParams params;
	params.mode = m_sweepCut ? PluginSketchSweepMode::Cut : PluginSketchSweepMode::Boss;
	params.targetParametricBackendIdUtf8 = page->activeBodyId().toStdString();
	params.pathSegments = pathSegs;
	params.profileSegments = m_sweepProfileSegments;
	params.twistDeg = page->sweepTwistDeg();
	QString previewErr;
	if (!geo->previewSketchSweep(doc, profile, path, params, &previewErr))
	{
		const QString msg =
			previewErr.isEmpty()
				? i18n(QStringLiteral("Sweep preview failed."), QStringLiteral("\u626b\u63cf\u9884\u89c8\u5931\u8d25\u3002"))
				: previewErr;
		page->setSweepStatus(msg);
		hostLogWarn(msg);
		return;
	}
	page->setSweepStatus(QString());
}

void GeometricModelingPlugin::onConfirmSweep()
{
	if (m_editSweepMode)
		commitEditSweep();
	else
		commitSweep();
}

void GeometricModelingPlugin::onCancelSweep()
{
	clearSweepPreviewUi();
	hostLogInfo(i18n(QStringLiteral("Sweep cancelled."), QStringLiteral("\u626b\u63cf\u5df2\u53d6\u6d88\u3002")));
}

void GeometricModelingPlugin::onPickSweepProfile()
{
	GeometricModelingPage* page = ensurePageForActiveDocument();
	if (!page || !m_sweepPreviewActive)
		return;
	const QString id = page->selectedFeatureId();
	const GeomodelingFeature* f = page->features().find(id);
	if (!f || f->kind != GeomodelingFeatureKind::Sketch)
	{
		hostLogWarn(i18n(QStringLiteral("Select a sketch in the feature tree first."),
						 QStringLiteral("\u8bf7\u5148\u5728\u7279\u5f81\u6811\u9009\u4e2d\u8f6e\u5ed3\u8349\u56fe\u3002")));
		return;
	}
	page->selectSweepProfileSketch(id);
	refreshSweepPreview();
}

void GeometricModelingPlugin::onPickSweepPath()
{
	GeometricModelingPage* page = ensurePageForActiveDocument();
	if (!page || !m_sweepPreviewActive)
		return;
	const QString id = page->selectedFeatureId();
	const GeomodelingFeature* f = page->features().find(id);
	if (!f || f->kind != GeomodelingFeatureKind::Sketch)
	{
		hostLogWarn(i18n(QStringLiteral("Select a sketch in the feature tree first."),
						 QStringLiteral("\u8bf7\u5148\u5728\u7279\u5f81\u6811\u9009\u4e2d\u8def\u5f84\u8349\u56fe\u3002")));
		return;
	}
	page->selectSweepPathSketch(id);
	refreshSweepPreview();
}

void GeometricModelingPlugin::commitSweep()
{
	GeometricModelingPage* page = ensurePageForActiveDocument();
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IPluginGeometryHost* geo = m_host ? m_host->geometryHost() : nullptr;
	if (!doc || !geo || !page || !m_sweepPreviewActive)
		return;

	refreshSweepPreview();
	if (m_sweepProfile.size() < 12 || (m_sweepPathSegments.empty() && m_sweepPath.size() < 6))
	{
		const QString msg = i18n(QStringLiteral("Invalid profile or path for sweep."),
								 QStringLiteral("\u626b\u63cf\u8f6e\u5ed3\u6216\u8def\u5f84\u65e0\u6548\u3002"));
		hostLogWarn(msg);
		page->setSweepStatus(msg);
		return;
	}

	const QString profileId = page->sweepProfileSketchId();
	const QString pathId = page->sweepPathSketchId();
	if (profileId.isEmpty() || (!m_sweepPathFromEdge && (pathId.isEmpty() || profileId == pathId)))
	{
		hostLogWarn(i18n(QStringLiteral("Select profile sketch (and path sketch or model edge)."),
						 QStringLiteral("\u8bf7\u9009\u62e9\u8f6e\u5ed3\u8349\u56fe\uff08\u4ee5\u53ca\u8def\u5f84\u8349\u56fe\u6216\u6a21\u578b\u8fb9\u8def\u5f84\uff09\u3002")));
		return;
	}
	const GeomodelingFeature* profileSk = page->features().find(profileId);
	const GeomodelingFeature* pathSk = m_sweepPathFromEdge ? nullptr : page->features().find(pathId);
	if (!profileSk || (!m_sweepPathFromEdge && !pathSk))
		return;

	QByteArray beforeHist;
	if (!page->activeBodyId().isEmpty())
	{
		QString qerr;
		(void)geo->queryParametricBodyHistoryJson(doc, page->activeBodyId().toStdString(), beforeHist, &qerr);
	}

	PluginSketchSweepParams params;
	params.mode = m_sweepCut ? PluginSketchSweepMode::Cut : PluginSketchSweepMode::Boss;
	params.resultNameUtf8 = "ParametricBody";
	params.targetParametricBackendIdUtf8 = page->activeBodyId().toStdString();
	if (!m_sweepCut && page->activeBodyId().isEmpty())
		params.targetParametricBackendIdUtf8.clear();
	params.profileSketchIdUtf8 = profileId.toStdString();
	if (!m_sweepPathFromEdge)
		params.pathSketchIdUtf8 = pathId.toStdString();
	params.profilePlane = profileSk->plane;
	if (pathSk)
	{
		params.pathPlane = pathSk->plane;
		params.pathSketchDocumentJsonUtf8 = QString::fromUtf8(pathSk->sketchDocumentUtf8).toStdString();
	}
	params.profileSketchDocumentJsonUtf8 = QString::fromUtf8(profileSk->sketchDocumentUtf8).toStdString();
	params.pathSegments = m_sweepPathSegments;
	params.profileSegments = m_sweepProfileSegments;
	params.twistDeg = page->sweepTwistDeg();

	const std::vector<float> profile = m_sweepProfile;
	const std::vector<float> path = m_sweepPath;
	clearSweepPreviewUi();

	geo->sweepSketchProfileToBrep(
		doc, profile, path, params,
		[this, page, doc, beforeHist](bool ok, const QString& err, const PluginGeometryJobResult& result)
		{
			if (!ok)
			{
				hostLogError(err);
				return;
			}
			page->setActiveBodyId(QString::fromStdString(result.newBackendId));
			refreshBodyList(page);
			syncFeaturesFromBody(page);

			QByteArray afterHist;
			QString qerr;
			if (IPluginGeometryHost* g = m_host ? m_host->geometryHost() : nullptr)
				(void)g->queryParametricBodyHistoryJson(doc, page->activeBodyId().toStdString(), afterHist, &qerr);
			const QByteArray beforeSnap =
				beforeHist.isEmpty() ? QByteArrayLiteral("{\"features\":[],\"seq\":1}") : beforeHist;
			page->commands().execute(std::make_unique<BodyHistoryCmd>(
				m_host, doc, page->activeBodyId(), beforeSnap, afterHist,
				[this, page]() { syncFeaturesFromBody(page); }, true));
			hostLogInfo(i18n(QStringLiteral("Sweep feature created on body: %1"),
							 QStringLiteral("\u5df2\u5728\u5b9e\u4f53 %1 \u4e0a\u521b\u5efa\u626b\u63cf\u7279\u5f81"))
							.arg(page->activeBodyId()));
		});
}

void GeometricModelingPlugin::commitEditSweep()
{
	GeometricModelingPage* page = ensurePageForActiveDocument();
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IPluginGeometryHost* geo = m_host ? m_host->geometryHost() : nullptr;
	if (!doc || !geo || !page || !m_sweepPreviewActive)
		return;

	const QString featureId = page->editingFeatureId();
	GeomodelingFeature* feature = page->features().find(featureId);
	if (!feature
		|| (feature->kind != GeomodelingFeatureKind::Sweep && feature->kind != GeomodelingFeatureKind::SweepCut))
		return;

	refreshSweepPreview();
	if (m_sweepProfile.size() < 12 || (m_sweepPathSegments.empty() && m_sweepPath.size() < 6))
	{
		const QString msg = i18n(QStringLiteral("Invalid profile or path for sweep."),
								 QStringLiteral("\u626b\u63cf\u8f6e\u5ed3\u6216\u8def\u5f84\u65e0\u6548\u3002"));
		hostLogWarn(msg);
		page->setSweepStatus(msg);
		return;
	}

	feature->sketchRefId = page->sweepProfileSketchId();
	feature->pathSketchRefId = page->sweepPathSketchId();
	feature->profileXyzMm = m_sweepProfile;
	feature->pathXyzMm = m_sweepPath;
	feature->pathSegments.clear();
	for (const auto& p : m_sweepPathSegments)
	{
		GeomodelingFeature::PathSegment s;
		s.kind = (p.kind == PluginSketchSweepPathSegKind::Arc)			? 1
				 : (p.kind == PluginSketchSweepPathSegKind::SplineThrough) ? 2
																		  : 0;
		s.ax = p.ax;
		s.ay = p.ay;
		s.az = p.az;
		s.bx = p.bx;
		s.by = p.by;
		s.bz = p.bz;
		s.mx = p.mx;
		s.my = p.my;
		s.mz = p.mz;
		feature->pathSegments.push_back(s);
	}
	page->refreshFeatureTree();

	QByteArray beforeHist;
	QString qerr;
	if (!geo->queryParametricBodyHistoryJson(doc, page->activeBodyId().toStdString(), beforeHist, &qerr))
	{
		hostLogWarn(qerr);
		return;
	}

	clearSweepPreviewUi();
	pushFeatureHistory(page, beforeHist);
	hostLogInfo(i18n(QStringLiteral("Sweep feature updated."), QStringLiteral("\u626b\u63cf\u7279\u5f81\u5df2\u66f4\u65b0\u3002")));
}

void GeometricModelingPlugin::onEditSweepFeature(const QString& featureId)
{
	GeometricModelingPage* page = ensurePageForActiveDocument();
	if (!page)
		return;

	const GeomodelingFeature* feature = page->features().find(featureId);
	if (!feature
		|| (feature->kind != GeomodelingFeatureKind::Sweep && feature->kind != GeomodelingFeatureKind::SweepCut))
		return;

	clearExtrudePreviewUi();
	clearSweepPreviewUi();

	const bool cut = feature->kind == GeomodelingFeatureKind::SweepCut;
	m_sweepCut = cut;
	m_sweepPreviewActive = true;
	m_editSweepMode = true;
	page->setEditingFeatureId(featureId);
	page->fillSweepSketchCombos(feature->sketchRefId, feature->pathSketchRefId);
	page->setSweepPreviewUi(true, cut);
	refreshSweepPreview();
	hostLogInfo(i18n(QStringLiteral("Editing sweep feature."), QStringLiteral("\u6b63\u5728\u7f16\u8f91\u626b\u63cf\u7279\u5f81\u3002")));
}

void GeometricModelingPlugin::fillExtrudeParams(GeometricModelingPage* page, PluginSketchExtrudeParams& params) const
{
	if (!page)
		return;
	params.lengthMm = page->extrudeLengthMm();
	params.length2Mm = page->extrudeLength2Mm();
	params.startOffsetMm = page->extrudeStartOffsetMm();
	params.draftAngleDeg = page->extrudeDraftAngleDeg();
	params.reversed = page->extrudeReversed();
	const GeomodelingExtrudeEnd end = page->extrudeEndCondition();
	if (end == GeomodelingExtrudeEnd::UpToFace)
		params.endCondition = PluginSketchExtrudeEnd::UpToFace;
	else if (end == GeomodelingExtrudeEnd::MidPlane)
		params.endCondition = PluginSketchExtrudeEnd::MidPlane;
	else if (end == GeomodelingExtrudeEnd::ThroughAll)
		params.endCondition = PluginSketchExtrudeEnd::ThroughAll;
	else if (end == GeomodelingExtrudeEnd::UpToVertex)
		params.endCondition = PluginSketchExtrudeEnd::UpToVertex;
	else if (end == GeomodelingExtrudeEnd::OffsetFromFace)
		params.endCondition = PluginSketchExtrudeEnd::OffsetFromFace;
	else if (end == GeomodelingExtrudeEnd::TwoDirections)
		params.endCondition = PluginSketchExtrudeEnd::TwoDirections;
	else
		params.endCondition = PluginSketchExtrudeEnd::Blind;
	params.hasUpToFacePlane = page->hasUpToFacePlane();
	if (params.hasUpToFacePlane)
		params.upToFacePlane = page->upToFacePlane();
	params.hasUpToVertex = page->hasUpToVertex();
	if (params.hasUpToVertex)
	{
		params.upToVertex = page->upToVertex();
		params.upToVertexIndex = page->upToVertexIndex();
		params.upToVertexBackendIdUtf8 = page->upToVertexBackendId().toStdString();
	}
	params.offsetFromFaceMm = page->offsetFromFaceMm();
	params.upToFaceBackendIdUtf8 = page->upToFaceBackendId().toStdString();
	params.upToFaceIndex = page->upToFaceIndex();
	params.resultNameUtf8 = "ParametricBody";
	if (page->extrudeCreateNewBody())
		params.targetParametricBackendIdUtf8.clear();
	else
		params.targetParametricBackendIdUtf8 = page->activeBodyId().toStdString();
}

void GeometricModelingPlugin::beginExtrudePreviewFromProfile(bool pocket, const std::vector<float>& profile,
															 const PluginSketchPlane& plane,
															 const std::vector<std::vector<float>>& holes,
															 const std::vector<PluginSketchSweepPathSegment>& profileSegs)
{
	GeometricModelingPage* page = ensurePageForActiveDocument();
	if (!page || profile.size() < 12)
		return;
	clearSweepPreviewUi();
	if (pocket && page->extrudeCreateNewBody())
	{
		hostLogWarn(i18n(QStringLiteral("Pocket cannot create a new body."),
						 QStringLiteral("Pocket \u4e0d\u652f\u6301\u65b0\u5efa\u5b9e\u4f53\u3002")));
		return;
	}
	if (pocket && page->activeBodyId().isEmpty())
	{
		hostLogWarn(i18n(QStringLiteral("Select a Pocket target body first."),
						 QStringLiteral("\u8bf7\u5148\u5728\u4fa7\u680f\u9009\u62e9\u5207\u9664\u76ee\u6807\u5b9e\u4f53\u3002")));
		return;
	}
	if ((page->extrudeEndCondition() == GeomodelingExtrudeEnd::UpToFace
		 || page->extrudeEndCondition() == GeomodelingExtrudeEnd::OffsetFromFace)
		&& !page->hasUpToFacePlane())
	{
		hostLogWarn(i18n(QStringLiteral("Select an up-to face first."),
						 QStringLiteral("\u8bf7\u5148\u9009\u62e9\u7ec8\u6b62\u9762\u3002")));
		return;
	}
	if (page->extrudeEndCondition() == GeomodelingExtrudeEnd::UpToVertex && !page->hasUpToVertex())
	{
		hostLogWarn(i18n(QStringLiteral("Select an up-to vertex first."),
						 QStringLiteral("\u8bf7\u5148\u9009\u62e9\u7ec8\u6b62\u9876\u70b9\u3002")));
		return;
	}
	if (page->extrudeEndCondition() == GeomodelingExtrudeEnd::ThroughAll
		&& (page->extrudeCreateNewBody() || page->activeBodyId().isEmpty()))
	{
		hostLogWarn(i18n(QStringLiteral("Through all requires an existing target body."),
						 QStringLiteral("\u8d2f\u901a\u9700\u8981\u5df2\u6709\u76ee\u6807\u5b9e\u4f53\u3002")));
		return;
	}

	page->setExtrudeOperationMode(pocket);
	m_previewActive = true;
	m_previewPocket = pocket;
	m_previewProfile = profile;
	m_previewHoles = holes;
	m_previewProfileSegments = profileSegs;
	m_previewPlane = plane;
	page->setExtrudePreviewUi(true);
	refreshExtrudePreview();
}

void GeometricModelingPlugin::beginExtrudePreview(bool pocket)
{
	IPluginDocument* doc = m_host->activeDocument();
	IPluginGeometryHost* geo = m_host->geometryHost();
	GeometricModelingPage* page = ensurePageForActiveDocument();
	if (!doc || !geo || !page)
		return;

	std::vector<float> profile;
	std::vector<std::vector<float>> holes;
	std::string exportErr;
	if (!m_sketch.active())
	{
		hostLogWarn(i18n(QStringLiteral("Need a closed sketch profile."),
						 QStringLiteral("\u9700\u8981\u95ed\u5408\u8349\u56fe\u8f6e\u5ed3\u3002")));
		return;
	}
	{
		std::vector<std::vector<float>> loops;
		if (!m_sketch.document().exportClosedProfilesXyz(m_sketch.plane(), loops, &exportErr) || loops.empty()
			|| loops.front().size() < 12)
		{
			const QString msg = exportErr.empty()
									? i18n(QStringLiteral("Need a closed sketch profile."),
										   QStringLiteral("\u9700\u8981\u95ed\u5408\u8349\u56fe\u8f6e\u5ed3\u3002"))
									: QString::fromStdString(exportErr);
			hostLogWarn(msg);
			return;
		}
		profile = loops.front();
		for (std::size_t i = 1; i < loops.size(); ++i)
		{
			if (loops[i].size() >= 12)
				holes.push_back(std::move(loops[i]));
		}
	}

	std::vector<PluginSketchSweepPathSegment> profileSegs;
	(void)m_sketch.document().exportClosedProfileSegments(m_sketch.plane(), profileSegs, nullptr);

	persistActiveSketchDocument(page);
	m_editExtrudeMode = false;
	page->setEditingFeatureId(QString());
	beginExtrudePreviewFromProfile(pocket, profile, m_sketch.plane(), holes, profileSegs);
	hostLogInfo(pocket ? i18n(QStringLiteral("Pocket preview started."), QStringLiteral("Pocket \u9884\u89c8\u5df2\u5f00\u59cb\u3002"))
					   : i18n(QStringLiteral("Pad preview started."), QStringLiteral("Pad \u9884\u89c8\u5df2\u5f00\u59cb\u3002")));
}

void GeometricModelingPlugin::refreshExtrudePreview()
{
	if (!m_previewActive)
		return;
	IPluginDocument* doc = m_host->activeDocument();
	IPluginGeometryHost* geo = m_host->geometryHost();
	GeometricModelingPage* page = ensurePageForActiveDocument();
	if (!doc || !geo || !page || m_previewProfile.size() < 12)
		return;

	PluginSketchExtrudeParams params;
	params.mode = m_previewPocket ? PluginSketchExtrudeMode::Pocket : PluginSketchExtrudeMode::Pad;
	fillExtrudeParams(page, params);
	params.holePolylinesXyzMm = m_previewHoles;
	params.profileSegments = m_previewProfileSegments;
	geo->previewSketchExtrude(doc, m_previewProfile, m_previewPlane, params);
}

void GeometricModelingPlugin::onLengthEdited(double /*mm*/)
{
	if (m_previewActive)
		refreshExtrudePreview();
}

void GeometricModelingPlugin::onNamedParamEdited(const QString& key, double value)
{
	if (!m_sketch.active())
		return;
	const int eid = m_sketch.selectedEntityId();
	if (eid < 0)
		return;
	QString err;
	if (!m_sketch.applyNamedParam(eid, key, value, &err))
	{
		if (!err.isEmpty())
			hostLogWarn(err);
		return;
	}
	if (GeometricModelingPage* page = ensurePageForActiveDocument())
		persistActiveSketchDocument(page);
}

void GeometricModelingPlugin::onFeatureParamApply(const QString& featureId, const QString& key, double value)
{
	GeometricModelingPage* page = ensurePageForActiveDocument();
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IPluginGeometryHost* geo = m_host ? m_host->geometryHost() : nullptr;
	if (!page || !doc || !geo || featureId.isEmpty())
		return;
	GeomodelingFeature* f = page->features().find(featureId);
	if (!f)
		return;
	if (key == QStringLiteral("pad.depth") && value > 1e-9)
		(void)page->features().setLength(featureId, value);
	else if (key == QStringLiteral("pad.draftAngle"))
		(void)page->features().setDraftAngleDeg(featureId, value);
	else if (key == QStringLiteral("datum.offset"))
	{
		if (f->kind != GeomodelingFeatureKind::DatumPlane && f->kind != GeomodelingFeatureKind::DatumPlaneAngle)
			return;
		f->datumOffsetMm = value;
		PluginSketchPlane src;
		QString err;
		if (!resolveDatumSourcePlane(page, *f, src, &err))
		{
			hostLogWarn(err.isEmpty() ? i18n(QStringLiteral("Datum source lost; keeping baked plane."),
											 QStringLiteral("\u53c2\u8003\u6e90\u5931\u6548\uff0c\u4fdd\u7559\u5f53\u524d\u5e73\u9762\u3002"))
									  : err);
			page->refreshFeatureTree();
			return;
		}
		if (f->kind == GeomodelingFeatureKind::DatumPlaneAngle)
			f->plane = rotatePlaneAroundAxis(offsetPlaneAlongNormal(src, f->datumOffsetMm), f->datumHingeOrigin,
											 f->datumHingeDir, f->datumAngleDeg);
		else
			f->plane = offsetPlaneAlongNormal(src, f->datumOffsetMm);
		syncSketchesBoundToDatum(page, f->id, f->plane);
		page->refreshFeatureTree();
		refreshVisibleSketchOverlays(page);
		rebuildDownstreamAfterSketch(page);
		return;
	}
	else if (key == QStringLiteral("datum.angle"))
	{
		if (f->kind != GeomodelingFeatureKind::DatumPlaneAngle)
			return;
		f->datumAngleDeg = value;
		PluginSketchPlane src;
		QString err;
		if (resolveDatumSourcePlane(page, *f, src, &err))
			f->plane = rotatePlaneAroundAxis(src, f->datumHingeOrigin, f->datumHingeDir, f->datumAngleDeg);
		else
			hostLogWarn(err.isEmpty() ? i18n(QStringLiteral("Datum source lost; angle stored only."),
											 QStringLiteral("\u53c2\u8003\u6e90\u5931\u6548\uff0c\u4ec5\u4fdd\u5b58\u89d2\u5ea6\u3002"))
									  : err);
		syncSketchesBoundToDatum(page, f->id, f->plane);
		page->refreshFeatureTree();
		refreshVisibleSketchOverlays(page);
		rebuildDownstreamAfterSketch(page);
		return;
	}
	else
		return;
	page->refreshFeatureTree();
	// 参数改完后走重建，避免与预览态打架
	onRebuild();
}

void GeometricModelingPlugin::onExtrudeOptionsChanged()
{
	if (m_previewActive)
		refreshExtrudePreview();
}

void GeometricModelingPlugin::onConfirmExtrude()
{
	if (m_editExtrudeMode)
		commitEditExtrude();
	else
		commitExtrude();
}

void GeometricModelingPlugin::onCancelExtrude()
{
	clearExtrudePreviewUi();
	hostLogInfo(i18n(QStringLiteral("Extrude preview cancelled."), QStringLiteral("\u62c9\u4f38\u9884\u89c8\u5df2\u53d6\u6d88\u3002")));
}

void GeometricModelingPlugin::commitEditExtrude()
{
	GeometricModelingPage* page = ensurePageForActiveDocument();
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IPluginGeometryHost* geo = m_host ? m_host->geometryHost() : nullptr;
	if (!doc || !geo || !page || !m_previewActive)
		return;

	const QString featureId = page->editingFeatureId();
	if (featureId.isEmpty())
		return;

	page->features().setLength(featureId, page->extrudeLengthMm());
	page->features().setDraftAngleDeg(featureId, page->extrudeDraftAngleDeg());
	page->features().setReversed(featureId, page->extrudeReversed());
	if (GeomodelingFeature* feat = page->features().find(featureId))
	{
		feat->startOffsetMm = page->extrudeStartOffsetMm();
		feat->length2Mm = page->extrudeLength2Mm();
	}
	PluginSketchPlane upPlane;
	const PluginSketchPlane* upTo = nullptr;
	QString upBackend;
	int upIndex = -1;
	if (page->hasUpToFacePlane())
	{
		upPlane = page->upToFacePlane();
		upTo = &upPlane;
		upBackend = page->upToFaceBackendId();
		upIndex = page->upToFaceIndex();
	}
	page->features().setExtrudeEnd(featureId, page->extrudeEndCondition(), upTo, upBackend, upIndex);
	page->refreshFeatureTree();

	QByteArray beforeHist;
	QString qerr;
	if (!geo->queryParametricBodyHistoryJson(doc, page->activeBodyId().toStdString(), beforeHist, &qerr))
	{
		hostLogWarn(qerr);
		return;
	}

	clearExtrudePreviewUi();
	pushFeatureHistory(page, beforeHist);
	hostLogInfo(i18n(QStringLiteral("Extrude feature updated."), QStringLiteral("\u62c9\u4f38\u7279\u5f81\u5df2\u66f4\u65b0\u3002")));
}

void GeometricModelingPlugin::commitExtrude()
{
	IPluginDocument* doc = m_host->activeDocument();
	IPluginGeometryHost* geo = m_host->geometryHost();
	GeometricModelingPage* page = ensurePageForActiveDocument();
	if (!doc || !geo || !page || !m_previewActive || m_previewProfile.size() < 12)
		return;

	const bool pocket = m_previewPocket;
	const std::vector<float> profile = m_previewProfile;
	const std::vector<std::vector<float>> holes = m_previewHoles;
	const PluginSketchPlane plane = m_previewPlane;

	QByteArray beforeHist;
	if (!page->activeBodyId().isEmpty())
	{
		QString qerr;
		(void)geo->queryParametricBodyHistoryJson(doc, page->activeBodyId().toStdString(), beforeHist, &qerr);
	}

	persistActiveSketchDocument(page);

	PluginSketchExtrudeParams params;
	params.mode = pocket ? PluginSketchExtrudeMode::Pocket : PluginSketchExtrudeMode::Pad;
	fillExtrudeParams(page, params);
	params.holePolylinesXyzMm = holes;
	params.profileSegments = m_previewProfileSegments;
	const QString sid = page->activeSketchId();
	if (!sid.isEmpty())
	{
		if (const auto* sk = page->features().find(sid))
			params.sketchDocumentJsonUtf8 = QString::fromUtf8(sk->sketchDocumentUtf8).toStdString();
	}

	geo->clearSketchExtrudePreview(doc);
	m_previewActive = false;
	m_editExtrudeMode = false;
	page->setExtrudePreviewUi(false);

	geo->extrudeSketchProfileToBrep(
		doc, profile, plane, params,
		[this, page, doc, beforeHist](bool ok, const QString& err, const PluginGeometryJobResult& result)
		{
			if (!ok)
			{
				hostLogError(err);
				return;
			}
			page->setActiveBodyId(QString::fromStdString(result.newBackendId));
			refreshBodyList(page);
			syncFeaturesFromBody(page);

			QByteArray afterHist;
			QString qerr;
			if (IPluginGeometryHost* g = m_host ? m_host->geometryHost() : nullptr)
				(void)g->queryParametricBodyHistoryJson(doc, page->activeBodyId().toStdString(), afterHist, &qerr);
			const QByteArray beforeSnap =
				beforeHist.isEmpty() ? QByteArrayLiteral("{\"features\":[],\"seq\":1}") : beforeHist;
			page->commands().execute(std::make_unique<BodyHistoryCmd>(
				m_host, doc, page->activeBodyId(), beforeSnap, afterHist,
				[this, page]() { syncFeaturesFromBody(page); }, true));
			m_sketch.end();
			if (page)
				page->hideLegendOverlay();
			hostLogInfo(i18n(QStringLiteral("Switched to parametric body: %1 (editable)."),
							 QStringLiteral("\u5df2\u5207\u6362 Parametric Body: %1\uff08\u53ef\u7f16\u8f91\uff09"))
								.arg(page->activeBodyId()));
		});
}

void GeometricModelingPlugin::onRebuild()
{
	GeometricModelingPage* page = ensurePageForActiveDocument();
	if (!page)
		return;
	IPluginDocument* doc = m_host->activeDocument();
	IPluginGeometryHost* geo = m_host->geometryHost();
	if (!doc || !geo || page->activeBodyId().isEmpty())
	{
		hostLogInfo(QStringLiteral("\u65e0 Parametric Body \u53ef\u91cd\u5efa"));
		return;
	}

	QByteArray beforeHist;
	QString qerr;
	if (!geo->queryParametricBodyHistoryJson(doc, page->activeBodyId().toStdString(), beforeHist, &qerr))
	{
		hostLogWarn(qerr);
		return;
	}

	const QByteArray afterHist = page->features().toParametricHistoryJson();
	geo->setParametricBodyHistoryJson(
		doc, page->activeBodyId().toStdString(), afterHist,
		[this, page, doc, beforeHist, afterHist](bool ok, const QString& err, const PluginGeometryJobResult&)
		{
			if (!ok)
			{
				hostLogError(err);
				return;
			}
			syncFeaturesFromBody(page);
			page->commands().execute(std::make_unique<BodyHistoryCmd>(
				m_host, doc, page->activeBodyId(), beforeHist, afterHist,
				[this, page]() { syncFeaturesFromBody(page); }, true));
			hostLogInfo(QStringLiteral("\u5df2\u91cd\u5efa Body"));
		});
}

void GeometricModelingPlugin::onUndo()
{
	if (GeometricModelingPage* page = ensurePageForActiveDocument())
		page->commands().undo();
}

void GeometricModelingPlugin::onRedo()
{
	if (GeometricModelingPage* page = ensurePageForActiveDocument())
		page->commands().redo();
}

void GeometricModelingPlugin::applyHistoryJsonToBody(GeometricModelingPage* page, IPluginDocument* doc,
													 IPluginGeometryHost* geo, const QString& bodyId,
													 const QByteArray& historyUtf8, const QByteArray& beforeHist)
{
	if (!page || !doc || !geo || bodyId.isEmpty())
		return;
	geo->setParametricBodyHistoryJson(
		doc, bodyId.toStdString(), historyUtf8,
		[this, page, doc, bodyId, beforeHist, historyUtf8](bool ok, const QString& err, const PluginGeometryJobResult&)
		{
			if (!ok)
			{
				hostLogError(err);
				return;
			}
			page->setActiveBodyId(bodyId);
			refreshBodyList(page);
			syncFeaturesFromBody(page);
			const QByteArray beforeSnap =
				beforeHist.isEmpty() ? QByteArrayLiteral("{\"features\":[],\"seq\":1}") : beforeHist;
			page->commands().execute(std::make_unique<BodyHistoryCmd>(
				m_host, doc, bodyId, beforeSnap, historyUtf8,
				[this, page]() { syncFeaturesFromBody(page); }, true));
			hostLogInfo(i18n(QStringLiteral("History imported: %1"), QStringLiteral("\u5df2\u5bfc\u5165\u5386\u53f2\uff1a%1"))
							.arg(bodyId));
		});
}

void GeometricModelingPlugin::createBodyThenApplyHistory(GeometricModelingPage* page, IPluginDocument* doc,
														 IPluginGeometryHost* geo, const QByteArray& historyUtf8)
{
	if (!page || !doc || !geo)
		return;
	// 空 target 建最小 Pad，再覆盖为完整 history（一期无新建 Body ABI）
	PluginSketchPlane plane;
	plane.origin = {0, 0, 0};
	plane.axisX = {1, 0, 0};
	plane.axisY = {0, 1, 0};
	plane.normal = {0, 0, 1};
	plane.isPlanar = true;
	const std::vector<float> profile = {0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0};
	PluginSketchExtrudeParams params;
	params.mode = PluginSketchExtrudeMode::Pad;
	params.lengthMm = 1.0;
	params.endCondition = PluginSketchExtrudeEnd::Blind;
	params.targetParametricBackendIdUtf8.clear();
	params.resultNameUtf8 = "ImportedBody";
	geo->extrudeSketchProfileToBrep(
		doc, profile, plane, params,
		[this, page, doc, geo, historyUtf8](bool ok, const QString& err, const PluginGeometryJobResult& result)
		{
			if (!ok)
			{
				hostLogError(err.isEmpty() ? QStringLiteral("create body failed") : err);
				return;
			}
			const QString bodyId = QString::fromStdString(result.newBackendId);
			page->setActiveBodyId(bodyId);
			refreshBodyList(page);
			applyHistoryJsonToBody(page, doc, geo, bodyId, historyUtf8, QByteArray());
		});
}

void GeometricModelingPlugin::onExportHistory()
{
	GeometricModelingPage* page = ensurePageForActiveDocument();
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IPluginGeometryHost* geo = m_host ? m_host->geometryHost() : nullptr;
	if (!page || !doc || !geo || page->activeBodyId().isEmpty())
	{
		hostLogWarn(i18n(QStringLiteral("No active Parametric Body."), QStringLiteral("\u65e0\u6d3b\u52a8 Parametric Body\u3002")));
		return;
	}
	QByteArray hist;
	QString qerr;
	if (!geo->queryParametricBodyHistoryJson(doc, page->activeBodyId().toStdString(), hist, &qerr))
	{
		hostLogError(qerr);
		return;
	}
	const QString path = QFileDialog::getSaveFileName(
		nullptr, i18n(QStringLiteral("Export history JSON"), QStringLiteral("\u5bfc\u51fa\u5386\u53f2 JSON")),
		QStringLiteral("part.cloudsim-part.json"),
		QStringLiteral("History JSON (*.cloudsim-part.json *.json)"));
	if (path.isEmpty())
		return;
	QFile f(path);
	if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate))
	{
		hostLogError(i18n(QStringLiteral("Cannot write file."), QStringLiteral("\u65e0\u6cd5\u5199\u5165\u6587\u4ef6\u3002")));
		return;
	}
	f.write(hist);
	hostLogInfo(i18n(QStringLiteral("Exported: %1"), QStringLiteral("\u5df2\u5bfc\u51fa\uff1a%1")).arg(path));
}

void GeometricModelingPlugin::onImportHistoryReplace()
{
	GeometricModelingPage* page = ensurePageForActiveDocument();
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IPluginGeometryHost* geo = m_host ? m_host->geometryHost() : nullptr;
	if (!page || !doc || !geo || page->activeBodyId().isEmpty())
	{
		hostLogWarn(i18n(QStringLiteral("No active Parametric Body to replace."),
						 QStringLiteral("\u65e0\u6d3b\u52a8 Body\uff0c\u8bf7\u7528\u300c\u5bfc\u5165\u65b0\u5efa\u300d\u3002")));
		return;
	}
	const QString path = QFileDialog::getOpenFileName(
		nullptr, i18n(QStringLiteral("Import history JSON"), QStringLiteral("\u5bfc\u5165\u5386\u53f2 JSON")), QString(),
		QStringLiteral("History JSON (*.cloudsim-part.json *.json)"));
	if (path.isEmpty())
		return;
	QFile f(path);
	if (!f.open(QIODevice::ReadOnly))
	{
		hostLogError(i18n(QStringLiteral("Cannot read file."), QStringLiteral("\u65e0\u6cd5\u8bfb\u53d6\u6587\u4ef6\u3002")));
		return;
	}
	const ScriptModelParseResult parsed = parseScriptModelJson(f.readAll());
	if (parsed.kind != ScriptModelJsonKind::History)
	{
		hostLogError(parsed.error.isEmpty()
						 ? i18n(QStringLiteral("Not a history JSON."), QStringLiteral("\u4e0d\u662f history JSON\u3002"))
						 : parsed.error);
		return;
	}
	QByteArray beforeHist;
	QString qerr;
	(void)geo->queryParametricBodyHistoryJson(doc, page->activeBodyId().toStdString(), beforeHist, &qerr);
	applyHistoryJsonToBody(page, doc, geo, page->activeBodyId(), parsed.payloadUtf8, beforeHist);
}

void GeometricModelingPlugin::onImportHistoryNew()
{
	GeometricModelingPage* page = ensurePageForActiveDocument();
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IPluginGeometryHost* geo = m_host ? m_host->geometryHost() : nullptr;
	if (!page || !doc || !geo)
		return;
	const QString path = QFileDialog::getOpenFileName(
		nullptr, i18n(QStringLiteral("Import history as new body"), QStringLiteral("\u5bfc\u5165\u4e3a\u65b0 Body")), QString(),
		QStringLiteral("History JSON (*.cloudsim-part.json *.json)"));
	if (path.isEmpty())
		return;
	QFile f(path);
	if (!f.open(QIODevice::ReadOnly))
	{
		hostLogError(i18n(QStringLiteral("Cannot read file."), QStringLiteral("\u65e0\u6cd5\u8bfb\u53d6\u6587\u4ef6\u3002")));
		return;
	}
	const ScriptModelParseResult parsed = parseScriptModelJson(f.readAll());
	if (parsed.kind != ScriptModelJsonKind::History)
	{
		hostLogError(parsed.error.isEmpty()
						 ? i18n(QStringLiteral("Not a history JSON."), QStringLiteral("\u4e0d\u662f history JSON\u3002"))
						 : parsed.error);
		return;
	}
	createBodyThenApplyHistory(page, doc, geo, parsed.payloadUtf8);
}

void GeometricModelingPlugin::onRunComposeFile()
{
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IAiAssistantHost* ai = m_host ? m_host->aiAssistantHost() : nullptr;
	if (!doc || !ai)
	{
		hostLogWarn(i18n(QStringLiteral("AI host unavailable."), QStringLiteral("AI \u4e3b\u673a\u4e0d\u53ef\u7528\u3002")));
		return;
	}
	const QString path = QFileDialog::getOpenFileName(
		nullptr, i18n(QStringLiteral("Run feature.compose JSON"), QStringLiteral("\u8fd0\u884c Compose JSON")), QString(),
		QStringLiteral("Compose JSON (*.cloudsim-compose.json *.json)"));
	if (path.isEmpty())
		return;
	QFile f(path);
	if (!f.open(QIODevice::ReadOnly))
	{
		hostLogError(i18n(QStringLiteral("Cannot read file."), QStringLiteral("\u65e0\u6cd5\u8bfb\u53d6\u6587\u4ef6\u3002")));
		return;
	}
	const ScriptModelParseResult parsed = parseScriptModelJson(f.readAll());
	if (parsed.kind != ScriptModelJsonKind::Compose)
	{
		hostLogError(parsed.error.isEmpty()
						 ? i18n(QStringLiteral("Not a feature.compose JSON."),
								QStringLiteral("\u4e0d\u662f feature.compose JSON\u3002"))
						 : parsed.error);
		return;
	}
	QString summary;
	QString err;
	if (!ai->executeActionPlan(parsed.payloadUtf8, &summary, &err))
	{
		hostLogError(err.isEmpty() ? QStringLiteral("executeActionPlan failed") : err);
		return;
	}
	if (GeometricModelingPage* page = ensurePageForActiveDocument())
	{
		refreshBodyList(page);
		syncFeaturesFromBody(page);
	}
	hostLogInfo(summary.isEmpty()
					? i18n(QStringLiteral("Compose finished."), QStringLiteral("Compose \u5df2\u5b8c\u6210\u3002"))
					: summary);
}

void GeometricModelingPlugin::onPythonConsole()
{
	CloudSimGeomPython::setHost(m_host);
	CloudSimGeomPython::openConsole(nullptr);
}

void GeometricModelingPlugin::pushBodyHistoryAfterRollback(GeometricModelingPage* page, const QByteArray& beforeHist)
{
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IPluginGeometryHost* geo = m_host ? m_host->geometryHost() : nullptr;
	if (!doc || !geo || !page || page->activeBodyId().isEmpty())
		return;
	const QByteArray afterHist = page->features().toParametricHistoryJson();
	geo->setParametricBodyHistoryJson(
		doc, page->activeBodyId().toStdString(), afterHist,
		[this, page, doc, beforeHist, afterHist](bool ok, const QString& err, const PluginGeometryJobResult&)
		{
			if (!ok)
			{
				hostLogError(err);
				return;
			}
			syncFeaturesFromBody(page);
			page->commands().execute(std::make_unique<BodyHistoryCmd>(
				m_host, doc, page->activeBodyId(), beforeHist, afterHist,
				[this, page]() { syncFeaturesFromBody(page); }, true));
			const QString rb = page->features().rollbackAfterFeatureId();
			hostLogInfo(rb.isEmpty() ? QStringLiteral("\u5df2\u6e05\u9664\u56de\u9000\u70b9")
									 : QStringLiteral("\u56de\u9000\u81f3: %1").arg(rb));
		});
}

void GeometricModelingPlugin::onFeatureDelete(const QString& featureId)
{
	GeometricModelingPage* page = ensurePageForActiveDocument();
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IPluginGeometryHost* geo = m_host ? m_host->geometryHost() : nullptr;
	if (!page || !doc || !geo || page->activeBodyId().isEmpty() || featureId.isEmpty())
		return;
	if (featureId.startsWith(QStringLiteral("__origin")))
		return;
	if (!page->features().find(featureId))
	{
		hostLogWarn(i18n(QStringLiteral("Feature not found."), QStringLiteral("\u7279\u5f81\u672a\u627e\u5230\u3002")));
		return;
	}

	if (page->activeSketchId() == featureId)
	{
		if (m_sketch.active())
		{
			m_sketch.end();
			page->hideLegendOverlay();
		}
		page->setActiveSketchId(QString());
	}
	if (page->editingFeatureId() == featureId)
	{
		clearExtrudePreviewUi();
		clearSweepPreviewUi();
		page->setEditingFeatureId(QString());
	}

	QByteArray beforeHist;
	QString qerr;
	if (!geo->queryParametricBodyHistoryJson(doc, page->activeBodyId().toStdString(), beforeHist, &qerr))
	{
		hostLogWarn(qerr);
		return;
	}
	if (!page->features().removeFeature(featureId))
	{
		hostLogWarn(i18n(QStringLiteral("Failed to delete feature."), QStringLiteral("\u5220\u9664\u7279\u5f81\u5931\u8d25\u3002")));
		return;
	}
	page->refreshFeatureTree();
	if (!pushFeatureHistory(page, beforeHist))
		return;
	refreshVisibleSketchOverlays(page);
	hostLogInfo(i18n(QStringLiteral("Deleted: %1"), QStringLiteral("\u5df2\u5220\u9664: %1")).arg(featureId));
}

void GeometricModelingPlugin::onFeatureRollback(const QString& featureId)
{
	GeometricModelingPage* page = ensurePageForActiveDocument();
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IPluginGeometryHost* geo = m_host ? m_host->geometryHost() : nullptr;
	if (!page || !doc || !geo || page->activeBodyId().isEmpty() || featureId.isEmpty())
		return;
	if (featureId.startsWith(QStringLiteral("__origin")))
		return;
	if (page->features().rollbackAfterFeatureId() == featureId)
		return;
	QByteArray beforeHist;
	QString qerr;
	if (!geo->queryParametricBodyHistoryJson(doc, page->activeBodyId().toStdString(), beforeHist, &qerr))
	{
		hostLogWarn(qerr);
		return;
	}
	if (!page->features().applyRollbackTo(featureId))
	{
		hostLogWarn(QStringLiteral("\u65e0\u6cd5\u6291\u5236\u8be5\u7279\u5f81"));
		return;
	}
	page->refreshFeatureTree();
	pushBodyHistoryAfterRollback(page, beforeHist);
}

void GeometricModelingPlugin::onExitRollback()
{
	GeometricModelingPage* page = ensurePageForActiveDocument();
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IPluginGeometryHost* geo = m_host ? m_host->geometryHost() : nullptr;
	if (!page || !doc || !geo || page->activeBodyId().isEmpty())
		return;
	if (page->features().rollbackAfterFeatureId().isEmpty())
		return;
	QByteArray beforeHist;
	QString qerr;
	if (!geo->queryParametricBodyHistoryJson(doc, page->activeBodyId().toStdString(), beforeHist, &qerr))
	{
		hostLogWarn(qerr);
		return;
	}
	page->features().clearRollback();
	page->refreshFeatureTree();
	pushBodyHistoryAfterRollback(page, beforeHist);
}

void GeometricModelingPlugin::onEditFeature(const QString& featureId)
{
	GeometricModelingPage* page = ensurePageForActiveDocument();
	if (!page || featureId.isEmpty())
		return;
	if (featureId.startsWith(QStringLiteral("__origin")))
		return;
	const GeomodelingFeature* feature = page->features().find(featureId);
	if (!feature)
	{
		hostLogWarn(i18n(QStringLiteral("Feature not found."), QStringLiteral("\u7279\u5f81\u672a\u627e\u5230\u3002")));
		return;
	}
	if (feature->kind == GeomodelingFeatureKind::Sketch)
		onEditSketch(featureId);
	else if (feature->kind == GeomodelingFeatureKind::Pad || feature->kind == GeomodelingFeatureKind::Pocket)
		onEditExtrudeFeature(featureId);
	else if (feature->kind == GeomodelingFeatureKind::Sweep || feature->kind == GeomodelingFeatureKind::SweepCut)
		onEditSweepFeature(featureId);
	else if (feature->kind == GeomodelingFeatureKind::Fillet)
		beginFilletPanel();
	else if (feature->kind == GeomodelingFeatureKind::Chamfer)
		beginChamferPanel();
	else if (feature->kind == GeomodelingFeatureKind::Revolve)
		beginRevolvePanel(false);
	else if (feature->kind == GeomodelingFeatureKind::RevolveCut)
		beginRevolvePanel(true);
	else if (feature->kind == GeomodelingFeatureKind::LinearPattern)
		beginPatternPanel();
	else if (feature->kind == GeomodelingFeatureKind::CircularPattern)
		beginCircularPatternPanel();
	else if (feature->kind == GeomodelingFeatureKind::Mirror3D)
		beginMirror3dPanel();
	else if (feature->kind == GeomodelingFeatureKind::Loft)
		beginLoftPanel(false);
	else if (feature->kind == GeomodelingFeatureKind::LoftCut)
		beginLoftPanel(true);
	else if (feature->kind == GeomodelingFeatureKind::Shell)
		beginShellPanel();
	else if (feature->kind == GeomodelingFeatureKind::DatumPlane ||
			 feature->kind == GeomodelingFeatureKind::DatumPlaneAngle)
	{
		page->setParamsFeatureId(featureId);
		std::vector<std::pair<QString, double>> rows;
		rows.emplace_back(QStringLiteral("datum.offset"), feature->datumOffsetMm);
		if (feature->kind == GeomodelingFeatureKind::DatumPlaneAngle)
			rows.emplace_back(QStringLiteral("datum.angle"), feature->datumAngleDeg);
		page->showNamedParams(feature->name.isEmpty() ? featureId : feature->name, rows, false);
		hostLogInfo(i18n(QStringLiteral("Edit datum offset/angle in params; use New Sketch to draw on this plane."),
						 QStringLiteral("\u5728\u53c2\u6570\u9875\u4fee\u6539\u504f\u79fb/\u89d2\u5ea6\uff1b\u7528\u300c\u65b0\u5efa\u8349\u56fe\u300d\u5728\u8be5\u9762\u4e0a\u7ed8\u5236\u3002")));
	}
	else
		hostLogInfo(i18n(QStringLiteral("Feature kind has no edit panel yet."),
						 QStringLiteral("\u8be5\u7279\u5f81\u7c7b\u578b\u6682\u65e0\u7f16\u8f91\u9762\u677f\u3002")));
}

void GeometricModelingPlugin::onEditSketch(const QString& sketchId)
{
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IPluginGeometryHost* geo = m_host ? m_host->geometryHost() : nullptr;
	GeometricModelingPage* page = ensurePageForActiveDocument();
	if (!doc || !geo || !page)
		return;

	clearExtrudePreviewUi();
	clearSweepPreviewUi();

	const GeomodelingFeature* feature = page->features().find(sketchId);
	if (!feature || feature->kind != GeomodelingFeatureKind::Sketch)
		return;
	if (feature->sketchDocumentUtf8.isEmpty())
	{
		hostLogWarn(i18n(QStringLiteral("Sketch has no saved document."), QStringLiteral("\u8349\u56fe\u65e0\u5df2\u4fdd\u5b58\u6570\u636e\u3002")));
		return;
	}

	if (m_sketch.active())
	{
		persistActiveSketchDocument(page);
		m_sketch.end();
	}

	page->setActiveSketchId(sketchId);
	QString beginErr;
	if (!m_sketch.beginWithDocument(geo, doc, feature->plane, feature->sketchDocumentUtf8, &beginErr))
	{
		hostLogError(beginErr);
		return;
	}

	m_sketch.setChangeNotifier(
		[this, page]()
		{
			updateDofUi(page);
			hostLogInfo(m_sketch.statusText());
			persistActiveSketchDocument(page);
			if (m_sketch.toolKind() == SketchToolKind::Mirror)
				refreshMirrorPanel(page);
			const int eid = m_sketch.selectedEntityId();
			if (eid >= 0)
			{
				std::vector<std::pair<QString, double>> rows;
				if (m_sketch.readNamedParams(eid, rows))
					page->showNamedParams(m_sketch.entityDisplayName(eid), rows, true);
			}
		});

	page->refreshFeatureTree();
	page->showLegendOverlay();
	page->refreshFeatureTree();
	updateDofUi(page);
	hostLogInfo(i18n(QStringLiteral("Editing sketch from feature tree."), QStringLiteral("\u5df2\u4ece\u7279\u5f81\u6811\u8fdb\u5165\u8349\u56fe\u7f16\u8f91\u3002")));
}

void GeometricModelingPlugin::onEditExtrudeFeature(const QString& featureId)
{
	GeometricModelingPage* page = ensurePageForActiveDocument();
	if (!page)
		return;

	const GeomodelingFeature* feature = page->features().find(featureId);
	if (!feature || (feature->kind != GeomodelingFeatureKind::Pad && feature->kind != GeomodelingFeatureKind::Pocket))
		return;

	const GeomodelingFeature* sketch = page->features().find(feature->sketchRefId);
	if (!sketch || sketch->profileXyzMm.size() < 12)
	{
		hostLogWarn(i18n(QStringLiteral("Sketch profile missing for extrude feature."),
						 QStringLiteral("\u62c9\u4f38\u7279\u5f81\u7f3a\u5c11\u8349\u56fe\u8f6e\u5ed3\u3002")));
		return;
	}

	clearExtrudePreviewUi();
	page->setEditingFeatureId(featureId);
	m_editExtrudeMode = true;
	page->setExtrudeOperationMode(feature->kind == GeomodelingFeatureKind::Pocket);
	page->setExtrudeUi(feature->lengthMm, feature->reversed, feature->endCondition, false, feature->draftAngleDeg,
					   feature->startOffsetMm, feature->length2Mm);
	if (feature->hasUpToFacePlane)
		page->setUpToFacePlane(feature->upToFacePlane, feature->upToFaceBackendId, feature->upToFaceIndex);
	else
		page->clearUpToFacePlane();

	const bool pocket = feature->kind == GeomodelingFeatureKind::Pocket;
	std::vector<PluginSketchSweepPathSegment> profileSegs;
	const auto& srcSegs = !feature->profileSegments.empty() ? feature->profileSegments : sketch->profileSegments;
	for (const auto& s : srcSegs)
	{
		PluginSketchSweepPathSegment p;
		p.kind = static_cast<PluginSketchSweepPathSegKind>(s.kind);
		p.ax = s.ax;
		p.ay = s.ay;
		p.az = s.az;
		p.bx = s.bx;
		p.by = s.by;
		p.bz = s.bz;
		p.mx = s.mx;
		p.my = s.my;
		p.mz = s.mz;
		profileSegs.push_back(p);
	}
	if (profileSegs.empty() && !sketch->sketchDocumentUtf8.isEmpty())
	{
		SketchDocument2d doc;
		if (doc.fromJsonUtf8(sketch->sketchDocumentUtf8))
			(void)doc.exportClosedProfileSegments(sketch->plane, profileSegs, nullptr);
	}
	beginExtrudePreviewFromProfile(pocket, sketch->profileXyzMm, sketch->plane, sketch->profileHolesXyzMm, profileSegs);
	if (!m_previewActive)
	{
		m_editExtrudeMode = false;
		page->setEditingFeatureId(QString());
	}
}

void GeometricModelingPlugin::rebuildDownstreamAfterSketch(GeometricModelingPage* page)
{
	if (!page || page->activeBodyId().isEmpty())
		return;

	const QString sketchId = page->activeSketchId();
	if (sketchId.isEmpty())
		return;

	bool hasDownstream = false;
	QStringList sweepIds;
	for (const GeomodelingFeature& feature : page->features().features())
	{
		if ((feature.kind == GeomodelingFeatureKind::Pad || feature.kind == GeomodelingFeatureKind::Pocket)
			&& feature.sketchRefId == sketchId)
		{
			hasDownstream = true;
		}
		if ((feature.kind == GeomodelingFeatureKind::Sweep || feature.kind == GeomodelingFeatureKind::SweepCut)
			&& (feature.sketchRefId == sketchId || feature.pathSketchRefId == sketchId))
		{
			hasDownstream = true;
			sweepIds.append(feature.id);
		}
	}
	for (const QString& fid : sweepIds)
	{
		GeomodelingFeature* feature = page->features().find(fid);
		if (!feature)
			continue;
		// 从轮廓草图回填 profile；无 pathSegments 时兜底
		if (GeomodelingFeature* profileSk = page->features().find(feature->sketchRefId))
		{
			QString err;
			std::vector<float> profile;
			if (loadSketchPolyline(*profileSk, false, profile, &err) && profile.size() >= 12)
				feature->profileXyzMm = profile;
		}
		if (GeomodelingFeature* pathSk = page->features().find(feature->pathSketchRefId))
		{
			QString err;
			std::vector<float> path;
			std::vector<PluginSketchSweepPathSegment> segs;
			if (loadSketchPathSegments(*pathSk, segs, &err) && !segs.empty())
			{
				feature->pathSegments.clear();
				for (const auto& p : segs)
				{
					GeomodelingFeature::PathSegment s;
					s.kind = (p.kind == PluginSketchSweepPathSegKind::Arc)			? 1
				 : (p.kind == PluginSketchSweepPathSegKind::SplineThrough) ? 2
																		  : 0;
					s.ax = p.ax;
					s.ay = p.ay;
					s.az = p.az;
					s.bx = p.bx;
					s.by = p.by;
					s.bz = p.bz;
					s.mx = p.mx;
					s.my = p.my;
					s.mz = p.mz;
					feature->pathSegments.push_back(s);
				}
			}
			else
			{
				feature->pathSegments.clear();
			}
			if (loadSketchPolyline(*pathSk, true, path, &err) && path.size() >= 6)
				feature->pathXyzMm = path;
			else if (!feature->pathSegments.empty())
			{
				feature->pathXyzMm.clear();
				for (const auto& s : feature->pathSegments)
				{
					if (feature->pathXyzMm.empty())
					{
						feature->pathXyzMm.push_back(s.ax);
						feature->pathXyzMm.push_back(s.ay);
						feature->pathXyzMm.push_back(s.az);
					}
					feature->pathXyzMm.push_back(s.bx);
					feature->pathXyzMm.push_back(s.by);
					feature->pathXyzMm.push_back(s.bz);
				}
			}
		}
	}
	if (!hasDownstream)
		return;

	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IPluginGeometryHost* geo = m_host ? m_host->geometryHost() : nullptr;
	if (!doc || !geo)
		return;

	QByteArray beforeHist;
	QString qerr;
	if (!geo->queryParametricBodyHistoryJson(doc, page->activeBodyId().toStdString(), beforeHist, &qerr))
	{
		hostLogWarn(qerr);
		return;
	}

	pushFeatureHistory(page, beforeHist);
	hostLogInfo(i18n(QStringLiteral("Rebuilt downstream features after sketch edit."),
					 QStringLiteral("\u8349\u56fe\u4fee\u6539\u540e\u5df2\u91cd\u5efa\u540e\u7eed\u7279\u5f81\u3002")));
}

void GeometricModelingPlugin::onPickUpToFace()
{
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IPluginGeometryHost* geo = m_host ? m_host->geometryHost() : nullptr;
	GeometricModelingPage* page = ensurePageForActiveDocument();
	if (!doc || !geo || !page)
		return;

	PluginGeometryElementPickRequest req;
	req.kind = PluginGeometryElementKind::Face;
	geo->pickStepElementFromViewport(
		doc, req,
		[this, page, geo, doc](bool ok, const QString& err, const PluginGeometryStepRef& ref)
		{
			if (!ok)
			{
				if (!err.isEmpty())
					hostLogInfo(err);
				return;
			}
			PluginSketchPlane plane;
			QString planeErr;
			if (!geo->queryFaceSketchPlane(doc, ref, plane, &planeErr))
			{
				hostLogWarn(planeErr.isEmpty()
								? i18n(QStringLiteral("Face is not planar."), QStringLiteral("\u9762\u4e0d\u662f\u5e73\u9762\u3002"))
								: planeErr);
				return;
			}
			page->setUpToFacePlane(plane, QString::fromStdString(ref.backendIdUtf8), ref.faceIndex);
		});
}

void GeometricModelingPlugin::onPickUpToVertex()
{
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IPluginGeometryHost* geo = m_host ? m_host->geometryHost() : nullptr;
	GeometricModelingPage* page = ensurePageForActiveDocument();
	if (!doc || !geo || !page)
		return;

	PluginGeometryElementPickRequest req;
	req.kind = PluginGeometryElementKind::Vertex;
	hostLogInfo(i18n(QStringLiteral("Pick near a vertex (edge endpoint)."),
					 QStringLiteral("\u8bf7\u5728\u9876\u70b9\u9644\u8fd1\u70b9\u9009\u8fb9\uff08\u5438\u9644\u5230\u8fd1\u7aef\u70b9\uff09\u3002")));
	geo->pickStepElementFromViewport(
		doc, req,
		[this, page](bool ok, const QString& err, const PluginGeometryStepRef& ref)
		{
			if (!ok)
			{
				if (!err.isEmpty())
					hostLogInfo(err);
				return;
			}
			if (!ref.hasHitPoint)
			{
				hostLogWarn(i18n(QStringLiteral("Vertex pick has no hit point."),
								 QStringLiteral("\u9876\u70b9\u62fe\u53d6\u672a\u8fd4\u56de\u5750\u6807\u3002")));
				return;
			}
			page->setUpToVertex(ref.hitWorldMm, ref.vertexIndex, QString::fromStdString(ref.backendIdUtf8));
			hostLogInfo(i18n(QStringLiteral("Up-to vertex set."), QStringLiteral("\u5df2\u8bbe\u7f6e\u5230\u9876\u70b9\u3002")));
		});
}

void GeometricModelingPlugin::onPickSweepEdgePath()
{
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IPluginGeometryHost* geo = m_host ? m_host->geometryHost() : nullptr;
	GeometricModelingPage* page = ensurePageForActiveDocument();
	if (!doc || !geo || !page || !m_sweepPreviewActive)
		return;

	PluginGeometryElementPickRequest req;
	req.kind = PluginGeometryElementKind::Edge;
	geo->pickStepElementFromViewport(
		doc, req,
		[this, page, geo, doc](bool ok, const QString& err, const PluginGeometryStepRef& ref)
		{
			if (!ok)
			{
				if (!err.isEmpty())
					hostLogInfo(err);
				return;
			}
			const std::string path = ref.stepPathUtf8.empty() ? ref.backendIdUtf8 : ref.stepPathUtf8;
			PluginMeshDiscretizeParams meshParams;
			geo->discretizeBackendEdgesToPolylines(
				doc, path, meshParams,
				[this, page](bool ok2, const QString& err2, const PluginGeometryJobResult& result)
				{
					if (!ok2 || result.polylines.empty())
					{
						hostLogWarn(err2.isEmpty() ? i18n(QStringLiteral("Edge discretize failed."),
														  QStringLiteral("\u8fb9\u79bb\u6563\u5316\u5931\u8d25\u3002"))
												   : err2);
						return;
					}
					// MVP：取第一条可用折线
					for (const auto& pl : result.polylines)
					{
						if (pl.size() >= 6)
						{
							m_sweepPath = pl;
							m_sweepPathSegments.clear();
							m_sweepPathFromEdge = true;
							page->setSweepStatus(i18n(QStringLiteral("Model edge path set (%1 points)."),
													  QStringLiteral("\u5df2\u8bbe\u6a21\u578b\u8fb9\u8def\u5f84\uff08%1 \u70b9\uff09\u3002"))
											   .arg(static_cast<int>(pl.size() / 3)));
							refreshSweepPreview();
							return;
						}
					}
					hostLogWarn(i18n(QStringLiteral("Edge too short."), QStringLiteral("\u8fb9\u8fc7\u77ed\u3002")));
				});
		});
}

void GeometricModelingPlugin::onActiveBodyChanged(const QString& bodyId)
{
	GeometricModelingPage* page = ensurePageForActiveDocument();
	if (!page)
		return;

	clearExtrudePreviewUi();
	clearSweepPreviewUi();
	clearSolidFeaturePreviewUi();
	if (m_sketch.active())
	{
		persistActiveSketchDocument(page);
		m_sketch.end();
		page->hideLegendOverlay();
	}

	page->setActiveBodyId(bodyId);
	syncFeaturesFromBody(page);
}

void GeometricModelingPlugin::onViewportEditPick()
{
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IPluginGeometryHost* geo = m_host ? m_host->geometryHost() : nullptr;
	GeometricModelingPage* page = ensurePageForActiveDocument();
	if (!doc || !geo || !page)
		return;

	hostLogInfo(i18n(QStringLiteral("Click a face to pick a feature for edit."),
					 QStringLiteral("\u8bf7\u70b9\u9009\u9762\u4ee5\u9009\u62e9\u8981\u7f16\u8f91\u7684\u7279\u5f81\u3002")));

	geo->pickParametricFeatureForEdit(
		doc,
		[this, page](bool ok, const QString& err, const QString& backendId, const QString& suggestedFeatureId)
		{
			if (!ok)
			{
				if (!err.isEmpty())
					hostLogInfo(err);
				return;
			}

			page->setActiveBodyId(backendId);
			syncFeaturesFromBody(page);

			if (!suggestedFeatureId.isEmpty())
			{
				onEditFeature(suggestedFeatureId);
				return;
			}

			QMenu menu;
			for (const GeomodelingFeature& feature : page->features().features())
			{
				if (feature.suppressed)
					continue;
				QAction* act = menu.addAction(feature.name.isEmpty() ? feature.id : feature.name);
				act->setData(feature.id);
			}
			if (QAction* chosen = menu.exec(QCursor::pos()))
				onEditFeature(chosen->data().toString());
		});
}

void GeometricModelingPlugin::onProjectAboutToSave(const QString& documentId, QJsonObject& root)
{
	GeometricModelingPage* page = m_pages.value(documentId, nullptr);
	if (!page)
		return;
	QJsonObject gm;
	gm.insert(QStringLiteral("activeBodyId"), page->activeBodyId());
	// 基准面等不进 Parametric tip 的节点，写入工程侧车
	gm.insert(QStringLiteral("features"), page->features().toJson().value(QStringLiteral("features")));
	gm.insert(QStringLiteral("featureSeq"), page->features().toJson().value(QStringLiteral("seq")));
	QJsonObject originVis;
	originVis.insert(QStringLiteral("point"), page->originPointVisible());
	originVis.insert(QStringLiteral("xy"), page->originPlaneXyVisible());
	originVis.insert(QStringLiteral("xz"), page->originPlaneXzVisible());
	originVis.insert(QStringLiteral("yz"), page->originPlaneYzVisible());
	gm.insert(QStringLiteral("originVisibility"), originVis);
	root.insert(QLatin1String(backend_type::kProjectKeyGeometricModeling), gm);
}

void GeometricModelingPlugin::onProjectLoaded(const QString& documentId, const QJsonObject& root)
{
	const QJsonObject gm = root.value(QLatin1String(backend_type::kProjectKeyGeometricModeling)).toObject();
	if (gm.isEmpty())
		return;
	GeometricModelingPage* page = m_pages.value(documentId, nullptr);
	if (!page)
		page = ensurePageForActiveDocument();
	if (!page)
		return;
	const QString bodyId = gm.value(QStringLiteral("activeBodyId")).toString();
	page->setActiveBodyId(bodyId);

	if (gm.contains(QStringLiteral("originVisibility")))
	{
		const QJsonObject ov = gm.value(QStringLiteral("originVisibility")).toObject();
		// 通过树切换接口的反面：直接写成员较难，用 toggle 对齐太脆；暴露 restore
		page->restoreOriginVisibility(ov.value(QStringLiteral("point")).toBool(true),
									  ov.value(QStringLiteral("xy")).toBool(true),
									  ov.value(QStringLiteral("xz")).toBool(true),
									  ov.value(QStringLiteral("yz")).toBool(true));
	}

	if (!bodyId.isEmpty())
		syncFeaturesFromBody(page);
	else
		refreshBodyList(page);

	// Body sync 之后再合并工程里保存的 DatumPlane
	if (gm.contains(QStringLiteral("features")))
	{
		QJsonObject wrap;
		wrap.insert(QStringLiteral("features"), gm.value(QStringLiteral("features")));
		wrap.insert(QStringLiteral("seq"), gm.value(QStringLiteral("featureSeq")).toInt(1));
		FeatureDocument side;
		side.fromJson(wrap);
		for (const GeomodelingFeature& f : side.features())
		{
			if (f.kind != GeomodelingFeatureKind::DatumPlane && f.kind != GeomodelingFeatureKind::DatumPlaneAngle)
				continue;
			if (page->features().find(f.id))
				continue;
			page->features().appendPreserved(f);
		}
		page->refreshFeatureTree();
	}

	refreshVisibleSketchOverlays(page);
	applyOriginReferenceVisibility(page);
}
