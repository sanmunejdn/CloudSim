#ifndef GEOMETRICMODELINGPLUGIN_GEOMETRICMODELINGPLUGIN_H
#define GEOMETRICMODELINGPLUGIN_GEOMETRICMODELINGPLUGIN_H

/// @file GeometricModelingPlugin.h

#include "ICloudSimPlugin.h"
#include "PluginGeometryTypes.h"
#include "FeatureDocument.h"
#include "GeometricModelingPage.h"
#include "SketchEditSession.h"
#include "SketchTools.h"

#include <QHash>
#include <QJsonObject>
#include <QObject>
#include <memory>
#include <vector>

class IPluginHostContext;
class QAction;
class QMenu;
class GeometricModelingPage;
class GeometricModelingRibbonBar;

class GeometricModelingPlugin : public QObject, public ICloudSimPlugin
{
	Q_OBJECT
	Q_PLUGIN_METADATA(IID CloudSimPlugin_iid)
	Q_INTERFACES(ICloudSimPlugin)

public:
	GeometricModelingPlugin() = default;
	~GeometricModelingPlugin() override;

	QString pluginId() const override;
	QString displayName() const override;
	bool initialize(IPluginHostContext* host) override;
	void shutdown() override;

private:
	void registerMenus();
	void applyLanguage();
	QString i18n(const QString& en, const QString& zh) const;
	bool useChinese() const;
	void ensureRibbon();
	void enterGeometricModeling();
	void exitGeometricModeling();
	void softExitMode();
	GeometricModelingPage* ensurePageForActiveDocument();
	void applyOriginReferenceVisibility(GeometricModelingPage* page);
	void clearOriginReferenceVisibility();
	void onOriginVisibilityChanged();
	void onNewSketch();
	void onDatumPlane();
	void onOriginPlaneSketchRequested(int planeIndex);
	void onFixPointToOriginRequested();
	void beginSketchOnPlane(GeometricModelingPage* page, IPluginGeometryHost* geo, IPluginDocument* doc,
							const PluginSketchPlane& plane);
	void onEndSketch();
	void onEditFeature(const QString& featureId);
	void onEditSketch(const QString& sketchId);
	void onEditExtrudeFeature(const QString& featureId);
	void onPickUpToFace();
	void onPickUpToVertex();
	void onActiveBodyChanged(const QString& bodyId);
	void onViewportEditPick();
	void refreshBodyList(GeometricModelingPage* page);
	bool pushFeatureHistory(GeometricModelingPage* page, const QByteArray& beforeHist);
	void rebuildDownstreamAfterSketch(GeometricModelingPage* page);
	void fillExtrudeParams(GeometricModelingPage* page, PluginSketchExtrudeParams& params) const;
	void beginExtrudePreviewFromProfile(bool pocket, const std::vector<float>& profile, const PluginSketchPlane& plane,
										const std::vector<std::vector<float>>& holes = {});
	void onToolLine();
	void onToolArc();
	void onToolCircle();
	void onToolRect();
	void onToolEllipse();
	void onToolPolygon();
	void onToolSlot();
	void onToolSpline();
	void onDimLength();
	void onDimDistance();
	void onDimRadius();
	void onDimAngle();
	void onDimArcRadius();
	void onToggleConstruction();
	void onGeomHorizontal();
	void onGeomVertical();
	void onGeomCoincident();
	void onGeomParallel();
	void onGeomPerpendicular();
	void onGeomEqualLength();
	void onGeomTangent();
	void onGeomSymmetric();
	void onGeomMidpoint();
	void onGeomFix();
	void onGeomFixOrigin();
	void onTrim();
	void onMirror();
	void onDelete();
	void onProjectEdges();
	void onConvertEntities();
	void onOffset();
	void onSolve();
	void onPad();
	void onPocket();
	void onSweep();
	void onSweepCut();
	void onFillet();
	void onChamfer();
	void onRevolve();
	void onRevolveCut();
	void onLinearPattern();
	void onMirror3d();
	void onLoft();
	void onLoftCut();
	void onShell();
	void onRebuild();
	void onUndo();
	void onRedo();
	void onFeatureRollback(const QString& featureId);
	void onExitRollback();
	void onFeatureDelete(const QString& featureId);
	void pushBodyHistoryAfterRollback(GeometricModelingPage* page, const QByteArray& beforeHist);
	void onLengthEdited(double mm);
	void onExtrudeOptionsChanged();
	void onConfirmExtrude();
	void onCancelExtrude();
	void beginExtrudePreview(bool pocket);
	void refreshExtrudePreview();
	void clearExtrudePreviewUi();
	void commitExtrude();
	void commitEditExtrude();
	void beginSweepPanel(bool cut);
	void refreshSweepPreview();
	void clearSweepPreviewUi();
	void onConfirmSweep();
	void onCancelSweep();
	void onPickSweepProfile();
	void onPickSweepPath();
	void onPickSweepEdgePath();
	void onEditSweepFeature(const QString& featureId);
	void commitSweep();
	void commitEditSweep();
	void clearSolidFeaturePreviewUi();
	void beginFilletPanel();
	void onPickFilletEdge();
	void refreshFilletPreview();
	void onConfirmFillet();
	void onCancelFillet();
	void beginChamferPanel();
	void onPickChamferEdge();
	void refreshChamferPreview();
	void onConfirmChamfer();
	void onCancelChamfer();
	void beginRevolvePanel(bool cut);
	void refreshRevolvePreview();
	void onConfirmRevolve();
	void onCancelRevolve();
	void beginPatternPanel();
	void refreshPatternPreview();
	void onConfirmPattern();
	void onCancelPattern();
	void beginMirror3dPanel();
	void refreshMirror3dPreview();
	void onConfirmMirror3d();
	void onCancelMirror3d();
	void beginLoftPanel(bool cut);
	void refreshLoftPreview();
	void onConfirmLoft();
	void onCancelLoft();
	void beginShellPanel();
	void onPickShellFace();
	void refreshShellPreview();
	void onConfirmShell();
	void onCancelShell();
	void beginDraftPanel();
	void onPickDraftFace();
	void onPickDraftNeutral();
	void refreshDraftPreview();
	void onConfirmDraft();
	void onCancelDraft();
	void onDraft();
	bool loadSketchPolyline(const GeomodelingFeature& sketch, bool asPath, std::vector<float>& out, QString* err) const;
	bool loadSketchPathSegments(const GeomodelingFeature& sketch, std::vector<PluginSketchSweepPathSegment>& out,
								QString* err) const;
	bool validateSweepPathAtProfileCenter(const std::vector<float>& profileXyzMm,
										  const std::vector<PluginSketchSweepPathSegment>& pathSegs,
										  const std::vector<float>& pathXyzMm, QString* err) const;
	PluginSketchPlane originMirrorPlane(int planeIndex) const;
	void persistActiveSketchDocument(GeometricModelingPage* page);
	void appendVisibleSketchOverlays(GeometricModelingPage* page, const QString& excludeSketchId,
									 std::vector<PluginSketchOverlaySegment>& out) const;
	void refreshVisibleSketchOverlays(GeometricModelingPage* page);
	void onToggleSketchVisibility(const QString& featureId);
	void updateDofUi(GeometricModelingPage* page);
	void syncFeaturesFromBody(GeometricModelingPage* page);
	void onProjectAboutToSave(const QString& documentId, QJsonObject& root);
	void onProjectLoaded(const QString& documentId, const QJsonObject& root);
	void setActiveTool(SketchToolKind kind);
	void refreshMirrorPanel(GeometricModelingPage* page);
	void onMirrorConfirm();
	void onMirrorCancel();
	void hostLogInfo(const QString& msg);
	void hostLogWarn(const QString& msg);
	void hostLogError(const QString& msg);

	IPluginHostContext* m_host = nullptr;
	QMenu* m_menu = nullptr;
	QAction* m_enterAction = nullptr;
	QAction* m_exitAction = nullptr;
	bool m_inMode = false;
	QHash<QString, GeometricModelingPage*> m_pages;
	GeometricModelingRibbonBar* m_ribbon = nullptr;
	SketchEditSession m_sketch;

	bool m_previewActive = false;
	bool m_previewPocket = false;
	bool m_editExtrudeMode = false;
	std::vector<float> m_previewProfile;
	std::vector<std::vector<float>> m_previewHoles;
	PluginSketchPlane m_previewPlane{};

	bool m_sweepPreviewActive = false;
	bool m_sweepCut = false;
	bool m_editSweepMode = false;
	std::vector<float> m_sweepProfile;
	std::vector<float> m_sweepPath;
	std::vector<PluginSketchSweepPathSegment> m_sweepPathSegments;
	bool m_sweepPathFromEdge = false;

	SideToolPanel m_solidPanel = SideToolPanel::None;
	bool m_solidPreviewActive = false;
	bool m_solidCutMode = false;
	std::vector<int> m_pickedEdgeIndices;
	std::vector<int> m_pickedFaceIndices;
	std::vector<float> m_revolveProfile;
	PluginSketchPlane m_revolvePlane{};
	std::vector<float> m_loftProfileA;
	std::vector<float> m_loftProfileB;
};

#endif
