#ifndef GEOMETRICMODELINGPLUGIN_GEOMETRICMODELINGPAGE_H
#define GEOMETRICMODELINGPLUGIN_GEOMETRICMODELINGPAGE_H

/// @file GeometricModelingPage.h
/// @brief 建模会话数据 + 左侧特征树/属性（3D 公用主视口，本页不展示）

#include "CommandStack.h"
#include "FeatureDocument.h"

#include <QString>
#include <QStringList>
#include <QWidget>
#include <memory>
#include <utility>
#include <vector>

class IPluginHostContext;
class QTreeWidget;
class QDoubleSpinBox;
class QPushButton;
class QLabel;
class QListWidget;
class QCheckBox;
class QComboBox;
class QStackedWidget;
class QVBoxLayout;

enum class SideToolPanel
{
	None = 0,
	Extrude,
	Mirror,
	Trim,
	Sweep,
	Fillet,
	Chamfer,
	Revolve,
	Pattern,
	Mirror3D,
	Loft,
	Shell,
	Draft
};

class GeometricModelingPage : public QWidget
{
	Q_OBJECT
public:
	explicit GeometricModelingPage(IPluginHostContext* host, QWidget* parent = nullptr);

	QWidget* featureTreePanel() const;
	double extrudeLengthMm() const;
	double extrudeDraftAngleDeg() const;
	bool extrudeReversed() const;
	bool extrudeCreateNewBody() const;
	GeomodelingExtrudeEnd extrudeEndCondition() const;
	PluginSketchPlane upToFacePlane() const { return m_upToFacePlane; }
	bool hasUpToFacePlane() const { return m_hasUpToFacePlane; }
	PluginPoint3d upToVertex() const { return m_upToVertex; }
	bool hasUpToVertex() const { return m_hasUpToVertex; }
	double offsetFromFaceMm() const;
	void setUpToVertex(const PluginPoint3d& v);
	void clearUpToVertex();
	QString upToFaceBackendId() const { return m_upToFaceBackendId; }
	int upToFaceIndex() const { return m_upToFaceIndex; }
	void setUpToFacePlane(const PluginSketchPlane& plane, const QString& backendId = QString(), int faceIndex = -1);
	void clearUpToFacePlane();

	void setExtrudeUi(double lengthMm, bool reversed, GeomodelingExtrudeEnd end, bool createNewBody,
					  double draftAngleDeg = 0.0);
	void setExtrudeOperationMode(bool pocket);
	QString editingFeatureId() const { return m_editingFeatureId; }
	void setEditingFeatureId(const QString& id) { m_editingFeatureId = id; }

	FeatureDocument& features() { return m_features; }
	CommandStack& commands() { return *m_commands; }
	QString activeBodyId() const { return m_activeBodyId; }
	void setActiveBodyId(const QString& id);
	void setBodyIdList(const QStringList& ids);
	QStringList bodyIdList() const { return m_bodyIds; }
	QString activeSketchId() const { return m_activeSketchId; }
	void setActiveSketchId(const QString& id) { m_activeSketchId = id; }
	QString selectedFeatureId() const { return m_selectedFeatureId; }

	void refreshFeatureTree();
	void setSideToolPanel(SideToolPanel panel);
	void setExtrudePreviewUi(bool active);
	void setSweepPreviewUi(bool active, bool cutMode);
	void fillSweepSketchCombos(const QString& selectedProfileId = QString(), const QString& selectedPathId = QString());
	QString sweepProfileSketchId() const;
	QString sweepPathSketchId() const;
	double sweepTwistDeg() const;
	void setSweepStatus(const QString& text);
	void selectSweepProfileSketch(const QString& id);
	void selectSweepPathSketch(const QString& id);

	void setFilletUi(bool active);
	void setFilletEdgeCount(int n);
	double filletRadiusMm() const;

	void setChamferUi(bool active);
	void setChamferEdgeCount(int n);
	double chamferDistanceMm() const;

	void setRevolveUi(bool active, bool cutMode);
	void fillRevolveSketchCombo(const QString& selectedId = QString());
	QString revolveSketchId() const;
	double revolveAngleDeg() const;
	void setRevolveStatus(const QString& text);

	void setPatternUi(bool active);
	void fillPatternSourceCombo();
	int patternCount() const;
	double patternDxMm() const;
	double patternDyMm() const;
	double patternDzMm() const;
	QString patternSourceFeatureId() const;

	void setMirror3dUi(bool active);
	bool mirror3dKeepOriginal() const;
	int mirror3dPlaneIndex() const;

	void setLoftUi(bool active, bool cutMode);
	void fillLoftSketchCombos(const QString& selectedA = QString(), const QString& selectedB = QString());
	QString loftSketchAId() const;
	QString loftSketchBId() const;
	void setLoftStatus(const QString& text);

	void setShellUi(bool active);
	double shellThicknessMm() const;
	void setShellFaceCount(int n);
	void setShellStatus(const QString& text);

	void setDraftUi(bool active);
	double draftAngleDeg() const;
	void setDraftFaceCount(int n);
	void setDraftStatus(const QString& text);
	bool hasDraftNeutralPlane() const { return m_hasDraftNeutral; }
	PluginSketchPlane draftNeutralPlane() const { return m_draftNeutralPlane; }
	void setDraftNeutralPlane(const PluginSketchPlane& plane);
	void clearDraftNeutralPlane();

	bool originPointVisible() const { return m_originPointVisible; }
	bool originPlaneXyVisible() const { return m_originXyVisible; }
	bool originPlaneXzVisible() const { return m_originXzVisible; }
	bool originPlaneYzVisible() const { return m_originYzVisible; }
	bool originNodeVisible(const QString& syntheticId) const;
	void toggleOriginVisibility(const QString& syntheticId);
	void restoreOriginVisibility(bool point, bool xy, bool xz, bool yz);
	void updateMirrorPanel(int axisId, const QString& axisText, const std::vector<std::pair<int, QString>>& entities,
						   bool pickingAxis, bool canConfirm);
	void setTrimHint(const QString& text);
	void applyLanguage(bool useChinese);

	void showLegendOverlay();
	void hideLegendOverlay();

signals:
	void lengthEdited(double mm);
	void extrudeOptionsChanged();
	void confirmExtrudeRequested();
	void cancelExtrudeRequested();
	void confirmSweepRequested();
	void cancelSweepRequested();
	void sweepSelectionChanged();
	void pickSweepProfileRequested();
	void pickSweepPathRequested();
	void pickSweepEdgePathRequested();
	void pickUpToFaceRequested();
	void pickUpToVertexRequested();
	void pickFilletEdgeRequested();
	void filletConfirmRequested();
	void filletCancelRequested();
	void pickChamferEdgeRequested();
	void chamferConfirmRequested();
	void chamferCancelRequested();
	void revolveConfirmRequested();
	void revolveCancelRequested();
	void revolveSelectionChanged();
	void patternConfirmRequested();
	void patternCancelRequested();
	void patternOptionsChanged();
	void mirror3dConfirmRequested();
	void mirror3dCancelRequested();
	void mirror3dOptionsChanged();
	void loftConfirmRequested();
	void loftCancelRequested();
	void loftSelectionChanged();
	void shellPickFaceRequested();
	void shellConfirmRequested();
	void shellCancelRequested();
	void draftPickFaceRequested();
	void draftPickNeutralRequested();
	void draftConfirmRequested();
	void draftCancelRequested();
	void featureEditRequested(const QString& featureId);
	void featureDeleteRequested(const QString& featureId);
	void sketchVisibilityToggleRequested(const QString& featureId);
	void viewportFeaturePickRequested();
	void featureRollbackRequested(const QString& featureId);
	void exitRollbackRequested();
	void activeBodyChanged(const QString& bodyId);
	void mirrorConfirmRequested();
	void mirrorCancelRequested();
	void mirrorPickAxisRequested();
	void mirrorPickEntitiesRequested();
	void mirrorClearEntitiesRequested();
	void mirrorRemoveEntityRequested(int entityId);
	void originPlaneSketchRequested(int planeIndex);
	void fixPointToOriginRequested();
	void originVisibilityChanged();

protected:
	bool eventFilter(QObject* watched, QEvent* event) override;

private:
	void buildLegendPanel();
	void rebuildLegendContent();
	void repositionLegend();
	QWidget* findViewportHost() const;
	QString i18n(const QString& en, const QString& zh) const;
	void syncExtrudeEndUi();
	void fillClosedSketchCombo(QComboBox* combo, const QString& selectedId);

	IPluginHostContext* m_host = nullptr;
	bool m_useChinese = true;
	QWidget* m_sidePanel = nullptr;
	QWidget* m_legendPanel = nullptr;
	QWidget* m_legendAnchor = nullptr;
	QVBoxLayout* m_legendBodyLay = nullptr;
	QLabel* m_emptyHint = nullptr;
	QLabel* m_extrudeTitle = nullptr;
	QLabel* m_bodyCaption = nullptr;
	QComboBox* m_bodyCombo = nullptr;
	QLabel* m_mirrorTitle = nullptr;
	QLabel* m_mirrorAxisCaption = nullptr;
	QLabel* m_mirrorEntCaption = nullptr;
	QLabel* m_trimTitle = nullptr;
	QTreeWidget* m_tree = nullptr;
	QStackedWidget* m_toolStack = nullptr;
	QWidget* m_pageEmpty = nullptr;
	QWidget* m_pageExtrude = nullptr;
	QWidget* m_pageMirror = nullptr;
	QWidget* m_pageTrim = nullptr;
	QWidget* m_pageSweep = nullptr;
	QWidget* m_pageFillet = nullptr;
	QWidget* m_pageChamfer = nullptr;
	QWidget* m_pageRevolve = nullptr;
	QWidget* m_pagePattern = nullptr;
	QWidget* m_pageMirror3d = nullptr;
	QWidget* m_pageLoft = nullptr;
	QWidget* m_pageShell = nullptr;
	QWidget* m_pageDraft = nullptr;
	QLabel* m_sweepTitle = nullptr;
	QLabel* m_sweepProfileCaption = nullptr;
	QLabel* m_sweepPathCaption = nullptr;
	QComboBox* m_sweepProfileCombo = nullptr;
	QComboBox* m_sweepPathCombo = nullptr;
	QPushButton* m_btnPickSweepProfile = nullptr;
	QPushButton* m_btnPickSweepPath = nullptr;
	QPushButton* m_btnPickSweepEdgePath = nullptr;
	QDoubleSpinBox* m_sweepTwist = nullptr;
	QPushButton* m_btnSweepOk = nullptr;
	QPushButton* m_btnSweepCancel = nullptr;
	QLabel* m_sweepStatus = nullptr;
	bool m_sweepCutMode = false;
	QLabel* m_filletTitle = nullptr;
	QDoubleSpinBox* m_filletRadius = nullptr;
	QLabel* m_filletEdgeCount = nullptr;
	QPushButton* m_btnPickFilletEdge = nullptr;
	QPushButton* m_btnFilletOk = nullptr;
	QPushButton* m_btnFilletCancel = nullptr;
	QLabel* m_chamferTitle = nullptr;
	QDoubleSpinBox* m_chamferDist = nullptr;
	QLabel* m_chamferEdgeCount = nullptr;
	QPushButton* m_btnPickChamferEdge = nullptr;
	QPushButton* m_btnChamferOk = nullptr;
	QPushButton* m_btnChamferCancel = nullptr;
	QLabel* m_revolveTitle = nullptr;
	QComboBox* m_revolveSketchCombo = nullptr;
	QDoubleSpinBox* m_revolveAngle = nullptr;
	QLabel* m_revolveStatus = nullptr;
	QPushButton* m_btnRevolveOk = nullptr;
	QPushButton* m_btnRevolveCancel = nullptr;
	bool m_revolveCutMode = false;
	QLabel* m_patternTitle = nullptr;
	QComboBox* m_patternSource = nullptr;
	QDoubleSpinBox* m_patternCount = nullptr;
	QDoubleSpinBox* m_patternDx = nullptr;
	QDoubleSpinBox* m_patternDy = nullptr;
	QDoubleSpinBox* m_patternDz = nullptr;
	QPushButton* m_btnPatternOk = nullptr;
	QPushButton* m_btnPatternCancel = nullptr;
	QLabel* m_mirror3dTitle = nullptr;
	QComboBox* m_mirror3dPlane = nullptr;
	QCheckBox* m_mirror3dKeep = nullptr;
	QPushButton* m_btnMirror3dOk = nullptr;
	QPushButton* m_btnMirror3dCancel = nullptr;
	QLabel* m_loftTitle = nullptr;
	QComboBox* m_loftSketchA = nullptr;
	QComboBox* m_loftSketchB = nullptr;
	QLabel* m_loftStatus = nullptr;
	QPushButton* m_btnLoftOk = nullptr;
	QPushButton* m_btnLoftCancel = nullptr;
	bool m_loftCutMode = false;
	QLabel* m_shellTitle = nullptr;
	QDoubleSpinBox* m_shellThickness = nullptr;
	QLabel* m_shellFaceCount = nullptr;
	QPushButton* m_btnPickShellFace = nullptr;
	QLabel* m_shellStatus = nullptr;
	QPushButton* m_btnShellOk = nullptr;
	QPushButton* m_btnShellCancel = nullptr;
	QLabel* m_draftTitle = nullptr;
	QDoubleSpinBox* m_draftAngle = nullptr;
	QLabel* m_draftFaceCount = nullptr;
	QPushButton* m_btnPickDraftFace = nullptr;
	QLabel* m_draftNeutralLabel = nullptr;
	QPushButton* m_btnPickDraftNeutral = nullptr;
	QLabel* m_draftStatus = nullptr;
	QPushButton* m_btnDraftOk = nullptr;
	QPushButton* m_btnDraftCancel = nullptr;
	PluginSketchPlane m_draftNeutralPlane{};
	bool m_hasDraftNeutral = false;
	QDoubleSpinBox* m_length = nullptr;
	QCheckBox* m_chkReversed = nullptr;
	QCheckBox* m_chkNewBody = nullptr;
	QComboBox* m_endCondition = nullptr;
	QPushButton* m_btnPickFace = nullptr;
	QPushButton* m_btnPickVertex = nullptr;
	QDoubleSpinBox* m_offsetFromFace = nullptr;
	QLabel* m_upToFaceStatus = nullptr;
	QPushButton* m_btnConfirm = nullptr;
	QPushButton* m_btnCancel = nullptr;
	QLabel* m_mirrorAxis = nullptr;
	QPushButton* m_btnPickAxis = nullptr;
	QPushButton* m_btnPickEnt = nullptr;
	QListWidget* m_mirrorList = nullptr;
	QCheckBox* m_keepOriginal = nullptr;
	QPushButton* m_btnMirrorOk = nullptr;
	QPushButton* m_btnMirrorCancel = nullptr;
	QPushButton* m_btnClearEnt = nullptr;
	QLabel* m_trimHint = nullptr;
	FeatureDocument m_features;
	std::unique_ptr<CommandStack> m_commands;
	QString m_activeBodyId;
	QStringList m_bodyIds;
	QString m_activeSketchId;
	QString m_selectedFeatureId;
	QString m_editingFeatureId;
	PluginSketchPlane m_upToFacePlane{};
	bool m_hasUpToFacePlane = false;
	QString m_upToFaceBackendId;
	int m_upToFaceIndex = -1;
	PluginPoint3d m_upToVertex{};
	bool m_hasUpToVertex = false;
	bool m_pocketMode = false;
	bool m_originPointVisible = true;
	bool m_originXyVisible = true;
	bool m_originXzVisible = true;
	bool m_originYzVisible = true;
};

#endif
