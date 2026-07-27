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
	Sweep
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
	QString upToFaceBackendId() const { return m_upToFaceBackendId; }
	int upToFaceIndex() const { return m_upToFaceIndex; }
	void setUpToFacePlane(const PluginSketchPlane& plane, const QString& backendId = QString(), int faceIndex = -1);
	void clearUpToFacePlane();

	void setExtrudeUi(double lengthMm, bool reversed, GeomodelingExtrudeEnd end, bool createNewBody,
					  double draftAngleDeg = 0.0);
	/// Pocket：强制目标 Body、禁用新建实体，并切换侧栏文案
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
	void setSweepStatus(const QString& text);
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
	void pickUpToFaceRequested();
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

protected:
	bool eventFilter(QObject* watched, QEvent* event) override;

private:
	void buildLegendPanel();
	void rebuildLegendContent();
	void repositionLegend();
	QWidget* findViewportHost() const;
	QString i18n(const QString& en, const QString& zh) const;
	void syncExtrudeEndUi();

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
	QLabel* m_sweepTitle = nullptr;
	QLabel* m_sweepProfileCaption = nullptr;
	QLabel* m_sweepPathCaption = nullptr;
	QComboBox* m_sweepProfileCombo = nullptr;
	QComboBox* m_sweepPathCombo = nullptr;
	QPushButton* m_btnSweepOk = nullptr;
	QPushButton* m_btnSweepCancel = nullptr;
	QLabel* m_sweepStatus = nullptr;
	bool m_sweepCutMode = false;
	QDoubleSpinBox* m_length = nullptr;
	QDoubleSpinBox* m_draftAngle = nullptr;
	QCheckBox* m_chkReversed = nullptr;
	QCheckBox* m_chkNewBody = nullptr;
	QComboBox* m_endCondition = nullptr;
	QPushButton* m_btnPickFace = nullptr;
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
	bool m_pocketMode = false;
};

#endif
