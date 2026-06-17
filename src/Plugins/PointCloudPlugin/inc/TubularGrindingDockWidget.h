#pragma once

#include "PluginPointCloudTypes.h"

#include <QWidget>

class IPluginHostContext;
class IPluginDocument;
class QComboBox;
class QDoubleSpinBox;
class QSpinBox;
class QPushButton;
class QLabel;
class QTextEdit;
class QGroupBox;
class QShowEvent;

class TubularGrindingDockWidget : public QWidget
{
	Q_OBJECT

public:
	explicit TubularGrindingDockWidget(IPluginHostContext* host, QWidget* parent = nullptr);

	void applyLanguage();
	void refreshMeshList();

protected:
	void showEvent(QShowEvent* event) override;

private slots:
	void onMeshSelectionChanged();
	void onResetSessionClicked();
	void onSegmentClicked();
	void onCenterlineClicked();
	void onTemplateClicked();
	void onProjectClicked();

private:
	QString i18n(const QString& en, const QString& zh) const;
	IPluginDocument* activeDoc() const;
	class IPluginPointCloudHost* pointCloudHost() const;
	std::string selectedMeshBackendId() const;
	PluginTubularGrindingParams buildParams() const;
	void appendLog(const QString& line);
	void resetSessionUi();
	void updateButtonStates();
	void ensureSession();
	void runStage(PluginTubularGrindingStage stage);
	void refreshSummary(const PluginTubularGrindingReport& report);

	IPluginHostContext* m_host = nullptr;
	bool m_busy = false;

	QGroupBox* m_rootGroup = nullptr;
	QLabel* m_meshLabel = nullptr;
	QComboBox* m_meshCombo = nullptr;
	QLabel* m_regionGrowAngleLabel = nullptr;
	QDoubleSpinBox* m_regionGrowAngleSpin = nullptr;
	QLabel* m_rayConvergenceLabel = nullptr;
	QDoubleSpinBox* m_rayConvergenceSpin = nullptr;
	QLabel* m_axisMergeAngleLabel = nullptr;
	QDoubleSpinBox* m_axisMergeAngleSpin = nullptr;
	QLabel* m_junctionSpreadLabel = nullptr;
	QDoubleSpinBox* m_junctionSpreadSpin = nullptr;
	QLabel* m_minSegmentFacesLabel = nullptr;
	QSpinBox* m_minSegmentFacesSpin = nullptr;
	QLabel* m_templateLabel = nullptr;
	QComboBox* m_templateCombo = nullptr;
	QLabel* m_sectionSpacingLabel = nullptr;
	QDoubleSpinBox* m_sectionSpacingSpin = nullptr;
	QLabel* m_helicalCoilsLabel = nullptr;
	QSpinBox* m_helicalCoilsSpin = nullptr;
	QLabel* m_projectionDistLabel = nullptr;
	QDoubleSpinBox* m_projectionDistSpin = nullptr;
	QPushButton* m_segmentBtn = nullptr;
	QPushButton* m_centerlineBtn = nullptr;
	QPushButton* m_templateBtn = nullptr;
	QPushButton* m_projectBtn = nullptr;
	QPushButton* m_resetBtn = nullptr;
	QTextEdit* m_log = nullptr;
	QLabel* m_summaryLabel = nullptr;

	PluginTubularGrindingSessionId m_sessionId;
	PluginTubularGrindingStage m_lastStage = PluginTubularGrindingStage::None;
	std::string m_meshBackendId;
};
