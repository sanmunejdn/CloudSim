#ifndef LABELINGPLUGIN_LABELINGANNOTWIDGET_H
#define LABELINGPLUGIN_LABELINGANNOTWIDGET_H

/// @file LabelingAnnotWidget.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief LabelingAnnotWidget 接口

#include "PluginLabelingTypes.h"

#include <QWidget>
#include <memory>
#include <vector>

class PointNetInference;
class IPluginHostContext;
class IPluginLabelingHost;
class QButtonGroup;
class QComboBox;
class QDoubleSpinBox;
class QGroupBox;
class QLabel;
class QPushButton;
class QTableWidget;
class QToolButton;

class LabelingAnnotWidget : public QWidget
{
	Q_OBJECT

public:
	explicit LabelingAnnotWidget(IPluginHostContext* host, QWidget* parent = nullptr);
	~LabelingAnnotWidget() override;

	void applyLanguage();
	void refreshBackendList();
	void setLastExportDir(const QString& path);
	QString lastExportDir() const { return m_lastExportDir; }

signals:
	void datasetExported(const QString& exportDir);

private slots:
	void onBackendChanged(int index);
	void onClassTableChanged(int row, int column);
	void onClassRowActivated(int row, int column);
	void onToolClicked(int toolId);
	void onUndoClicked();
	void onRedoClicked();
	void onExportClicked();
	void onPrelabelClicked();
	void onRefreshBackendsClicked();

private:
	QString i18n(const QString& en, const QString& zh) const;
	void rebuildDefaultClasses();
	PluginLabelingSessionConfig buildSessionConfig() const;
	bool ensureSession(QString* err = nullptr);
	void clearSession();
	void refreshSummary();
	void selectClassRowById(int classId);
	void activateTool(PluginLabelingTool tool);
	void onToolPickCancelled();
	void armActiveTool();
	void checkToolButton(PluginLabelingTool tool);
	bool loadPointNetSegmentModel(QString* err);
	bool extractBackendPoints(const std::string& backendId, std::vector<float>& outPoints, int& outCount) const;
	void applySelection(const PluginLabelingSelectionResult& selection, bool erase);

	IPluginHostContext* m_host = nullptr;
	IPluginLabelingHost* m_labelingHost = nullptr;
	std::unique_ptr<PointNetInference> m_inference;

	QGroupBox* m_targetGroup = nullptr;
	QComboBox* m_backendCombo = nullptr;
	QPushButton* m_refreshBackendsBtn = nullptr;
	QLabel* m_summaryLabel = nullptr;

	QGroupBox* m_classGroup = nullptr;
	QTableWidget* m_classTable = nullptr;

	QGroupBox* m_toolGroup = nullptr;
	QButtonGroup* m_toolButtons = nullptr;
	QToolButton* m_clickTool = nullptr;
	QToolButton* m_brushTool = nullptr;
	QToolButton* m_lassoTool = nullptr;
	QToolButton* m_eraseTool = nullptr;
	QPushButton* m_cancelPickBtn = nullptr;
	QDoubleSpinBox* m_brushRadiusSpin = nullptr;
	QPushButton* m_undoBtn = nullptr;
	QPushButton* m_redoBtn = nullptr;

	QPushButton* m_prelabelBtn = nullptr;
	QPushButton* m_exportBtn = nullptr;

	PluginLabelingSessionId m_sessionId = kInvalidLabelingSessionId;
	std::string m_backendId;
	PluginLabelingGeometryKind m_geometryKind = PluginLabelingGeometryKind::PointCloud;
	PluginLabelingTool m_activeTool = PluginLabelingTool::Click;
	bool m_eraseMode = false;
	bool m_toolPickActive = false;
	QString m_lastExportDir;
};

#endif // LABELINGPLUGIN_LABELINGANNOTWIDGET_H
