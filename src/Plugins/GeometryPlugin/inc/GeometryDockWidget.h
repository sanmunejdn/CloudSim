#pragma once

#include <QWidget>

#include "PluginGeometryTypes.h"

#include <vector>

class IPluginHostContext;
class QComboBox;
class QGroupBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QSpinBox;

class GeometryDockWidget : public QWidget
{
	Q_OBJECT

public:
	explicit GeometryDockWidget(IPluginHostContext* host, QWidget* parent = nullptr);

	void applyLanguage();
	void refreshDocumentLabel();

private:
	void wireSignals();
	void refreshComputableBackends();
	void syncSourceUiState();
	QString activeStepPath() const;
	std::string activeBackendId() const;
	bool hasBackendSource() const;
	PluginMeshDiscretizeParams buildDiscretizeParams() const;
	PluginMeshCreateOptions buildMeshCreateOptions(const QString& displayName) const;
	void setStatus(const QString& text, bool isError = false);
	bool ensureGeometryHostReady();

	void browseStepFile();
	void discretizeStep();
	void intersectEdgeFace();
	void intersectFaceFace();
	void pickEdgeForEdgeFace();
	void pickFaceForEdgeFace();
	void pickFaceAForFaceFace();
	void pickFaceBForFaceFace();
	void createTubeFromLastIntersection();
	void createRibbonFromLastIntersection();

	IPluginHostContext* m_host = nullptr;
	bool m_useChinese = true;
	std::vector<PluginGeometryBackendEntry> m_backendEntries;
	std::vector<std::vector<float>> m_lastIntersectionPolylines;
	QString m_lastIntersectionSourcePath;

	QGroupBox* m_docGroup = nullptr;
	QLabel* m_docLabel = nullptr;

	QGroupBox* m_stepGroup = nullptr;
	QComboBox* m_sourceCombo = nullptr;
	QComboBox* m_backendCombo = nullptr;
	QPushButton* m_refreshBackendsBtn = nullptr;
	QLineEdit* m_stepPathEdit = nullptr;
	QPushButton* m_browseBtn = nullptr;
	QComboBox* m_qualityCombo = nullptr;
	QPushButton* m_discretizeBtn = nullptr;

	QGroupBox* m_ixEdgeFaceGroup = nullptr;
	QSpinBox* m_edgeSpin = nullptr;
	QSpinBox* m_faceSpin = nullptr;
	QPushButton* m_pickEdgeBtn = nullptr;
	QPushButton* m_pickFaceBtn = nullptr;
	QPushButton* m_ixEdgeFaceBtn = nullptr;

	QGroupBox* m_ixFaceFaceGroup = nullptr;
	QSpinBox* m_faceASpin = nullptr;
	QSpinBox* m_faceBSpin = nullptr;
	QPushButton* m_pickFaceABtn = nullptr;
	QPushButton* m_pickFaceBBtn = nullptr;
	QPushButton* m_ixFaceFaceBtn = nullptr;

	QGroupBox* m_resultGroup = nullptr;
	QPushButton* m_createTubeBtn = nullptr;
	QPushButton* m_createRibbonBtn = nullptr;
	QLabel* m_statusLabel = nullptr;
};
