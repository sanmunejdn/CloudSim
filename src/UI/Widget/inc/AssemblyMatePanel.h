#ifndef WIDGET_ASSEMBLYMATEPANEL_H
#define WIDGET_ASSEMBLYMATEPANEL_H

/// @file AssemblyMatePanel.h
/// @brief Insert 装配一次定位面板

#include "AssemblyMateApply.h"
#include "widget_global.h"

#include <QWidget>
#include <vector>

#include <osg/Vec3f>

class MainWindow;
class QButtonGroup;
class QDoubleSpinBox;
class QLabel;
class QPushButton;
class QRadioButton;
struct PickResult;
enum class PickKind;

class AssemblyMatePanel : public QWidget
{
	Q_OBJECT

public:
	explicit AssemblyMatePanel(MainWindow* mw, QWidget* parent = nullptr);
	void applyLanguage();
	void beginSession();
	void endSession();
	void interruptPicking();

private:
	void startFacePick(int slot);
	void stopFacePick();
	void resetForNextMate();
	void onPickCommitted(const PickResult& pick, PickKind kind);
	void previewFromSnapshot();
	void restoreMovingIfPreviewed();
	geoalgo::AssemblyMateParams currentParams() const;
	void updateValueEditors();
	void setStatus(const QString& text, bool error);

	MainWindow* m_mw = nullptr;
	QButtonGroup* m_kindGroup = nullptr;
	QDoubleSpinBox* m_distanceSpin = nullptr;
	QDoubleSpinBox* m_angleSpin = nullptr;
	QRadioButton* m_antiAlignRadio = nullptr;
	QRadioButton* m_alignRadio = nullptr;
	QPushButton* m_pickFace1Btn = nullptr;
	QPushButton* m_pickFace2Btn = nullptr;
	QPushButton* m_okBtn = nullptr;
	QPushButton* m_cancelBtn = nullptr;
	QLabel* m_face1Label = nullptr;
	QLabel* m_face2Label = nullptr;
	QLabel* m_statusLabel = nullptr;
	QLabel* m_kindTitle = nullptr;
	QLabel* m_alignTitle = nullptr;
	QLabel* m_distanceLabel = nullptr;
	QLabel* m_angleLabel = nullptr;

	int m_pickSlot = -1;
	cloudsim::host::AssemblyMateFaceRef m_face1;
	cloudsim::host::AssemblyMateFaceRef m_face2;
	std::vector<osg::Vec3f> m_face1Verts;
	BackendMat4 m_movingSnapshot{};
	bool m_haveSnapshot = false;
	bool m_previewed = false;
};

#endif // WIDGET_ASSEMBLYMATEPANEL_H
