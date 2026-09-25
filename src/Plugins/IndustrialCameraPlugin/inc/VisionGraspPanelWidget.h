#ifndef INDUSTRIALCAMERAPLUGIN_VISIONGRASPPANELWIDGET_H
#define INDUSTRIALCAMERAPLUGIN_VISIONGRASPPANELWIDGET_H

/// @file VisionGraspPanelWidget.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 视觉抓取 Tab：合成接近/抓取/抬起，复用碰撞页轨迹规划

#include "PluginRobotTypes.h"

#include <QJsonObject>
#include <QWidget>

class QCheckBox;
class QDoubleSpinBox;
class QGroupBox;
class QLabel;
class QPushButton;
class QRadioButton;
class HandEyePanelWidget;
class IPluginHostContext;

class VisionGraspPanelWidget : public QWidget
{
	Q_OBJECT
public:
	explicit VisionGraspPanelWidget(HandEyePanelWidget* handEye, IPluginHostContext* host, QWidget* parent = nullptr);

	void setUseChinese(bool zh);
	void applyLanguage();

	void loadOffsetsFromJson(const QJsonObject& obj);
	void saveOffsetsToJson(QJsonObject& obj) const;

private slots:
	void onReadCurrentTcp();
	void onReadSelectedObject();
	void onPreviewThreePoints();
	void onPlanAndConfirm();
	void refreshHandEyeStatus();

private:
	enum class PoseSource
	{
		ManualBase = 0,
		ManualCamera,
		SceneObject
	};

	PoseSource currentSource() const;
	bool composeGraspPoses(PluginPose6d& approach, PluginPose6d& grasp, PluginPose6d& retract, QString* err);
	PluginPose6d readPoseSpins() const;
	void writePoseSpins(const PluginPose6d& p);
	void hostLog(const QString& s, bool isError = false);

	bool zh_ = true;
	IPluginHostContext* host_ = nullptr;
	HandEyePanelWidget* handEye_ = nullptr;

	QLabel* handEyeStatus_ = nullptr;
	QLabel* planHint_ = nullptr;
	QGroupBox* srcBox_ = nullptr;
	QGroupBox* poseBox_ = nullptr;
	QGroupBox* graspBox_ = nullptr;
	QRadioButton* srcManualBase_ = nullptr;
	QRadioButton* srcManualCam_ = nullptr;
	QRadioButton* srcScene_ = nullptr;
	QDoubleSpinBox* px_ = nullptr;
	QDoubleSpinBox* py_ = nullptr;
	QDoubleSpinBox* pz_ = nullptr;
	QDoubleSpinBox* prx_ = nullptr;
	QDoubleSpinBox* pry_ = nullptr;
	QDoubleSpinBox* prz_ = nullptr;
	QDoubleSpinBox* approachZ_ = nullptr;
	QDoubleSpinBox* retractZ_ = nullptr;
	QCheckBox* lockEuler_ = nullptr;
	QPushButton* readTcpBtn_ = nullptr;
	QPushButton* readObjBtn_ = nullptr;
	QPushButton* previewBtn_ = nullptr;
	QPushButton* planBtn_ = nullptr;
	QLabel* status_ = nullptr;
};

#endif // INDUSTRIALCAMERAPLUGIN_VISIONGRASPPANELWIDGET_H
