#ifndef INDUSTRIALCAMERAPLUGIN_HANDEYEPANELWIDGET_H
#define INDUSTRIALCAMERAPLUGIN_HANDEYEPANELWIDGET_H

/// @file HandEyePanelWidget.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 手眼标定阶段向导

#include "HandEyeTypes.h"
#include "IRobotPoseSource.h"
#include "MechOfficialHandEye.h"

#include <QWidget>
#include <memory>
#include <vector>

class QLabel;
class QRadioButton;
class QDoubleSpinBox;
class QTableWidget;
class QLineEdit;
class QSpinBox;
class QComboBox;
class CameraPanelWidget;
class IPluginHostContext;

class HandEyePanelWidget : public QWidget
{
	Q_OBJECT
public:
	explicit HandEyePanelWidget(CameraPanelWidget* cameraPanel, IPluginHostContext* host,
								QWidget* parent = nullptr);

	void setUseChinese(bool zh);
	void applyLanguage();

private slots:
	void onNextConfig();
	void onBackCollect();
	void onReadPose();
	void onAddSample();
	void onCaptureDetect();
	void onDeleteSample();
	void onSolve();
	void onExport();
	void onOpenDir();
	void onFillDemo();

private:
	enum class Stage
	{
		Config = 0,
		Collect,
		Result
	};
	void setStage(Stage s);
	void refreshStageLabel();
	void fillScoreTable(const industrial_camera::HandEyeResult& r);
	industrial_camera::Pose6d readPoseSpins() const;
	void writePoseSpins(const industrial_camera::Pose6d& p);
	void hostLog(const QString& s, bool isError = false);
	void appendSampleRow(const industrial_camera::Pose6d& robot, const industrial_camera::Pose6d& board);

	bool zh_ = true;
	Stage stage_ = Stage::Config;
	IPluginHostContext* host_ = nullptr;
	CameraPanelWidget* cameraPanel_ = nullptr;
	std::vector<industrial_camera::HandEyeSample> samples_;
	industrial_camera::HandEyeResult lastResult_;
	QString lastCalibDir_;
	std::unique_ptr<industrial_camera::IRobotPoseSource> poseSrc_;
	std::unique_ptr<industrial_camera::MechOfficialHandEyeSession> mechSession_;

	QLabel* stageLabel_ = nullptr;
	QWidget* pageConfig_ = nullptr;
	QWidget* pageCollect_ = nullptr;
	QWidget* pageResult_ = nullptr;
	QRadioButton* eyeInHand_ = nullptr;
	QRadioButton* eyeToHand_ = nullptr;
	QRadioButton* poseManual_ = nullptr;
	QRadioButton* poseReal_ = nullptr;
	QLineEdit* robotHost_ = nullptr;
	QSpinBox* robotPort_ = nullptr;
	QComboBox* boardTypeCombo_ = nullptr;
	QSpinBox* boardCornersX_ = nullptr;
	QSpinBox* boardCornersY_ = nullptr;
	QDoubleSpinBox* boardSquareMm_ = nullptr;
	QDoubleSpinBox* px_ = nullptr;
	QDoubleSpinBox* py_ = nullptr;
	QDoubleSpinBox* pz_ = nullptr;
	QDoubleSpinBox* prx_ = nullptr;
	QDoubleSpinBox* pry_ = nullptr;
	QDoubleSpinBox* prz_ = nullptr;
	QDoubleSpinBox* bx_ = nullptr;
	QDoubleSpinBox* by_ = nullptr;
	QDoubleSpinBox* bz_ = nullptr;
	QDoubleSpinBox* brx_ = nullptr;
	QDoubleSpinBox* bry_ = nullptr;
	QDoubleSpinBox* brz_ = nullptr;
	QTableWidget* sampleTable_ = nullptr;
	QTableWidget* scoreTable_ = nullptr;
	QLabel* status_ = nullptr;
	QLabel* pathLabel_ = nullptr;
};

#endif // INDUSTRIALCAMERAPLUGIN_HANDEYEPANELWIDGET_H
