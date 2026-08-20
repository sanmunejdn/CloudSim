#ifndef INDUSTRIALCAMERAPLUGIN_CAMERAPANELWIDGET_H
#define INDUSTRIALCAMERAPLUGIN_CAMERAPANELWIDGET_H

/// @file CameraPanelWidget.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 工业相机侧栏：连接/预览/落盘

#include <QWidget>
#include <memory>

namespace industrial_camera
{
class ICamera;
class ICameraFactory;
struct CameraFrame2D;
struct CameraFrame3D;
} // namespace industrial_camera

class QComboBox;
class QLineEdit;
class QLabel;
class QCheckBox;
class QSpinBox;
class IPluginHostContext;

class CameraPanelWidget : public QWidget
{
	Q_OBJECT
public:
	explicit CameraPanelWidget(IPluginHostContext* host, QWidget* parent = nullptr);
	~CameraPanelWidget() override;

	void setUseChinese(bool zh);
	void applyLanguage();

	industrial_camera::ICamera* camera() const { return camera_.get(); }

signals:
	void frameCaptured();

private slots:
	void onBrandChanged(int index);
	void onEnumerate();
	void onDevicePicked(int index);
	void onConnect();
	void onDisconnect();
	void onGrab();
	void onLiveToggled(bool on);
	void onSave();
	void onImportCloud();
	void onOpenDir();

private:
	void appendLog(const QString& s);
	void updatePreview(const industrial_camera::CameraFrame2D& f);
	void setConnectedUi(bool on);
	void refreshCapabilityChecks();

	IPluginHostContext* host_ = nullptr;
	bool zh_ = true;
	std::unique_ptr<industrial_camera::ICameraFactory> factory_;
	std::unique_ptr<industrial_camera::ICamera> camera_;
	std::unique_ptr<industrial_camera::CameraFrame2D> last2d_;
	std::unique_ptr<industrial_camera::CameraFrame3D> last3d_;
	QString lastCaptureDir_;

	QComboBox* brandCombo_ = nullptr;
	QComboBox* typeCombo_ = nullptr;
	QComboBox* deviceCombo_ = nullptr;
	QLineEdit* ipEdit_ = nullptr;
	QLabel* preview_ = nullptr;
	QLabel* status_ = nullptr;
	QCheckBox* saveColor_ = nullptr;
	QCheckBox* saveDepth_ = nullptr;
	QCheckBox* saveCloud_ = nullptr;
	QCheckBox* live_ = nullptr;
	QSpinBox* timeoutSpin_ = nullptr;
	QLabel* pathLabel_ = nullptr;
	class QTimer* liveTimer_ = nullptr;
};

#endif // INDUSTRIALCAMERAPLUGIN_CAMERAPANELWIDGET_H
