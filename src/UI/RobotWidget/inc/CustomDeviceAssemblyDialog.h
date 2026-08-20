#ifndef ROBOTWIDGET_CUSTOMDEVICEASSEMBLYDIALOG_H
#define ROBOTWIDGET_CUSTOMDEVICEASSEMBLYDIALOG_H

/// @file CustomDeviceAssemblyDialog.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 自定义设备组装对话框（画布 + 运动副属性）

#include "robotwidget_global.h"

#include <QDialog>
#include <QString>

#include <memory>

class CustomDeviceAssemblyCanvasWidget;
class CustomDeviceBackendData;
class ICustomDeviceAssemblyHost;
class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QFormLayout;
class QFrame;
class QLineEdit;
class QPushButton;

class ROBOTWIDGET_EXPORT CustomDeviceAssemblyDialog : public QDialog
{
	Q_OBJECT

public:
	/// existingDeviceBackendId 空则为新建
	explicit CustomDeviceAssemblyDialog(ICustomDeviceAssemblyHost* host, const QString& existingDeviceBackendId,
										QWidget* parent = nullptr);

	QString committedDeviceId() const { return m_committedDeviceId; }

private slots:
	void onFromScene();
	void onPickSolidToggled(bool on);
	void onImportModels();
	void onExportUrdf();
	void onApplyAccepted();
	void refreshJointProps();
	void pushJointProps();
	void refillCenterOptions();

private:
	QString i18n(const QString& en, const QString& zh) const;
	bool ensureDevice(QString* outErr);
	bool attachChildId(const QString& childId, QString* outErr);
	void preloadEditGraph();
	void setRotationCenterVisible(bool visible);

	ICustomDeviceAssemblyHost* m_host = nullptr;
	bool m_editMode = false;
	std::shared_ptr<CustomDeviceBackendData> m_device;
	QStringList m_childRootIds;
	bool m_blockProps = false;
	QString m_committedDeviceId;

	QLineEdit* m_nameEdit = nullptr;
	CustomDeviceAssemblyCanvasWidget* m_canvas = nullptr;
	QFrame* m_props = nullptr;
	QFormLayout* m_jointForm = nullptr;
	QComboBox* m_motionCombo = nullptr;
	QDoubleSpinBox* m_lowerSpin = nullptr;
	QDoubleSpinBox* m_upperSpin = nullptr;
	QDoubleSpinBox* m_homeSpin = nullptr;
	QDoubleSpinBox* m_axisX = nullptr;
	QDoubleSpinBox* m_axisY = nullptr;
	QDoubleSpinBox* m_axisZ = nullptr;
	QComboBox* m_centerFrameCombo = nullptr;
	QPushButton* m_connectBtn = nullptr;
	QPushButton* m_pickSolidBtn = nullptr;
};

#endif // ROBOTWIDGET_CUSTOMDEVICEASSEMBLYDIALOG_H
