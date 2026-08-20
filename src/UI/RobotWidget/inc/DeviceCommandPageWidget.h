#ifndef ROBOTWIDGET_DEVICECOMMANDPAGEWIDGET_H
#define ROBOTWIDGET_DEVICECOMMANDPAGEWIDGET_H

/// @file DeviceCommandPageWidget.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 设备指令：姿态库 + 本机 DI 绑定

#include "robotwidget_global.h"

#include <QWidget>

class DevicePoseMotionPlayer;
class IRobotMainWindowHost;
class IoSignalNetworkService;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QListWidget;
class QPushButton;
class QTableWidget;

namespace RobotIo
{
class NamedSignalTable;
}

class ROBOTWIDGET_EXPORT DeviceCommandPageWidget : public QWidget
{
	Q_OBJECT

public:
	explicit DeviceCommandPageWidget(QWidget* parent = nullptr);

	void setUseChinese(bool chinese);
	void setHost(IRobotMainWindowHost* host);
	void setNetwork(IoSignalNetworkService* network);
	void setMotionPlayer(DevicePoseMotionPlayer* player);

	void refreshDevices();
	void refreshDiSignalOptions();
	void reloadCurrentDeviceUi();

private slots:
	void onDeviceChanged(int index);
	void onTeachPose();
	void onAddPose();
	void onRenamePose();
	void onDeletePose();
	void onGoToPose();
	void onAddBinding();
	void onDeleteBinding();
	void onBindingCellChanged(int row, int column);
	void onStopMotion();
	void onPlayerStatusChanged();

private:
	QString i18n(const QString& en, const QString& zh) const;
	void retranslateUi();
	QString currentDeviceId() const;
	RobotIo::NamedSignalTable* currentDeviceSignalTable() const;
	void persistBindingsFromTable();
	void fillPoseList();
	void fillBindingTable();
	void updateStatusLabel();

	bool m_useChinese = true;
	bool m_blockBindingEdit = false;
	IRobotMainWindowHost* m_host = nullptr;
	IoSignalNetworkService* m_network = nullptr;
	DevicePoseMotionPlayer* m_player = nullptr;

	QLabel* m_deviceLabel = nullptr;
	QComboBox* m_deviceCombo = nullptr;
	QLabel* m_poseLabel = nullptr;
	QListWidget* m_poseList = nullptr;
	QPushButton* m_teachBtn = nullptr;
	QPushButton* m_addPoseBtn = nullptr;
	QPushButton* m_renamePoseBtn = nullptr;
	QPushButton* m_deletePoseBtn = nullptr;
	QPushButton* m_goPoseBtn = nullptr;
	QLabel* m_bindLabel = nullptr;
	QTableWidget* m_bindTable = nullptr;
	QPushButton* m_addBindBtn = nullptr;
	QPushButton* m_deleteBindBtn = nullptr;
	QLabel* m_durationLabel = nullptr;
	QDoubleSpinBox* m_goDurationSpin = nullptr;
	QPushButton* m_stopBtn = nullptr;
	QLabel* m_statusLabel = nullptr;
};

#endif // ROBOTWIDGET_DEVICECOMMANDPAGEWIDGET_H
