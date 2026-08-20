#ifndef ROBOTWIDGET_ROBOTCOMMPAGEWIDGET_H
#define ROBOTWIDGET_ROBOTCOMMPAGEWIDGET_H

/// @file RobotCommPageWidget.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 机器人通讯页：Bridge/真机连接、镜像开关、反馈显示

#include "robotwidget_global.h"

#include <QWidget>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QSpinBox;

class ROBOTWIDGET_EXPORT RobotCommPageWidget : public QWidget
{
	Q_OBJECT

public:
	explicit RobotCommPageWidget(QWidget* parent = nullptr);

	void setUseChinese(bool chinese);
	void setStatusText(const QString& text);
	void setFeedbackText(const QString& jointsLine, const QString& poseLine);
	void setConnectedUi(bool bridgeOk, bool robotOk);

	QString bridgeHost() const;
	int bridgePort() const;
	QString brand() const;
	QString robotHost() const;
	int robotPort() const;
	QString user() const;
	QString password() const;
	bool mirrorEnabled() const;
	int pollIntervalMs() const;

signals:
	void connectRequested();
	void disconnectRequested();
	void mirrorToggled(bool enabled);
	void pollIntervalChanged(int ms);

private:
	void retranslate();

	bool m_useChinese = true;
	QComboBox* m_brandCombo = nullptr;
	QLineEdit* m_bridgeHostEdit = nullptr;
	QSpinBox* m_bridgePortSpin = nullptr;
	QLineEdit* m_robotHostEdit = nullptr;
	QSpinBox* m_robotPortSpin = nullptr;
	QLineEdit* m_userEdit = nullptr;
	QLineEdit* m_passEdit = nullptr;
	QCheckBox* m_mirrorCheck = nullptr;
	QSpinBox* m_pollSpin = nullptr;
	QPushButton* m_connectBtn = nullptr;
	QPushButton* m_disconnectBtn = nullptr;
	QLabel* m_statusLabel = nullptr;
	QLabel* m_jointsLabel = nullptr;
	QLabel* m_poseLabel = nullptr;
	QLabel* m_titleBrand = nullptr;
	QLabel* m_titleBridge = nullptr;
	QLabel* m_titleRobot = nullptr;
	QLabel* m_titleUser = nullptr;
	QLabel* m_titlePass = nullptr;
	QLabel* m_titlePoll = nullptr;
};

#endif // ROBOTWIDGET_ROBOTCOMMPAGEWIDGET_H
