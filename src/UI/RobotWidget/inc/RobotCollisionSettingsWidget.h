#ifndef ROBOTWIDGET_ROBOTCOLLISIONSETTINGSWIDGET_H
#define ROBOTWIDGET_ROBOTCOLLISIONSETTINGSWIDGET_H

/// @file RobotCollisionSettingsWidget.h
/// @brief 仿真 Dock：启用碰撞检测 + 安全余量

#include "robotwidget_global.h"

#include "RobotCollisionSettings.h"

#include <QWidget>

class QCheckBox;
class QDoubleSpinBox;

class ROBOTWIDGET_EXPORT RobotCollisionSettingsWidget : public QWidget
{
	Q_OBJECT

public:
	explicit RobotCollisionSettingsWidget(QWidget* parent = nullptr);

	void setUseChinese(bool chinese);
	void setSettings(const RobotCollision::Settings& s);
	RobotCollision::Settings settings() const;

signals:
	void settingsChanged();

private slots:
	void onFieldChanged();

private:
	void refreshEnabledState();

	QCheckBox* m_enabledCheck = nullptr;
	QDoubleSpinBox* m_marginSpin = nullptr;
	bool m_chinese = true;
	bool m_block = false;
};

#endif // ROBOTWIDGET_ROBOTCOLLISIONSETTINGSWIDGET_H
