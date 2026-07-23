#ifndef ROBOTWIDGET_ROBOTEXTERNALAXISSETTINGSWIDGET_H
#define ROBOTWIDGET_ROBOTEXTERNALAXISSETTINGSWIDGET_H

/// @file RobotExternalAxisSettingsWidget.h
/// @brief 机器人对象外部轴配置页（一期地轨）

#include "robotwidget_global.h"

#include "RobotExternalAxes.h"

#include <QStringList>
#include <QWidget>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QFormLayout;
class QGroupBox;
class QLabel;
class QListWidget;
class QPushButton;
class QTimer;

class ROBOTWIDGET_EXPORT RobotExternalAxisSettingsWidget : public QWidget
{
	Q_OBJECT

public:
	explicit RobotExternalAxisSettingsWidget(QWidget* parent = nullptr);

	void setUseChinese(bool chinese);
	void setJointNameOptions(const QStringList& jointNames);
	void setExternalAxes(const RobotExternal::RobotExternalAxisConfigSet& axes);
	RobotExternal::RobotExternalAxisConfigSet externalAxes();

signals:
	void externalAxesChanged();

private:
	void rebuildList();
	void loadFieldsFromSelection();
	void saveFieldsToSelection();
	void scheduleChanged();
	void onFieldChanged();
	void onListSelectionChanged();
	void onAddAxis();
	void onRemoveAxis();
	void onChangedDebounce();
	void refreshEmptyHint();

	bool m_useChinese = true;
	bool m_blockSignals = false;
	RobotExternal::RobotExternalAxisConfigSet m_axes;
	QStringList m_jointNames;

	QLabel* m_emptyHint = nullptr;
	QGroupBox* m_editorGroup = nullptr;
	QFormLayout* m_form = nullptr;
	QListWidget* m_list = nullptr;
	QCheckBox* m_enabledCheck = nullptr;
	QComboBox* m_jointCombo = nullptr;
	QDoubleSpinBox* m_lowerSpin = nullptr;
	QDoubleSpinBox* m_upperSpin = nullptr;
	QDoubleSpinBox* m_homeSpin = nullptr;
	QDoubleSpinBox* m_axisSpin[3]{};
	QPushButton* m_addBtn = nullptr;
	QPushButton* m_removeBtn = nullptr;
	QTimer* m_debounceTimer = nullptr;
};

#endif // ROBOTWIDGET_ROBOTEXTERNALAXISSETTINGSWIDGET_H
