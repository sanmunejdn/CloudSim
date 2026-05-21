#pragma once

#include "RobotCoordinateFrames.h"
#include "robotwidget_global.h"

#include <QStringList>
#include <QWidget>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QFormLayout;
class QGroupBox;
class QListWidget;
class QPushButton;
class QTimer;

class ROBOTWIDGET_EXPORT RobotFrameSettingsWidget : public QWidget
{
	Q_OBJECT

public:
	explicit RobotFrameSettingsWidget(QWidget* parent = nullptr);

	void setUseChinese(bool chinese);
	void setLinkNameOptions(const QStringList& linkNames);
	void setCoordinateFrames(const RobotCoordinate::RobotCoordinateFrameSet& frames);
	RobotCoordinate::RobotCoordinateFrameSet coordinateFrames();

signals:
	void framesChanged();
	void captureToolFromTcpRequested();
	void captureUserFrameFromTcpRequested();
	void resetToolFrameRequested();

private:
	void rebuildToolFrameList();
	void rebuildUserFrameList();
	void loadToolFieldsFromSelection();
	void loadUserFieldsFromSelection();
	void saveToolFieldsToSelection();
	void saveUserFieldsToSelection();
	void scheduleFramesChanged();

	void onToolFieldChanged();
	void onUserFieldChanged();
	void onToolListSelectionChanged();
	void onUserListSelectionChanged();
	void onAddToolFrame();
	void onRemoveToolFrame();
	void onDuplicateToolFrame();
	void onSetActiveToolFrame();
	void onAddUserFrame();
	void onRemoveUserFrame();
	void onDuplicateUserFrame();
	void onSetActiveUserFrame();
	void onShowToggled();
	void onFramesChangeDebounce();

	bool m_useChinese = true;
	bool m_blockSignals = false;
	int m_lastToolListRow = -1;
	RobotCoordinate::RobotCoordinateFrameSet m_frames;
	QStringList m_linkNames;

	QGroupBox* m_toolGroup = nullptr;
	QGroupBox* m_userGroup = nullptr;
	QGroupBox* m_showGroup = nullptr;
	QFormLayout* m_toolForm = nullptr;
	QFormLayout* m_userForm = nullptr;
	QListWidget* m_toolList = nullptr;
	QComboBox* m_flangeLinkCombo = nullptr;
	QDoubleSpinBox* m_toolPos[3]{};
	QDoubleSpinBox* m_toolEuler[3]{};
	QListWidget* m_userList = nullptr;
	QDoubleSpinBox* m_userPos[3]{};
	QDoubleSpinBox* m_userEuler[3]{};
	QCheckBox* m_showToolCheck = nullptr;
	QCheckBox* m_showUserCheck = nullptr;
	QPushButton* m_captureToolBtn = nullptr;
	QPushButton* m_resetToolBtn = nullptr;
	QPushButton* m_captureUserBtn = nullptr;
	QPushButton* m_addToolBtn = nullptr;
	QPushButton* m_removeToolBtn = nullptr;
	QPushButton* m_duplicateToolBtn = nullptr;
	QPushButton* m_setActiveToolBtn = nullptr;
	QPushButton* m_addUserBtn = nullptr;
	QPushButton* m_removeUserBtn = nullptr;
	QPushButton* m_duplicateUserBtn = nullptr;
	QPushButton* m_setActiveUserBtn = nullptr;
	QTimer* m_framesDebounceTimer = nullptr;
};
