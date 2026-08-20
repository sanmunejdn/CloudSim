#ifndef ROBOTWIDGET_ROBOTCOLLISIONSETTINGSWIDGET_H
#define ROBOTWIDGET_ROBOTCOLLISIONSETTINGSWIDGET_H

/// @file RobotCollisionSettingsWidget.h
/// @brief 碰撞黑白名单 + 页内起终点路径规划

#include "robotwidget_global.h"

#include "MotionPathPlanDialog.h"
#include "RobotCollisionSettings.h"

#include <QWidget>
#include <QVector>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QGroupBox;
class QLabel;
class QListWidget;
class QPushButton;
class QTableWidget;

struct ROBOTWIDGET_EXPORT CollisionSceneObjectItem
{
	QString backendId;
	QString label;
};

class ROBOTWIDGET_EXPORT RobotCollisionSettingsWidget : public QWidget
{
	Q_OBJECT

public:
	explicit RobotCollisionSettingsWidget(QWidget* parent = nullptr);

	void setUseChinese(bool chinese);
	void setSettings(const RobotCollision::Settings& s);
	RobotCollision::Settings settings() const;
	void setPlanStatusText(const QString& text);
	void setConfirmEnabled(bool enabled);

	void setMotionWaypoints(const QVector<MotionPathWaypointItem>& items);
	void selectMotionWaypointIds(const QString& startId, const QString& endId);
	QString selectedStartWaypointId() const;
	QString selectedEndWaypointId() const;

	void setCollisionSceneObjects(const QVector<CollisionSceneObjectItem>& items);

signals:
	void settingsChanged();
	void planRequested();
	void clearPreviewRequested();
	void confirmTrajectoryRequested();
	void refreshSceneObjectsRequested();

private slots:
	void onFieldChanged();
	void onAddToWhite();
	void onAddToBlack();
	void onRemoveFromWhite();
	void onRemoveFromBlack();

private:
	void retranslateUi();
	void refreshEnabledState();
	void rebuildListTables();
	QStringList selectedPoolIds() const;
	void moveSelectedPoolToList(bool toWhite);
	void removeSelectedFromTable(QTableWidget* table);

	QGroupBox* m_collisionGroup = nullptr;
	QCheckBox* m_enabledCheck = nullptr;
	QDoubleSpinBox* m_marginSpin = nullptr;
	QLabel* m_marginLabel = nullptr;
	QLabel* m_listHintLabel = nullptr;
	QLabel* m_poolLabel = nullptr;
	QListWidget* m_poolList = nullptr;
	QLabel* m_whiteLabel = nullptr;
	QTableWidget* m_whiteTable = nullptr;
	QLabel* m_blackLabel = nullptr;
	QTableWidget* m_blackTable = nullptr;
	QPushButton* m_addWhiteBtn = nullptr;
	QPushButton* m_addBlackBtn = nullptr;
	QPushButton* m_removeWhiteBtn = nullptr;
	QPushButton* m_removeBlackBtn = nullptr;
	QPushButton* m_refreshObjectsBtn = nullptr;

	QGroupBox* m_planGroup = nullptr;
	QLabel* m_planHintLabel = nullptr;
	QLabel* m_startLabel = nullptr;
	QLabel* m_endLabel = nullptr;
	QComboBox* m_startCombo = nullptr;
	QComboBox* m_endCombo = nullptr;
	QPushButton* m_planBtn = nullptr;
	QPushButton* m_clearPreviewBtn = nullptr;
	QPushButton* m_confirmBtn = nullptr;
	QLabel* m_planStatusLabel = nullptr;

	QVector<CollisionSceneObjectItem> m_allObjects;
	QStringList m_whiteIds;
	QStringList m_blackIds;

	bool m_chinese = true;
	bool m_block = false;
};

#endif // ROBOTWIDGET_ROBOTCOLLISIONSETTINGSWIDGET_H
