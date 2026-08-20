/// @file RobotCollisionSettingsWidget.cpp
/// @brief 碰撞黑白名单 + 页内起终点路径规划

#include "RobotCollisionSettingsWidget.h"

#include "CollisionWorld.h"

#include <QAbstractItemView>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QListWidget>
#include <QPushButton>
#include <QSet>
#include <QTableWidget>
#include <QVariant>
#include <QVBoxLayout>

namespace
{
void setupIdTable(QTableWidget* table)
{
	table->setColumnCount(1);
	table->horizontalHeader()->setStretchLastSection(true);
	table->horizontalHeader()->setVisible(false);
	table->verticalHeader()->setVisible(false);
	table->setSelectionBehavior(QAbstractItemView::SelectRows);
	table->setSelectionMode(QAbstractItemView::ExtendedSelection);
	table->setEditTriggers(QAbstractItemView::NoEditTriggers);
	table->setMinimumHeight(100);
}

void fillIdTable(QTableWidget* table, const QStringList& ids, const QHash<QString, QString>& labelById)
{
	table->setRowCount(0);
	for (const QString& id : ids)
	{
		const int row = table->rowCount();
		table->insertRow(row);
		const QString label = labelById.value(id, id);
		auto* item = new QTableWidgetItem(label);
		item->setData(Qt::UserRole, id);
		table->setItem(row, 0, item);
	}
}

QStringList tableIds(const QTableWidget* table)
{
	QStringList out;
	if (!table)
		return out;
	for (int r = 0; r < table->rowCount(); ++r)
	{
		if (const QTableWidgetItem* it = table->item(r, 0))
			out.push_back(it->data(Qt::UserRole).toString());
	}
	return out;
}
} // namespace

RobotCollisionSettingsWidget::RobotCollisionSettingsWidget(QWidget* parent) : QWidget(parent)
{
	auto* layout = new QVBoxLayout(this);
	layout->setContentsMargins(8, 8, 8, 8);
	layout->setSpacing(8);

	m_collisionGroup = new QGroupBox(this);
	auto* colLayout = new QVBoxLayout(m_collisionGroup);
	auto* form = new QFormLayout();
	m_enabledCheck = new QCheckBox(m_collisionGroup);
	m_marginSpin = new QDoubleSpinBox(m_collisionGroup);
	m_marginSpin->setRange(0.0, 500.0);
	m_marginSpin->setDecimals(2);
	m_marginSpin->setSingleStep(0.5);
	m_marginSpin->setValue(1.0);
	m_marginSpin->setSuffix(QStringLiteral(" mm"));
	m_marginLabel = new QLabel(m_collisionGroup);
	form->addRow(m_enabledCheck);
	form->addRow(m_marginLabel, m_marginSpin);
	colLayout->addLayout(form);

	m_listHintLabel = new QLabel(m_collisionGroup);
	m_listHintLabel->setWordWrap(true);
	colLayout->addWidget(m_listHintLabel);

	m_poolLabel = new QLabel(m_collisionGroup);
	m_poolList = new QListWidget(m_collisionGroup);
	m_poolList->setSelectionMode(QAbstractItemView::ExtendedSelection);
	m_poolList->setMinimumHeight(80);
	colLayout->addWidget(m_poolLabel);
	colLayout->addWidget(m_poolList);

	auto* moveRow = new QHBoxLayout();
	m_addWhiteBtn = new QPushButton(m_collisionGroup);
	m_addBlackBtn = new QPushButton(m_collisionGroup);
	m_refreshObjectsBtn = new QPushButton(m_collisionGroup);
	moveRow->addWidget(m_addWhiteBtn);
	moveRow->addWidget(m_addBlackBtn);
	moveRow->addStretch(1);
	moveRow->addWidget(m_refreshObjectsBtn);
	colLayout->addLayout(moveRow);

	auto* tablesRow = new QHBoxLayout();
	auto* whiteCol = new QVBoxLayout();
	m_whiteLabel = new QLabel(m_collisionGroup);
	m_whiteTable = new QTableWidget(m_collisionGroup);
	setupIdTable(m_whiteTable);
	m_removeWhiteBtn = new QPushButton(m_collisionGroup);
	whiteCol->addWidget(m_whiteLabel);
	whiteCol->addWidget(m_whiteTable);
	whiteCol->addWidget(m_removeWhiteBtn);
	auto* blackCol = new QVBoxLayout();
	m_blackLabel = new QLabel(m_collisionGroup);
	m_blackTable = new QTableWidget(m_collisionGroup);
	setupIdTable(m_blackTable);
	m_removeBlackBtn = new QPushButton(m_collisionGroup);
	blackCol->addWidget(m_blackLabel);
	blackCol->addWidget(m_blackTable);
	blackCol->addWidget(m_removeBlackBtn);
	tablesRow->addLayout(whiteCol, 1);
	tablesRow->addLayout(blackCol, 1);
	colLayout->addLayout(tablesRow);
	layout->addWidget(m_collisionGroup);

	m_planGroup = new QGroupBox(this);
	auto* planLayout = new QVBoxLayout(m_planGroup);
	m_planHintLabel = new QLabel(m_planGroup);
	m_planHintLabel->setWordWrap(true);
	planLayout->addWidget(m_planHintLabel);

	auto* wpForm = new QFormLayout();
	m_startCombo = new QComboBox(m_planGroup);
	m_endCombo = new QComboBox(m_planGroup);
	m_startLabel = new QLabel(m_planGroup);
	m_endLabel = new QLabel(m_planGroup);
	wpForm->addRow(m_startLabel, m_startCombo);
	wpForm->addRow(m_endLabel, m_endCombo);
	planLayout->addLayout(wpForm);

	auto* btnRow = new QHBoxLayout();
	m_planBtn = new QPushButton(m_planGroup);
	m_planBtn->setProperty("btnRole", QVariant(QStringLiteral("primary")));
	m_clearPreviewBtn = new QPushButton(m_planGroup);
	btnRow->addWidget(m_planBtn);
	btnRow->addWidget(m_clearPreviewBtn);
	planLayout->addLayout(btnRow);

	m_confirmBtn = new QPushButton(m_planGroup);
	m_confirmBtn->setEnabled(false);
	m_confirmBtn->setProperty("btnRole", QVariant(QStringLiteral("primary")));
	planLayout->addWidget(m_confirmBtn);

	m_planStatusLabel = new QLabel(m_planGroup);
	m_planStatusLabel->setWordWrap(true);
	planLayout->addWidget(m_planStatusLabel);
	layout->addWidget(m_planGroup);
	layout->addStretch(1);

	connect(m_enabledCheck, &QCheckBox::toggled, this, &RobotCollisionSettingsWidget::onFieldChanged);
	connect(m_marginSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
			&RobotCollisionSettingsWidget::onFieldChanged);
	connect(m_planBtn, &QPushButton::clicked, this, &RobotCollisionSettingsWidget::planRequested);
	connect(m_clearPreviewBtn, &QPushButton::clicked, this, &RobotCollisionSettingsWidget::clearPreviewRequested);
	connect(m_confirmBtn, &QPushButton::clicked, this, &RobotCollisionSettingsWidget::confirmTrajectoryRequested);
	connect(m_addWhiteBtn, &QPushButton::clicked, this, &RobotCollisionSettingsWidget::onAddToWhite);
	connect(m_addBlackBtn, &QPushButton::clicked, this, &RobotCollisionSettingsWidget::onAddToBlack);
	connect(m_removeWhiteBtn, &QPushButton::clicked, this, &RobotCollisionSettingsWidget::onRemoveFromWhite);
	connect(m_removeBlackBtn, &QPushButton::clicked, this, &RobotCollisionSettingsWidget::onRemoveFromBlack);
	connect(m_refreshObjectsBtn, &QPushButton::clicked, this, &RobotCollisionSettingsWidget::refreshSceneObjectsRequested);

	if (!collision::CollisionWorld::hasCoalBackend())
	{
		m_enabledCheck->setToolTip(QStringLiteral("Built-in mesh collision (AABB + triangles)"));
	}
	retranslateUi();
	refreshEnabledState();
}

void RobotCollisionSettingsWidget::setUseChinese(const bool chinese)
{
	if (m_chinese == chinese)
		return;
	m_chinese = chinese;
	retranslateUi();
	refreshEnabledState();
}

void RobotCollisionSettingsWidget::setSettings(const RobotCollision::Settings& s)
{
	m_block = true;
	m_enabledCheck->setChecked(s.enabled);
	m_marginSpin->setValue(s.securityMarginMm);
	m_whiteIds.clear();
	m_blackIds.clear();
	for (const std::string& id : s.whiteListBackendIds)
		m_whiteIds.push_back(QString::fromStdString(id));
	for (const std::string& id : s.blackListBackendIds)
		m_blackIds.push_back(QString::fromStdString(id));
	rebuildListTables();
	m_block = false;
	refreshEnabledState();
}

RobotCollision::Settings RobotCollisionSettingsWidget::settings() const
{
	RobotCollision::Settings s;
	s.enabled = m_enabledCheck->isChecked();
	s.securityMarginMm = m_marginSpin->value();
	for (const QString& id : tableIds(m_whiteTable))
		s.whiteListBackendIds.push_back(id.toStdString());
	for (const QString& id : tableIds(m_blackTable))
		s.blackListBackendIds.push_back(id.toStdString());
	return s;
}

void RobotCollisionSettingsWidget::setPlanStatusText(const QString& text)
{
	if (m_planStatusLabel)
		m_planStatusLabel->setText(text);
}

void RobotCollisionSettingsWidget::setConfirmEnabled(const bool enabled)
{
	if (m_confirmBtn)
		m_confirmBtn->setEnabled(enabled);
}

void RobotCollisionSettingsWidget::setMotionWaypoints(const QVector<MotionPathWaypointItem>& items)
{
	const QString keepStart = selectedStartWaypointId();
	const QString keepEnd = selectedEndWaypointId();
	m_startCombo->clear();
	m_endCombo->clear();
	for (const MotionPathWaypointItem& it : items)
	{
		m_startCombo->addItem(it.label, it.id);
		m_endCombo->addItem(it.label, it.id);
	}
	selectMotionWaypointIds(keepStart, keepEnd);
	if (m_startCombo->count() >= 2 && m_startCombo->currentIndex() == m_endCombo->currentIndex())
		m_endCombo->setCurrentIndex(1);
}

void RobotCollisionSettingsWidget::selectMotionWaypointIds(const QString& startId, const QString& endId)
{
	auto selectId = [](QComboBox* box, const QString& id) {
		if (!box || id.isEmpty())
			return;
		const int idx = box->findData(id);
		if (idx >= 0)
			box->setCurrentIndex(idx);
	};
	selectId(m_startCombo, startId);
	selectId(m_endCombo, endId);
}

QString RobotCollisionSettingsWidget::selectedStartWaypointId() const
{
	return m_startCombo ? m_startCombo->currentData().toString() : QString();
}

QString RobotCollisionSettingsWidget::selectedEndWaypointId() const
{
	return m_endCombo ? m_endCombo->currentData().toString() : QString();
}

void RobotCollisionSettingsWidget::setCollisionSceneObjects(const QVector<CollisionSceneObjectItem>& items)
{
	m_allObjects = items;
	QSet<QString> valid;
	for (const CollisionSceneObjectItem& it : m_allObjects)
		valid.insert(it.backendId);
	auto filterList = [&](QStringList& ids) {
		QStringList kept;
		for (const QString& id : ids)
		{
			if (valid.contains(id))
				kept.push_back(id);
		}
		ids = kept;
	};
	filterList(m_whiteIds);
	filterList(m_blackIds);
	rebuildListTables();
}

void RobotCollisionSettingsWidget::rebuildListTables()
{
	QHash<QString, QString> labelById;
	QSet<QString> assigned;
	for (const CollisionSceneObjectItem& it : m_allObjects)
		labelById.insert(it.backendId, it.label);
	for (const QString& id : m_whiteIds)
		assigned.insert(id);
	for (const QString& id : m_blackIds)
		assigned.insert(id);

	m_poolList->clear();
	for (const CollisionSceneObjectItem& it : m_allObjects)
	{
		if (assigned.contains(it.backendId))
			continue;
		auto* item = new QListWidgetItem(it.label);
		item->setData(Qt::UserRole, it.backendId);
		m_poolList->addItem(item);
	}
	fillIdTable(m_whiteTable, m_whiteIds, labelById);
	fillIdTable(m_blackTable, m_blackIds, labelById);
}

QStringList RobotCollisionSettingsWidget::selectedPoolIds() const
{
	QStringList out;
	const QList<QListWidgetItem*> sel = m_poolList->selectedItems();
	for (QListWidgetItem* it : sel)
		out.push_back(it->data(Qt::UserRole).toString());
	return out;
}

void RobotCollisionSettingsWidget::moveSelectedPoolToList(const bool toWhite)
{
	const QStringList ids = selectedPoolIds();
	if (ids.isEmpty())
		return;
	for (const QString& id : ids)
	{
		m_whiteIds.removeAll(id);
		m_blackIds.removeAll(id);
		if (toWhite)
			m_whiteIds.push_back(id);
		else
			m_blackIds.push_back(id);
	}
	rebuildListTables();
	onFieldChanged();
}

void RobotCollisionSettingsWidget::removeSelectedFromTable(QTableWidget* table)
{
	if (!table)
		return;
	QStringList removeIds;
	const QList<QTableWidgetItem*> sel = table->selectedItems();
	for (QTableWidgetItem* it : sel)
	{
		if (it->column() == 0)
			removeIds.push_back(it->data(Qt::UserRole).toString());
	}
	if (removeIds.isEmpty())
		return;
	for (const QString& id : removeIds)
	{
		m_whiteIds.removeAll(id);
		m_blackIds.removeAll(id);
	}
	rebuildListTables();
	onFieldChanged();
}

void RobotCollisionSettingsWidget::onAddToWhite()
{
	moveSelectedPoolToList(true);
}

void RobotCollisionSettingsWidget::onAddToBlack()
{
	moveSelectedPoolToList(false);
}

void RobotCollisionSettingsWidget::onRemoveFromWhite()
{
	removeSelectedFromTable(m_whiteTable);
}

void RobotCollisionSettingsWidget::onRemoveFromBlack()
{
	removeSelectedFromTable(m_blackTable);
}

void RobotCollisionSettingsWidget::retranslateUi()
{
	if (m_collisionGroup)
		m_collisionGroup->setTitle(m_chinese ? QStringLiteral("碰撞检测") : QStringLiteral("Collision"));
	if (m_enabledCheck)
		m_enabledCheck->setText(m_chinese ? QStringLiteral("启用碰撞检测") : QStringLiteral("Enable collision check"));
	if (m_marginLabel)
		m_marginLabel->setText(m_chinese ? QStringLiteral("安全间隙") : QStringLiteral("Security margin"));
	if (m_listHintLabel)
	{
		m_listHintLabel->setText(
			m_chinese ? QStringLiteral("白/黑名单：同名单对象互不检测；仅对另一名单对象做碰撞检测。未入名单对象不参与名单规则（仍受默认 ACM 约束）。")
					  : QStringLiteral("White/Black lists: no checks within a list; only cross-list pairs are checked. "
									   "Unlisted bodies keep default ACM excludes only."));
	}
	if (m_poolLabel)
		m_poolLabel->setText(m_chinese ? QStringLiteral("场景 Mesh/Brep（未分配）") : QStringLiteral("Scene Mesh/Brep (unassigned)"));
	if (m_whiteLabel)
		m_whiteLabel->setText(m_chinese ? QStringLiteral("白名单") : QStringLiteral("White list"));
	if (m_blackLabel)
		m_blackLabel->setText(m_chinese ? QStringLiteral("黑名单") : QStringLiteral("Black list"));
	if (m_addWhiteBtn)
		m_addWhiteBtn->setText(m_chinese ? QStringLiteral("加入白名单") : QStringLiteral("Add to white"));
	if (m_addBlackBtn)
		m_addBlackBtn->setText(m_chinese ? QStringLiteral("加入黑名单") : QStringLiteral("Add to black"));
	if (m_removeWhiteBtn)
		m_removeWhiteBtn->setText(m_chinese ? QStringLiteral("移出白名单") : QStringLiteral("Remove"));
	if (m_removeBlackBtn)
		m_removeBlackBtn->setText(m_chinese ? QStringLiteral("移出黑名单") : QStringLiteral("Remove"));
	if (m_refreshObjectsBtn)
		m_refreshObjectsBtn->setText(m_chinese ? QStringLiteral("刷新对象") : QStringLiteral("Refresh objects"));
	if (m_planGroup)
		m_planGroup->setTitle(m_chinese ? QStringLiteral("运动规划") : QStringLiteral("Motion planning"));
	if (m_planHintLabel)
	{
		m_planHintLabel->setText(
			m_chinese ? QStringLiteral("选择起点/终点路点后规划；确认后插入两者之间（保留起终点）。")
					  : QStringLiteral("Pick start/goal waypoints, plan, then insert intermediates between them."));
	}
	if (m_startLabel)
		m_startLabel->setText(m_chinese ? QStringLiteral("起点路点") : QStringLiteral("Start waypoint"));
	if (m_endLabel)
		m_endLabel->setText(m_chinese ? QStringLiteral("终点路点") : QStringLiteral("Goal waypoint"));
	if (m_planBtn)
		m_planBtn->setText(m_chinese ? QStringLiteral("规划") : QStringLiteral("Plan"));
	if (m_clearPreviewBtn)
		m_clearPreviewBtn->setText(m_chinese ? QStringLiteral("清除路径预览") : QStringLiteral("Clear path preview"));
	if (m_confirmBtn)
		m_confirmBtn->setText(m_chinese ? QStringLiteral("确认插入中间点") : QStringLiteral("Insert intermediates"));
}

void RobotCollisionSettingsWidget::refreshEnabledState()
{
	const bool on = m_enabledCheck->isChecked();
	m_marginSpin->setEnabled(on);
	m_poolList->setEnabled(on);
	m_whiteTable->setEnabled(on);
	m_blackTable->setEnabled(on);
	m_addWhiteBtn->setEnabled(on);
	m_addBlackBtn->setEnabled(on);
	m_removeWhiteBtn->setEnabled(on);
	m_removeBlackBtn->setEnabled(on);
	m_refreshObjectsBtn->setEnabled(on);
}

void RobotCollisionSettingsWidget::onFieldChanged()
{
	if (m_block)
		return;
	m_whiteIds = tableIds(m_whiteTable);
	m_blackIds = tableIds(m_blackTable);
	refreshEnabledState();
	emit settingsChanged();
}
