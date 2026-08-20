/// @file MotionPathPlanDialog.cpp
/// @brief 起终点路点下拉 + 规划/插入

#include "MotionPathPlanDialog.h"

#include <QComboBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVariant>
#include <QVBoxLayout>

MotionPathPlanDialog::MotionPathPlanDialog(QWidget* parent) : QDialog(parent)
{
	setWindowModality(Qt::ApplicationModal);
	setMinimumWidth(420);

	auto* root = new QVBoxLayout(this);
	m_hintLabel = new QLabel(this);
	m_hintLabel->setWordWrap(true);
	root->addWidget(m_hintLabel);

	auto* form = new QFormLayout();
	m_startCombo = new QComboBox(this);
	m_endCombo = new QComboBox(this);
	m_startLabel = new QLabel(this);
	m_endLabel = new QLabel(this);
	form->addRow(m_startLabel, m_startCombo);
	form->addRow(m_endLabel, m_endCombo);
	root->addLayout(form);

	auto* row = new QHBoxLayout();
	m_planBtn = new QPushButton(this);
	m_planBtn->setProperty("btnRole", QVariant(QStringLiteral("primary")));
	m_clearBtn = new QPushButton(this);
	m_confirmBtn = new QPushButton(this);
	m_confirmBtn->setEnabled(false);
	m_confirmBtn->setProperty("btnRole", QVariant(QStringLiteral("primary")));
	m_closeBtn = new QPushButton(this);
	row->addWidget(m_planBtn);
	row->addWidget(m_clearBtn);
	row->addWidget(m_confirmBtn);
	row->addStretch(1);
	row->addWidget(m_closeBtn);
	root->addLayout(row);

	m_statusLabel = new QLabel(this);
	m_statusLabel->setWordWrap(true);
	root->addWidget(m_statusLabel);

	connect(m_planBtn, &QPushButton::clicked, this, &MotionPathPlanDialog::planClicked);
	connect(m_clearBtn, &QPushButton::clicked, this, &MotionPathPlanDialog::clearPreviewClicked);
	connect(m_confirmBtn, &QPushButton::clicked, this, &MotionPathPlanDialog::confirmInsertClicked);
	connect(m_closeBtn, &QPushButton::clicked, this, &QDialog::reject);

	retranslateUi();
}

void MotionPathPlanDialog::setUseChinese(const bool chinese)
{
	m_chinese = chinese;
	retranslateUi();
}

void MotionPathPlanDialog::setWaypoints(const QVector<MotionPathWaypointItem>& items)
{
	const QString keepStart = selectedStartId();
	const QString keepEnd = selectedEndId();
	m_startCombo->clear();
	m_endCombo->clear();
	for (const MotionPathWaypointItem& it : items)
	{
		m_startCombo->addItem(it.label, it.id);
		m_endCombo->addItem(it.label, it.id);
	}
	selectWaypointIds(keepStart, keepEnd);
	if (m_startCombo->count() >= 2 && m_startCombo->currentIndex() == m_endCombo->currentIndex())
		m_endCombo->setCurrentIndex(1);
}

void MotionPathPlanDialog::selectWaypointIds(const QString& startId, const QString& endId)
{
	auto selectId = [](QComboBox* box, const QString& id) {
		if (id.isEmpty() || !box)
			return;
		const int idx = box->findData(id);
		if (idx >= 0)
			box->setCurrentIndex(idx);
	};
	selectId(m_startCombo, startId);
	selectId(m_endCombo, endId);
}

void MotionPathPlanDialog::setStatusText(const QString& text)
{
	m_statusLabel->setText(text);
}

void MotionPathPlanDialog::setConfirmEnabled(const bool enabled)
{
	m_confirmBtn->setEnabled(enabled);
}

QString MotionPathPlanDialog::selectedStartId() const
{
	return m_startCombo->currentData().toString();
}

QString MotionPathPlanDialog::selectedEndId() const
{
	return m_endCombo->currentData().toString();
}

void MotionPathPlanDialog::retranslateUi()
{
	setWindowTitle(m_chinese ? QStringLiteral("路径规划（起终点）") : QStringLiteral("Motion plan (start/goal)"));
	m_hintLabel->setText(
		m_chinese ? QStringLiteral("从程序路点中选择起点与终点；规划生成中间点，确认后插入两者之间（保留起终点）。")
				  : QStringLiteral("Pick start/goal waypoints. Plan fills intermediates; confirm inserts them "
								   "between (keeps endpoints)."));
	m_startLabel->setText(m_chinese ? QStringLiteral("起点路点") : QStringLiteral("Start waypoint"));
	m_endLabel->setText(m_chinese ? QStringLiteral("终点路点") : QStringLiteral("Goal waypoint"));
	m_planBtn->setText(m_chinese ? QStringLiteral("规划") : QStringLiteral("Plan"));
	m_clearBtn->setText(m_chinese ? QStringLiteral("清除预览") : QStringLiteral("Clear preview"));
	m_confirmBtn->setText(m_chinese ? QStringLiteral("确认插入中间点") : QStringLiteral("Insert intermediates"));
	m_closeBtn->setText(m_chinese ? QStringLiteral("关闭") : QStringLiteral("Close"));
}
