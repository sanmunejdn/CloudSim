/// @file DeviceCommandPageWidget.cpp
/// @brief 设备指令页

#include "DeviceCommandPageWidget.h"

#include "BackendTypeIds.h"
#include "CustomDeviceBackendData.h"
#include "DevicePoseMotionPlayer.h"
#include "IRobotDocumentHost.h"
#include "IRobotMainWindowHost.h"
#include "IoSignalNetworkService.h"
#include "NamedSignalTable.h"
#include "UiIconDecorators.h"
#include "UiIconId.h"

#include <QAbstractItemView>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QInputDialog>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QPushButton>
#include <QStyle>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QVBoxLayout>
#include <algorithm>
#include <memory>

namespace
{
void setBtnRole(QPushButton* btn, const char* role)
{
	if (!btn)
	{
		return;
	}
	btn->setProperty("btnRole", QLatin1String(role));
	if (btn->style())
	{
		btn->style()->unpolish(btn);
		btn->style()->polish(btn);
	}
}

std::shared_ptr<CustomDeviceBackendData> currentDevicePtr(IRobotMainWindowHost* host, const QString& id)
{
	IRobotDocumentHost* doc = host ? host->document() : nullptr;
	if (!doc || id.isEmpty())
	{
		return nullptr;
	}
	return std::dynamic_pointer_cast<CustomDeviceBackendData>(doc->findObject(id.toStdString()));
}
} // namespace

DeviceCommandPageWidget::DeviceCommandPageWidget(QWidget* parent) : QWidget(parent)
{
	auto* root = new QVBoxLayout(this);
	root->setContentsMargins(8, 8, 8, 8);
	root->setSpacing(8);

	auto* deviceRow = new QHBoxLayout();
	m_deviceLabel = new QLabel(this);
	m_deviceCombo = new QComboBox(this);
	m_deviceCombo->setMinimumHeight(26);
	m_deviceCombo->setMaxVisibleItems(16);
	deviceRow->addWidget(m_deviceLabel);
	deviceRow->addWidget(m_deviceCombo, 1);
	root->addLayout(deviceRow);

	m_poseLabel = new QLabel(this);
	root->addWidget(m_poseLabel);
	m_poseList = new QListWidget(this);
	m_poseList->setMinimumHeight(120);
	root->addWidget(m_poseList, 1);

	auto* poseBtnRow = new QHBoxLayout();
	m_teachBtn = new QPushButton(this);
	m_addPoseBtn = new QPushButton(this);
	m_renamePoseBtn = new QPushButton(this);
	m_deletePoseBtn = new QPushButton(this);
	m_goPoseBtn = new QPushButton(this);
	setBtnRole(m_teachBtn, "secondary");
	setBtnRole(m_addPoseBtn, "secondary");
	setBtnRole(m_renamePoseBtn, "secondary");
	setBtnRole(m_deletePoseBtn, "danger");
	setBtnRole(m_goPoseBtn, "primary");
	poseBtnRow->addWidget(m_teachBtn);
	poseBtnRow->addWidget(m_addPoseBtn);
	poseBtnRow->addWidget(m_renamePoseBtn);
	poseBtnRow->addWidget(m_deletePoseBtn);
	poseBtnRow->addWidget(m_goPoseBtn);
	poseBtnRow->addStretch(1);
	root->addLayout(poseBtnRow);

	auto* goRow = new QHBoxLayout();
	m_durationLabel = new QLabel(this);
	m_goDurationSpin = new QDoubleSpinBox(this);
	m_goDurationSpin->setRange(0.0, 3600.0);
	m_goDurationSpin->setDecimals(2);
	m_goDurationSpin->setSingleStep(0.1);
	m_goDurationSpin->setValue(1.0);
	m_goDurationSpin->setSuffix(QStringLiteral(" s"));
	goRow->addWidget(m_durationLabel);
	goRow->addWidget(m_goDurationSpin);
	goRow->addStretch(1);
	root->addLayout(goRow);

	m_bindLabel = new QLabel(this);
	root->addWidget(m_bindLabel);
	m_bindTable = new QTableWidget(this);
	m_bindTable->setColumnCount(4);
	m_bindTable->horizontalHeader()->setStretchLastSection(true);
	m_bindTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Interactive);
	m_bindTable->setSelectionBehavior(QAbstractItemView::SelectRows);
	m_bindTable->setSelectionMode(QAbstractItemView::SingleSelection);
	m_bindTable->setMinimumHeight(140);
	root->addWidget(m_bindTable, 1);

	auto* bindBtnRow = new QHBoxLayout();
	m_addBindBtn = new QPushButton(this);
	m_deleteBindBtn = new QPushButton(this);
	setBtnRole(m_addBindBtn, "secondary");
	setBtnRole(m_deleteBindBtn, "danger");
	bindBtnRow->addWidget(m_addBindBtn);
	bindBtnRow->addWidget(m_deleteBindBtn);
	bindBtnRow->addStretch(1);
	root->addLayout(bindBtnRow);

	auto* runRow = new QHBoxLayout();
	m_stopBtn = new QPushButton(this);
	setBtnRole(m_stopBtn, "danger");
	UiIconDecorators::apply(m_stopBtn, UiIconId::Stop, UiIconDecorators::IconPlacement::Leading,
							UiIcons::Size::Medium);
	m_statusLabel = new QLabel(this);
	runRow->addWidget(m_stopBtn);
	runRow->addWidget(m_statusLabel, 1);
	root->addLayout(runRow);

	connect(m_deviceCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
			&DeviceCommandPageWidget::onDeviceChanged);
	connect(m_teachBtn, &QPushButton::clicked, this, &DeviceCommandPageWidget::onTeachPose);
	connect(m_addPoseBtn, &QPushButton::clicked, this, &DeviceCommandPageWidget::onAddPose);
	connect(m_renamePoseBtn, &QPushButton::clicked, this, &DeviceCommandPageWidget::onRenamePose);
	connect(m_deletePoseBtn, &QPushButton::clicked, this, &DeviceCommandPageWidget::onDeletePose);
	connect(m_goPoseBtn, &QPushButton::clicked, this, &DeviceCommandPageWidget::onGoToPose);
	connect(m_addBindBtn, &QPushButton::clicked, this, &DeviceCommandPageWidget::onAddBinding);
	connect(m_deleteBindBtn, &QPushButton::clicked, this, &DeviceCommandPageWidget::onDeleteBinding);
	connect(m_bindTable, &QTableWidget::cellChanged, this, &DeviceCommandPageWidget::onBindingCellChanged);
	connect(m_stopBtn, &QPushButton::clicked, this, &DeviceCommandPageWidget::onStopMotion);

	retranslateUi();
}

QString DeviceCommandPageWidget::i18n(const QString& en, const QString& zh) const
{
	if (m_host)
	{
		return m_host->i18n(en, zh);
	}
	return m_useChinese ? zh : en;
}

void DeviceCommandPageWidget::setUseChinese(const bool chinese)
{
	m_useChinese = chinese;
	retranslateUi();
}

void DeviceCommandPageWidget::setHost(IRobotMainWindowHost* host)
{
	m_host = host;
	refreshDevices();
}

void DeviceCommandPageWidget::setNetwork(IoSignalNetworkService* network)
{
	m_network = network;
	refreshDiSignalOptions();
	fillBindingTable();
}

void DeviceCommandPageWidget::setMotionPlayer(DevicePoseMotionPlayer* player)
{
	if (m_player)
	{
		disconnect(m_player, &DevicePoseMotionPlayer::statusChanged, this,
				   &DeviceCommandPageWidget::onPlayerStatusChanged);
	}
	m_player = player;
	if (m_player)
	{
		connect(m_player, &DevicePoseMotionPlayer::statusChanged, this,
				&DeviceCommandPageWidget::onPlayerStatusChanged);
	}
	updateStatusLabel();
}

void DeviceCommandPageWidget::retranslateUi()
{
	m_deviceLabel->setText(i18n(QStringLiteral("Device"), QStringLiteral("设备")));
	m_poseLabel->setText(i18n(QStringLiteral("Poses"), QStringLiteral("姿态库")));
	m_teachBtn->setText(i18n(QStringLiteral("Teach"), QStringLiteral("示教当前")));
	m_addPoseBtn->setText(i18n(QStringLiteral("New"), QStringLiteral("新建")));
	m_renamePoseBtn->setText(i18n(QStringLiteral("Rename"), QStringLiteral("重命名")));
	m_deletePoseBtn->setText(i18n(QStringLiteral("Delete"), QStringLiteral("删除")));
	m_goPoseBtn->setText(i18n(QStringLiteral("Go to"), QStringLiteral("运动到此")));
	m_durationLabel->setText(i18n(QStringLiteral("Go duration"), QStringLiteral("运动时长")));
	m_bindLabel->setText(i18n(QStringLiteral("DI → Pose bindings"), QStringLiteral("信号绑定（DI 上升沿）")));
	m_addBindBtn->setText(i18n(QStringLiteral("Add"), QStringLiteral("添加")));
	m_deleteBindBtn->setText(i18n(QStringLiteral("Delete"), QStringLiteral("删除")));
	m_stopBtn->setText(i18n(QStringLiteral("Stop"), QStringLiteral("停止")));
	m_bindTable->setHorizontalHeaderLabels(
		{i18n(QStringLiteral("On"), QStringLiteral("启用")), i18n(QStringLiteral("DI"), QStringLiteral("DI")),
		 i18n(QStringLiteral("Pose"), QStringLiteral("姿态")),
		 i18n(QStringLiteral("Duration (s)"), QStringLiteral("时长 (s)"))});
	updateStatusLabel();
}

void DeviceCommandPageWidget::refreshDevices()
{
	const QString keep = currentDeviceId();
	m_deviceCombo->blockSignals(true);
	m_deviceCombo->clear();
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	if (doc)
	{
		for (const QString& id :
			 doc->documentData().findByClassName(QString::fromUtf8(backend_type::kClassCustomDevice)))
		{
			QString label = doc->documentData().displayName(id);
			if (label.isEmpty())
			{
				label = id;
			}
			m_deviceCombo->addItem(label, id);
		}
	}
	const int fi = m_deviceCombo->findData(keep);
	m_deviceCombo->setCurrentIndex(fi >= 0 ? fi : (m_deviceCombo->count() > 0 ? 0 : -1));
	m_deviceCombo->blockSignals(false);
	reloadCurrentDeviceUi();
}

void DeviceCommandPageWidget::refreshDiSignalOptions()
{
	fillBindingTable();
}

void DeviceCommandPageWidget::reloadCurrentDeviceUi()
{
	fillPoseList();
	fillBindingTable();
}

QString DeviceCommandPageWidget::currentDeviceId() const
{
	return m_deviceCombo->currentData().toString();
}

RobotIo::NamedSignalTable* DeviceCommandPageWidget::currentDeviceSignalTable() const
{
	if (!m_network)
	{
		return nullptr;
	}
	return m_network->table(currentDeviceId());
}

void DeviceCommandPageWidget::onDeviceChanged(int)
{
	reloadCurrentDeviceUi();
}

void DeviceCommandPageWidget::fillPoseList()
{
	m_poseList->clear();
	const auto device = currentDevicePtr(m_host, currentDeviceId());
	if (!device)
	{
		return;
	}
	for (const CustomDeviceNamedPose& p : device->namedPoses())
	{
		auto* item = new QListWidgetItem(QString::fromStdString(p.name), m_poseList);
		item->setData(Qt::UserRole, QString::fromStdString(p.id));
	}
}

void DeviceCommandPageWidget::fillBindingTable()
{
	m_blockBindingEdit = true;
	m_bindTable->setRowCount(0);
	const auto device = currentDevicePtr(m_host, currentDeviceId());
	if (!device)
	{
		m_blockBindingEdit = false;
		return;
	}

	QStringList diNames;
	if (RobotIo::NamedSignalTable* signalTable = currentDeviceSignalTable())
	{
		for (const RobotIo::SignalDef& s : signalTable->entries())
		{
			if (s.kind == RobotIo::SignalKind::DI && !s.name.empty())
			{
				diNames << QString::fromStdString(s.name);
			}
		}
	}
	QStringList poseNames;
	QStringList poseIds;
	for (const CustomDeviceNamedPose& p : device->namedPoses())
	{
		poseNames << QString::fromStdString(p.name);
		poseIds << QString::fromStdString(p.id);
	}

	for (const CustomDevicePoseSignalBinding& b : device->poseSignalBindings())
	{
		const int row = m_bindTable->rowCount();
		m_bindTable->insertRow(row);

		auto* enItem = new QTableWidgetItem();
		enItem->setFlags(Qt::ItemIsUserCheckable | Qt::ItemIsEnabled | Qt::ItemIsSelectable);
		enItem->setCheckState(b.enabled ? Qt::Checked : Qt::Unchecked);
		enItem->setData(Qt::UserRole, QString::fromStdString(b.id));
		m_bindTable->setItem(row, 0, enItem);

		auto* diCombo = new QComboBox(m_bindTable);
		diCombo->addItems(diNames);
		const int diIdx = diCombo->findText(QString::fromStdString(b.signalName));
		if (diIdx >= 0)
		{
			diCombo->setCurrentIndex(diIdx);
		}
		else if (!b.signalName.empty())
		{
			diCombo->addItem(QString::fromStdString(b.signalName) +
							 i18n(QStringLiteral(" (missing)"), QStringLiteral("（已失效）")));
			diCombo->setCurrentIndex(diCombo->count() - 1);
			diCombo->setStyleSheet(QStringLiteral("color: #c62828;"));
		}
		m_bindTable->setCellWidget(row, 1, diCombo);
		connect(diCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
				[this](int) {
					if (!m_blockBindingEdit)
					{
						persistBindingsFromTable();
					}
				});

		auto* poseCombo = new QComboBox(m_bindTable);
		for (int i = 0; i < poseNames.size(); ++i)
		{
			poseCombo->addItem(poseNames[i], poseIds[i]);
		}
		const int pi = poseCombo->findData(QString::fromStdString(b.poseId));
		poseCombo->setCurrentIndex(pi >= 0 ? pi : 0);
		m_bindTable->setCellWidget(row, 2, poseCombo);
		connect(poseCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
				[this](int) {
					if (!m_blockBindingEdit)
					{
						persistBindingsFromTable();
					}
				});

		auto* durItem = new QTableWidgetItem(QString::number(b.durationSec, 'f', 2));
		m_bindTable->setItem(row, 3, durItem);
	}
	m_blockBindingEdit = false;
}

void DeviceCommandPageWidget::persistBindingsFromTable()
{
	const auto device = currentDevicePtr(m_host, currentDeviceId());
	if (!device || m_blockBindingEdit)
	{
		return;
	}
	std::vector<CustomDevicePoseSignalBinding> bindings;
	bindings.reserve(static_cast<size_t>(m_bindTable->rowCount()));
	for (int row = 0; row < m_bindTable->rowCount(); ++row)
	{
		CustomDevicePoseSignalBinding b;
		QTableWidgetItem* en = m_bindTable->item(row, 0);
		if (en)
		{
			b.id = en->data(Qt::UserRole).toString().toStdString();
			b.enabled = en->checkState() == Qt::Checked;
		}
		if (b.id.empty())
		{
			b.id = makeCustomDevicePoseBindingId();
		}
		if (auto* diCombo = qobject_cast<QComboBox*>(m_bindTable->cellWidget(row, 1)))
		{
			QString name = diCombo->currentText();
			const int cut = name.indexOf(QStringLiteral("（"));
			const int cutEn = name.indexOf(QStringLiteral(" ("));
			if (cut > 0)
			{
				name = name.left(cut);
			}
			else if (cutEn > 0)
			{
				name = name.left(cutEn);
			}
			b.signalName = name.trimmed().toStdString();
		}
		if (auto* poseCombo = qobject_cast<QComboBox*>(m_bindTable->cellWidget(row, 2)))
		{
			b.poseId = poseCombo->currentData().toString().toStdString();
		}
		if (QTableWidgetItem* dur = m_bindTable->item(row, 3))
		{
			bool ok = false;
			const double v = dur->text().toDouble(&ok);
			b.durationSec = ok ? std::max(0.0, v) : 1.0;
		}
		bindings.push_back(std::move(b));
	}
	device->setPoseSignalBindings(bindings);
}

void DeviceCommandPageWidget::onTeachPose()
{
	const auto device = currentDevicePtr(m_host, currentDeviceId());
	if (!device)
	{
		QMessageBox::information(this, i18n(QStringLiteral("Device"), QStringLiteral("设备")),
								 i18n(QStringLiteral("Select a custom device first."),
									  QStringLiteral("请先选择自定义设备。")));
		return;
	}
	device->syncAxesFromJoints();
	device->ensureQSize();
	bool ok = false;
	const QString name = QInputDialog::getText(
		this, i18n(QStringLiteral("Teach pose"), QStringLiteral("示教姿态")),
		i18n(QStringLiteral("Pose name"), QStringLiteral("姿态名称")), QLineEdit::Normal,
		i18n(QStringLiteral("Pose"), QStringLiteral("姿态")), &ok);
	if (!ok || name.trimmed().isEmpty())
	{
		return;
	}
	CustomDeviceNamedPose p;
	p.id = makeCustomDevicePoseId();
	p.name = name.trimmed().toStdString();
	p.q = device->qValues();
	auto poses = device->namedPoses();
	poses.push_back(std::move(p));
	device->setNamedPoses(poses);
	fillPoseList();
	fillBindingTable();
}

void DeviceCommandPageWidget::onAddPose()
{
	onTeachPose();
}

void DeviceCommandPageWidget::onRenamePose()
{
	const auto device = currentDevicePtr(m_host, currentDeviceId());
	QListWidgetItem* item = m_poseList->currentItem();
	if (!device || !item)
	{
		return;
	}
	const QString poseId = item->data(Qt::UserRole).toString();
	bool ok = false;
	const QString name = QInputDialog::getText(this, i18n(QStringLiteral("Rename"), QStringLiteral("重命名")),
											   i18n(QStringLiteral("Pose name"), QStringLiteral("姿态名称")),
											   QLineEdit::Normal, item->text(), &ok);
	if (!ok || name.trimmed().isEmpty())
	{
		return;
	}
	auto poses = device->namedPoses();
	for (CustomDeviceNamedPose& p : poses)
	{
		if (p.id == poseId.toStdString())
		{
			p.name = name.trimmed().toStdString();
			break;
		}
	}
	device->setNamedPoses(poses);
	fillPoseList();
	fillBindingTable();
}

void DeviceCommandPageWidget::onDeletePose()
{
	const auto device = currentDevicePtr(m_host, currentDeviceId());
	QListWidgetItem* item = m_poseList->currentItem();
	if (!device || !item)
	{
		return;
	}
	const QString poseId = item->data(Qt::UserRole).toString();
	auto poses = device->namedPoses();
	poses.erase(std::remove_if(poses.begin(), poses.end(),
							   [&](const CustomDeviceNamedPose& p) { return p.id == poseId.toStdString(); }),
				poses.end());
	device->setNamedPoses(poses);
	auto bindings = device->poseSignalBindings();
	bindings.erase(std::remove_if(bindings.begin(), bindings.end(),
								  [&](const CustomDevicePoseSignalBinding& b) {
									  return b.poseId == poseId.toStdString();
								  }),
				   bindings.end());
	device->setPoseSignalBindings(bindings);
	fillPoseList();
	fillBindingTable();
}

void DeviceCommandPageWidget::onGoToPose()
{
	const auto device = currentDevicePtr(m_host, currentDeviceId());
	QListWidgetItem* item = m_poseList->currentItem();
	if (!device || !item || !m_player)
	{
		return;
	}
	const QString poseId = item->data(Qt::UserRole).toString();
	const CustomDeviceNamedPose* pose = device->findNamedPose(poseId.toStdString());
	if (!pose)
	{
		return;
	}
	(void)m_player->start(currentDeviceId(), QString::fromStdString(pose->name), pose->q,
						  m_goDurationSpin->value());
}

void DeviceCommandPageWidget::onAddBinding()
{
	const auto device = currentDevicePtr(m_host, currentDeviceId());
	if (!device)
	{
		return;
	}
	RobotIo::NamedSignalTable* signalTable = currentDeviceSignalTable();
	if (!signalTable)
	{
		return;
	}
	QString firstDi;
	for (const RobotIo::SignalDef& s : signalTable->entries())
	{
		if (s.kind == RobotIo::SignalKind::DI && !s.name.empty())
		{
			firstDi = QString::fromStdString(s.name);
			break;
		}
	}
	if (firstDi.isEmpty())
	{
		QMessageBox::information(
			this, i18n(QStringLiteral("Signals"), QStringLiteral("信号")),
			i18n(QStringLiteral("Define a DI on the Signals page first."),
				 QStringLiteral("请先在「信号」页添加 DI。")));
		return;
	}
	if (device->namedPoses().empty())
	{
		QMessageBox::information(this, i18n(QStringLiteral("Poses"), QStringLiteral("姿态库")),
								 i18n(QStringLiteral("Teach a pose first."), QStringLiteral("请先示教一个姿态。")));
		return;
	}
	CustomDevicePoseSignalBinding b;
	b.id = makeCustomDevicePoseBindingId();
	b.signalName = firstDi.toStdString();
	b.poseId = device->namedPoses().front().id;
	b.durationSec = m_goDurationSpin->value();
	b.enabled = true;
	auto bindings = device->poseSignalBindings();
	bindings.push_back(std::move(b));
	device->setPoseSignalBindings(bindings);
	fillBindingTable();
}

void DeviceCommandPageWidget::onDeleteBinding()
{
	const auto device = currentDevicePtr(m_host, currentDeviceId());
	const int row = m_bindTable->currentRow();
	if (!device || row < 0)
	{
		return;
	}
	QTableWidgetItem* en = m_bindTable->item(row, 0);
	const QString bindId = en ? en->data(Qt::UserRole).toString() : QString();
	auto bindings = device->poseSignalBindings();
	bindings.erase(std::remove_if(bindings.begin(), bindings.end(),
								  [&](const CustomDevicePoseSignalBinding& b) {
									  return b.id == bindId.toStdString();
								  }),
				   bindings.end());
	device->setPoseSignalBindings(bindings);
	fillBindingTable();
}

void DeviceCommandPageWidget::onBindingCellChanged(int, int)
{
	if (!m_blockBindingEdit)
	{
		persistBindingsFromTable();
	}
}

void DeviceCommandPageWidget::onStopMotion()
{
	if (m_player)
	{
		m_player->stopAll();
	}
}

void DeviceCommandPageWidget::onPlayerStatusChanged()
{
	updateStatusLabel();
}

void DeviceCommandPageWidget::updateStatusLabel()
{
	if (!m_player || !m_player->isBusy())
	{
		m_statusLabel->setText(i18n(QStringLiteral("Idle"), QStringLiteral("空闲")));
		return;
	}
	m_statusLabel->setText(i18n(QStringLiteral("Moving to %1").arg(m_player->statusText()),
								QStringLiteral("正在运动到 %1").arg(m_player->statusText())));
}
