/// @file IoSignalPageWidget.cpp
/// @brief IO 信号页

#include "IoSignalPageWidget.h"

#include "IoSignalNetworkService.h"
#include "NamedSignalIoSink.h"
#include "SignalConnectionStationWidget.h"

#include <QAbstractItemView>
#include <QComboBox>
#include <QDialog>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QSignalBlocker>
#include <QTableWidget>
#include <QVBoxLayout>

namespace
{
enum Col
{
	ColName = 0,
	ColKind,
	ColPort,
	ColValue,
	ColForce,
	ColCount
};
} // namespace

IoSignalPageWidget::IoSignalPageWidget(QWidget* parent) : QWidget(parent)
{
	setupUi();
	updateUiLabels();
}

void IoSignalPageWidget::setupUi()
{
	auto* root = new QVBoxLayout(this);
	root->setContentsMargins(6, 6, 6, 6);
	root->setSpacing(6);

	auto* ownerRow = new QHBoxLayout;
	auto* ownerLabel = new QLabel(this);
	ownerLabel->setObjectName(QStringLiteral("ioOwnerLabel"));
	m_ownerCombo = new QComboBox(this);
	ownerRow->addWidget(ownerLabel);
	ownerRow->addWidget(m_ownerCombo, 1);
	root->addLayout(ownerRow);

	m_tableWidget = new QTableWidget(this);
	m_tableWidget->setColumnCount(ColCount);
	m_tableWidget->horizontalHeader()->setStretchLastSection(true);
	m_tableWidget->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
	m_tableWidget->setSelectionBehavior(QAbstractItemView::SelectRows);
	m_tableWidget->setSelectionMode(QAbstractItemView::SingleSelection);
	m_tableWidget->setEditTriggers(QAbstractItemView::DoubleClicked | QAbstractItemView::EditKeyPressed |
								   QAbstractItemView::AnyKeyPressed);
	m_tableWidget->verticalHeader()->setVisible(false);
	root->addWidget(m_tableWidget, 1);

	auto* btnRow = new QHBoxLayout;
	m_addBtn = new QPushButton(this);
	m_removeBtn = new QPushButton(this);
	m_resetBtn = new QPushButton(this);
	m_stationBtn = new QPushButton(this);
	btnRow->addWidget(m_addBtn);
	btnRow->addWidget(m_removeBtn);
	btnRow->addWidget(m_resetBtn);
	btnRow->addWidget(m_stationBtn);
	btnRow->addStretch(1);
	root->addLayout(btnRow);

	connect(m_ownerCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
			&IoSignalPageWidget::onOwnerChanged);
	connect(m_addBtn, &QPushButton::clicked, this, &IoSignalPageWidget::onAddClicked);
	connect(m_removeBtn, &QPushButton::clicked, this, &IoSignalPageWidget::onRemoveClicked);
	connect(m_resetBtn, &QPushButton::clicked, this, &IoSignalPageWidget::onResetDefaultsClicked);
	connect(m_stationBtn, &QPushButton::clicked, this, &IoSignalPageWidget::onOpenStationClicked);
	connect(m_tableWidget, &QTableWidget::cellChanged, this, &IoSignalPageWidget::onCellChanged);
}

void IoSignalPageWidget::setUseChinese(const bool chinese)
{
	m_useChinese = chinese;
	updateUiLabels();
	rebuildTable();
	if (m_stationDialog && m_stationWidget)
	{
		m_stationDialog->setWindowTitle(chinese ? QStringLiteral("信号连接站") : QStringLiteral("Signal station"));
		m_stationWidget->setUseChinese(chinese);
	}
}

void IoSignalPageWidget::setNetwork(IoSignalNetworkService* network)
{
	if (m_network)
	{
		disconnect(m_network, nullptr, this, nullptr);
	}
	m_network = network;
	if (m_network)
	{
		connect(m_network, &IoSignalNetworkService::networkChanged, this, &IoSignalPageWidget::onNetworkChanged);
	}
	refreshOwners();
}

void IoSignalPageWidget::setCurrentOwnerId(const QString& ownerId)
{
	const int idx = m_ownerCombo->findData(ownerId);
	if (idx >= 0)
	{
		m_ownerCombo->setCurrentIndex(idx);
	}
}

QString IoSignalPageWidget::currentOwnerId() const
{
	return m_ownerCombo->currentData().toString();
}

void IoSignalPageWidget::refreshOwners()
{
	const QString keep = currentOwnerId();
	m_ownerCombo->blockSignals(true);
	m_ownerCombo->clear();
	if (m_network)
	{
		for (const QString& id : m_network->ownerIds())
		{
			const QString kindTag = m_network->ownerKind(id) == IoSignalOwnerKind::Device
										? (m_useChinese ? QStringLiteral("设备") : QStringLiteral("Device"))
										: (m_useChinese ? QStringLiteral("机器人") : QStringLiteral("Robot"));
			const QString label = QStringLiteral("%1 [%2]").arg(m_network->displayName(id), kindTag);
			m_ownerCombo->addItem(label, id);
		}
	}
	const int fi = m_ownerCombo->findData(keep);
	m_ownerCombo->setCurrentIndex(fi >= 0 ? fi : (m_ownerCombo->count() > 0 ? 0 : -1));
	m_ownerCombo->blockSignals(false);
	bindCurrentOwner();
}

void IoSignalPageWidget::refreshFromModel()
{
	refreshOwners();
}

void IoSignalPageWidget::onNetworkChanged()
{
	refreshOwners();
}

void IoSignalPageWidget::onOwnerChanged(int)
{
	bindCurrentOwner();
	emit currentOwnerChanged(currentOwnerId());
}

void IoSignalPageWidget::bindCurrentOwner()
{
	// clear() 会先删 sink 再 networkChanged；QPointer 避免对悬空指针 disconnect
	if (m_sink)
	{
		disconnect(m_sink, nullptr, this, nullptr);
	}
	m_table = nullptr;
	m_sink.clear();
	const QString id = currentOwnerId();
	if (m_network && !id.isEmpty())
	{
		m_table = m_network->table(id);
		m_sink = m_network->sink(id);
		if (m_sink)
		{
			connect(m_sink, &NamedSignalIoSink::ioValuesChanged, this, &IoSignalPageWidget::onSinkValuesChanged);
		}
	}
	rebuildTable();
}

void IoSignalPageWidget::flushDeviceIfNeeded()
{
	if (!m_network)
	{
		return;
	}
	const QString id = currentOwnerId();
	if (id.isEmpty() || m_network->ownerKind(id) != IoSignalOwnerKind::Device)
	{
		return;
	}
	// 设备表由 Controller flushDeviceIoTablesToDocument 在保存时写回；编辑时先更新内存 table
}

void IoSignalPageWidget::updateUiLabels()
{
	const bool zh = m_useChinese;
	if (QLabel* ownerLabel = findChild<QLabel*>(QStringLiteral("ioOwnerLabel")))
	{
		ownerLabel->setText(zh ? QStringLiteral("所属") : QStringLiteral("Owner"));
	}
	m_tableWidget->setHorizontalHeaderLabels({
		zh ? QStringLiteral("名称") : QStringLiteral("Name"),
		zh ? QStringLiteral("类型") : QStringLiteral("Kind"),
		zh ? QStringLiteral("端口") : QStringLiteral("Port"),
		zh ? QStringLiteral("值") : QStringLiteral("Value"),
		zh ? QStringLiteral("强制") : QStringLiteral("Force"),
	});
	m_addBtn->setText(zh ? QStringLiteral("添加") : QStringLiteral("Add"));
	m_removeBtn->setText(zh ? QStringLiteral("删除") : QStringLiteral("Remove"));
	m_resetBtn->setText(zh ? QStringLiteral("重置默认") : QStringLiteral("Reset defaults"));
	m_stationBtn->setText(zh ? QStringLiteral("信号连接站") : QStringLiteral("Signal station"));
}

RobotIo::SignalKind IoSignalPageWidget::kindFromComboText(const QString& text) const
{
	RobotIo::SignalKind k = RobotIo::SignalKind::DI;
	(void)RobotIo::NamedSignalTable::kindFromString(text.toStdString(), k);
	return k;
}

bool IoSignalPageWidget::parseBoolText(const QString& text)
{
	const QString t = text.trimmed();
	if (t.isEmpty() || t == QStringLiteral("0") || t.compare(QStringLiteral("false"), Qt::CaseInsensitive) == 0 ||
		t.compare(QStringLiteral("off"), Qt::CaseInsensitive) == 0)
	{
		return false;
	}
	return t == QStringLiteral("1") || t.compare(QStringLiteral("true"), Qt::CaseInsensitive) == 0 ||
		   t.compare(QStringLiteral("on"), Qt::CaseInsensitive) == 0 || t.toInt() != 0;
}

void IoSignalPageWidget::syncForceItemFlags(const int row)
{
	if (!m_table || row < 0 || row >= static_cast<int>(m_table->entries().size()))
	{
		return;
	}
	QTableWidgetItem* forceItem = m_tableWidget->item(row, ColForce);
	if (!forceItem)
	{
		return;
	}
	const RobotIo::SignalDef& s = m_table->entries()[static_cast<size_t>(row)];
	const bool canForce = (s.kind == RobotIo::SignalKind::DI && s.simForceable);
	if (canForce)
	{
		forceItem->setFlags((forceItem->flags() | Qt::ItemIsUserCheckable | Qt::ItemIsEnabled) & ~Qt::ItemIsEditable);
	}
	else
	{
		forceItem->setCheckState(Qt::Unchecked);
		forceItem->setFlags(forceItem->flags() & ~Qt::ItemIsUserCheckable & ~Qt::ItemIsEnabled);
	}
}

void IoSignalPageWidget::rebuildTable()
{
	m_updating = true;
	m_tableWidget->setRowCount(0);
	if (!m_table)
	{
		m_updating = false;
		return;
	}
	const auto& signalEntries = m_table->entries();
	m_tableWidget->setRowCount(static_cast<int>(signalEntries.size()));
	for (int row = 0; row < static_cast<int>(signalEntries.size()); ++row)
	{
		const RobotIo::SignalDef& s = signalEntries[static_cast<size_t>(row)];
		m_tableWidget->setItem(row, ColName, new QTableWidgetItem(QString::fromStdString(s.name)));
		auto* kindCombo = new QComboBox(m_tableWidget);
		kindCombo->addItems({QStringLiteral("DI"), QStringLiteral("DO"), QStringLiteral("AI"), QStringLiteral("AO")});
		kindCombo->setCurrentText(QString::fromStdString(RobotIo::NamedSignalTable::kindToString(s.kind)));
		m_tableWidget->setCellWidget(row, ColKind, kindCombo);
		connect(kindCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
				[this, row](int) { applyRowToModel(row); });
		m_tableWidget->setItem(row, ColPort, new QTableWidgetItem(QString::number(s.port)));

		auto* valueItem = new QTableWidgetItem;
		valueItem->setFlags(valueItem->flags() | Qt::ItemIsEditable);
		m_tableWidget->setItem(row, ColValue, valueItem);

		auto* forceItem = new QTableWidgetItem;
		forceItem->setFlags((forceItem->flags() | Qt::ItemIsUserCheckable) & ~Qt::ItemIsEditable);
		forceItem->setCheckState(Qt::Unchecked);
		m_tableWidget->setItem(row, ColForce, forceItem);
		syncForceItemFlags(row);
	}
	m_updating = false;
	syncValueColumnsFromSink();
}

void IoSignalPageWidget::syncValueColumnsFromSink()
{
	if (!m_table || !m_tableWidget)
	{
		return;
	}
	const QSignalBlocker blocker(m_tableWidget);
	const auto& signalEntries = m_table->entries();
	for (int row = 0; row < m_tableWidget->rowCount() && row < static_cast<int>(signalEntries.size()); ++row)
	{
		const RobotIo::SignalDef& s = signalEntries[static_cast<size_t>(row)];
		QTableWidgetItem* valueItem = m_tableWidget->item(row, ColValue);
		QTableWidgetItem* forceItem = m_tableWidget->item(row, ColForce);
		if (!valueItem)
		{
			continue;
		}
		QString text = QStringLiteral("-");
		if (m_sink)
		{
			switch (s.kind)
			{
			case RobotIo::SignalKind::DI:
			{
				bool v = false;
				m_sink->getDigitalInput(s.port, &v);
				text = v ? QStringLiteral("1") : QStringLiteral("0");
				if (forceItem)
				{
					forceItem->setCheckState(m_sink->isDigitalInputForced(s.port) ? Qt::Checked : Qt::Unchecked);
				}
				break;
			}
			case RobotIo::SignalKind::DO:
			{
				bool v = false;
				m_sink->getDigitalOutput(s.port, &v);
				text = v ? QStringLiteral("1") : QStringLiteral("0");
				break;
			}
			case RobotIo::SignalKind::AI:
			{
				double v = 0.0;
				m_sink->getAnalogInput(s.port, &v);
				text = QString::number(v, 'g', 6);
				break;
			}
			case RobotIo::SignalKind::AO:
			{
				double v = 0.0;
				m_sink->getAnalogOutput(s.port, &v);
				text = QString::number(v, 'g', 6);
				break;
			}
			}
		}
		valueItem->setText(text);
	}
}

void IoSignalPageWidget::applyValueCellToSink(const int row)
{
	if (!m_sink || !m_table || row < 0 || row >= static_cast<int>(m_table->entries().size()))
	{
		return;
	}
	QTableWidgetItem* valueItem = m_tableWidget->item(row, ColValue);
	if (!valueItem)
	{
		return;
	}
	const RobotIo::SignalDef& s = m_table->entries()[static_cast<size_t>(row)];
	const QString text = valueItem->text().trimmed();
	switch (s.kind)
	{
	case RobotIo::SignalKind::DI:
	{
		const bool v = parseBoolText(text);
		QTableWidgetItem* forceItem = m_tableWidget->item(row, ColForce);
		const bool forced = forceItem && forceItem->checkState() == Qt::Checked;
		if (forced || s.simForceable)
		{
			m_sink->setDigitalInputForced(s.port, v);
			if (forceItem)
			{
				const QSignalBlocker blocker(m_tableWidget);
				forceItem->setCheckState(Qt::Checked);
			}
		}
		else
		{
			m_sink->setDigitalInput(s.port, v);
		}
		break;
	}
	case RobotIo::SignalKind::DO:
		m_sink->setDigitalOutput(s.port, parseBoolText(text));
		break;
	case RobotIo::SignalKind::AI:
	{
		bool ok = false;
		const double v = text.toDouble(&ok);
		m_sink->setAnalogInput(s.port, ok ? v : 0.0);
		break;
	}
	case RobotIo::SignalKind::AO:
	{
		bool ok = false;
		const double v = text.toDouble(&ok);
		m_sink->setAnalogOutput(s.port, ok ? v : 0.0);
		break;
	}
	}
}

void IoSignalPageWidget::applyRowToModel(const int row)
{
	if (m_updating || !m_table || row < 0 || row >= static_cast<int>(m_table->entriesMut().size()))
	{
		return;
	}
	const QString oldName = QString::fromStdString(m_table->entries()[static_cast<size_t>(row)].name);
	RobotIo::SignalDef& s = m_table->entriesMut()[static_cast<size_t>(row)];
	if (QTableWidgetItem* nameItem = m_tableWidget->item(row, ColName))
	{
		s.name = nameItem->text().trimmed().toStdString();
	}
	if (auto* kindCombo = qobject_cast<QComboBox*>(m_tableWidget->cellWidget(row, ColKind)))
	{
		s.kind = kindFromComboText(kindCombo->currentText());
		s.simForceable = (s.kind == RobotIo::SignalKind::DI);
	}
	if (QTableWidgetItem* portItem = m_tableWidget->item(row, ColPort))
	{
		s.port = portItem->text().toInt();
	}
	syncForceItemFlags(row);
	if (m_network && oldName != QString::fromStdString(s.name))
	{
		m_network->removeWiresTouchingSignal(currentOwnerId(), oldName);
	}
	flushDeviceIfNeeded();
	emit signalTableEdited();
}

void IoSignalPageWidget::onCellChanged(const int row, const int column)
{
	if (m_updating)
	{
		return;
	}
	if (column == ColValue)
	{
		applyValueCellToSink(row);
		return;
	}
	if (column == ColForce)
	{
		if (!m_sink || !m_table || row < 0 || row >= static_cast<int>(m_table->entries().size()))
		{
			return;
		}
		const RobotIo::SignalDef& s = m_table->entries()[static_cast<size_t>(row)];
		if (s.kind != RobotIo::SignalKind::DI || !s.simForceable)
		{
			return;
		}
		QTableWidgetItem* forceItem = m_tableWidget->item(row, ColForce);
		if (!forceItem)
		{
			return;
		}
		if (forceItem->checkState() == Qt::Checked)
		{
			bool v = false;
			if (QTableWidgetItem* valueItem = m_tableWidget->item(row, ColValue))
			{
				v = parseBoolText(valueItem->text());
			}
			else
			{
				m_sink->getDigitalInput(s.port, &v);
			}
			m_sink->setDigitalInputForced(s.port, v);
		}
		else
		{
			m_sink->clearDigitalInputForced(s.port);
		}
		syncValueColumnsFromSink();
		return;
	}
	if (column == ColName || column == ColPort)
	{
		applyRowToModel(row);
	}
}

void IoSignalPageWidget::onAddClicked()
{
	if (!m_table)
	{
		return;
	}
	RobotIo::SignalDef s;
	s.id = RobotIo::NamedSignalTable::makeSignalId();
	s.name = "Signal" + std::to_string(m_table->entries().size() + 1);
	s.kind = RobotIo::SignalKind::DI;
	s.port = static_cast<int>(m_table->entries().size());
	s.simForceable = true;
	m_table->entriesMut().push_back(std::move(s));
	if (m_sink)
	{
		m_sink->resetRuntimeFromTable(true);
	}
	rebuildTable();
	flushDeviceIfNeeded();
	emit signalTableEdited();
}

void IoSignalPageWidget::onRemoveClicked()
{
	if (!m_table)
	{
		return;
	}
	const int row = m_tableWidget->currentRow();
	if (row < 0 || row >= static_cast<int>(m_table->entriesMut().size()))
	{
		return;
	}
	const QString name = QString::fromStdString(m_table->entries()[static_cast<size_t>(row)].name);
	m_table->entriesMut().erase(m_table->entriesMut().begin() + row);
	if (m_network)
	{
		m_network->removeWiresTouchingSignal(currentOwnerId(), name);
	}
	if (m_sink)
	{
		m_sink->resetRuntimeFromTable(true);
	}
	rebuildTable();
	flushDeviceIfNeeded();
	emit signalTableEdited();
}

void IoSignalPageWidget::onResetDefaultsClicked()
{
	if (m_sink)
	{
		m_sink->resetRuntimeFromTable(false);
	}
	syncValueColumnsFromSink();
}

void IoSignalPageWidget::onOpenStationClicked()
{
	if (!m_stationDialog)
	{
		m_stationDialog = new QDialog(window());
		m_stationDialog->setAttribute(Qt::WA_DeleteOnClose);
		m_stationDialog->setWindowFlags(m_stationDialog->windowFlags() | Qt::WindowMinMaxButtonsHint);
		auto* layout = new QVBoxLayout(m_stationDialog);
		layout->setContentsMargins(8, 8, 8, 8);
		m_stationWidget = new SignalConnectionStationWidget(m_stationDialog);
		layout->addWidget(m_stationWidget, 1);
		m_stationDialog->resize(960, 640);
		connect(m_stationDialog, &QObject::destroyed, this, [this]() { m_stationWidget = nullptr; });
	}
	m_stationDialog->setWindowTitle(m_useChinese ? QStringLiteral("信号连接站") : QStringLiteral("Signal station"));
	if (m_stationWidget)
	{
		m_stationWidget->setUseChinese(m_useChinese);
		m_stationWidget->setNetwork(m_network);
		m_stationWidget->refreshFromNetwork();
	}
	m_stationDialog->show();
	m_stationDialog->raise();
	m_stationDialog->activateWindow();
}

void IoSignalPageWidget::onSinkValuesChanged()
{
	syncValueColumnsFromSink();
}
