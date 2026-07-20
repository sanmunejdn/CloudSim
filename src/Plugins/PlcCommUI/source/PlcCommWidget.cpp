/// @file PlcCommWidget.cpp
/// @brief PlcCommWidget 实现

#include "PlcCommWidget.h"

#include "PlcCommController.h"
#include "UiIconDecorators.h"

#include <QCheckBox>
#include <QComboBox>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QTableWidget>
#include <QTimer>
#include <QVBoxLayout>
#include <limits>

namespace
{
QString formatHex(const QByteArray& data)
{
	return data.toHex(' ').toUpper();
}

QByteArray parseHex(const QString& text, bool* ok)
{
	QByteArray out;
	QString compact = text;
	compact.remove(QLatin1Char(' '));
	compact.remove(QLatin1Char('\t'));
	if (compact.isEmpty())
	{
		*ok = true;
		return out;
	}
	if (compact.size() % 2 != 0)
	{
		*ok = false;
		return out;
	}
	out.reserve(compact.size() / 2);
	for (int i = 0; i < compact.size(); i += 2)
	{
		bool byteOk = false;
		const int value = compact.mid(i, 2).toInt(&byteOk, 16);
		if (!byteOk || value < 0 || value > 255)
		{
			*ok = false;
			return QByteArray();
		}
		out.append(static_cast<char>(value));
	}
	*ok = true;
	return out;
}

QString formatDecimalBytes(const QByteArray& data)
{
	if (data.isEmpty())
	{
		return QString();
	}
	QStringList parts;
	parts.reserve(data.size());
	for (char byte : data)
	{
		parts.append(QString::number(static_cast<unsigned char>(byte)));
	}
	return parts.join(QLatin1Char(' '));
}

bool parseDecimalBytes(const QString& text, QByteArray* out)
{
	out->clear();
	const QString normalized = text;
	QString token;
	for (const QChar ch : normalized)
	{
		if (ch.isSpace() || ch == QLatin1Char(',') || ch == QLatin1Char(';'))
		{
			if (!token.isEmpty())
			{
				bool ok = false;
				const int value = token.toInt(&ok);
				if (!ok || value < 0 || value > 255)
				{
					return false;
				}
				out->append(static_cast<char>(value));
				token.clear();
			}
		}
		else
		{
			token.append(ch);
		}
	}
	if (!token.isEmpty())
	{
		bool ok = false;
		const int value = token.toInt(&ok);
		if (!ok || value < 0 || value > 255)
		{
			return false;
		}
		out->append(static_cast<char>(value));
	}
	return true;
}

QString formatBinaryBytes(const QByteArray& data)
{
	if (data.isEmpty())
	{
		return QString();
	}
	QStringList parts;
	parts.reserve(data.size());
	for (char byte : data)
	{
		parts.append(QString::number(static_cast<unsigned char>(byte), 2).rightJustified(8, QLatin1Char('0')));
	}
	return parts.join(QLatin1Char(' '));
}

bool parseBinaryBytes(const QString& text, QByteArray* out)
{
	out->clear();
	QString compact = text;
	compact.remove(QLatin1Char(' '));
	compact.remove(QLatin1Char('\t'));
	if (compact.isEmpty())
	{
		return true;
	}
	if (compact.size() % 8 != 0)
	{
		return false;
	}
	out->reserve(compact.size() / 8);
	for (int i = 0; i < compact.size(); i += 8)
	{
		bool ok = false;
		const int value = compact.mid(i, 8).toInt(&ok, 2);
		if (!ok || value < 0 || value > 255)
		{
			return false;
		}
		out->append(static_cast<char>(value));
	}
	return true;
}

QString formatUInt16Le(const QByteArray& data)
{
	if (data.size() < 2)
	{
		return QString();
	}
	QStringList parts;
	for (int i = 0; i + 1 < data.size(); i += 2)
	{
		const auto lo = static_cast<uint8_t>(data[i]);
		const auto hi = static_cast<uint8_t>(data[i + 1]);
		const uint16_t value = static_cast<uint16_t>(lo | (static_cast<uint16_t>(hi) << 8));
		parts.append(QString::number(value));
	}
	return parts.join(QLatin1Char(' '));
}

bool parseUInt16Le(const QString& text, QByteArray* out)
{
	out->clear();
	QString token;
	for (const QChar ch : text)
	{
		if (ch.isSpace() || ch == QLatin1Char(','))
		{
			if (!token.isEmpty())
			{
				bool ok = false;
				const uint value = token.toUInt(&ok);
				if (!ok || value > std::numeric_limits<uint16_t>::max())
				{
					return false;
				}
				const uint16_t word = static_cast<uint16_t>(value);
				out->append(static_cast<char>(word & 0xFF));
				out->append(static_cast<char>((word >> 8) & 0xFF));
				token.clear();
			}
		}
		else
		{
			token.append(ch);
		}
	}
	if (!token.isEmpty())
	{
		bool ok = false;
		const uint value = token.toUInt(&ok);
		if (!ok || value > std::numeric_limits<uint16_t>::max())
		{
			return false;
		}
		const uint16_t word = static_cast<uint16_t>(value);
		out->append(static_cast<char>(word & 0xFF));
		out->append(static_cast<char>((word >> 8) & 0xFF));
	}
	return true;
}

QString formatInt32Le(const QByteArray& data)
{
	if (data.size() < 4)
	{
		return QString();
	}
	QStringList parts;
	for (int i = 0; i + 3 < data.size(); i += 4)
	{
		const uint32_t value = static_cast<uint32_t>(static_cast<uint8_t>(data[i])) |
							   (static_cast<uint32_t>(static_cast<uint8_t>(data[i + 1])) << 8) |
							   (static_cast<uint32_t>(static_cast<uint8_t>(data[i + 2])) << 16) |
							   (static_cast<uint32_t>(static_cast<uint8_t>(data[i + 3])) << 24);
		parts.append(QString::number(static_cast<int32_t>(value)));
	}
	return parts.join(QLatin1Char(' '));
}

bool parseInt32Le(const QString& text, QByteArray* out)
{
	out->clear();
	QString token;
	for (const QChar ch : text)
	{
		if (ch.isSpace() || ch == QLatin1Char(','))
		{
			if (!token.isEmpty())
			{
				bool ok = false;
				const int value = token.toInt(&ok);
				if (!ok)
				{
					return false;
				}
				const int32_t word = static_cast<int32_t>(value);
				out->append(static_cast<char>(word & 0xFF));
				out->append(static_cast<char>((word >> 8) & 0xFF));
				out->append(static_cast<char>((word >> 16) & 0xFF));
				out->append(static_cast<char>((word >> 24) & 0xFF));
				token.clear();
			}
		}
		else
		{
			token.append(ch);
		}
	}
	if (!token.isEmpty())
	{
		bool ok = false;
		const int value = token.toInt(&ok);
		if (!ok)
		{
			return false;
		}
		const int32_t word = static_cast<int32_t>(value);
		out->append(static_cast<char>(word & 0xFF));
		out->append(static_cast<char>((word >> 8) & 0xFF));
		out->append(static_cast<char>((word >> 16) & 0xFF));
		out->append(static_cast<char>((word >> 24) & 0xFF));
	}
	return true;
}

} // namespace

PlcCommWidget::PlcCommWidget(QWidget* parent)
	: QWidget(parent), controller_(new PlcCommController(this)), pollTimer_(new QTimer(this))
{
	auto* root = new QVBoxLayout(this);

	connGroup_ = new QGroupBox(this);
	auto* connLayout = new QGridLayout(connGroup_);

	protocolCombo_ = new QComboBox(connGroup_);
	protocolCombo_->addItem(QString(), static_cast<int>(PlcProtocol::AbEip));
	protocolCombo_->addItem(QString(), static_cast<int>(PlcProtocol::ModbusTcp));

	gatewayEdit_ = new QLineEdit(QString(), connGroup_);
	gatewayEdit_->setPlaceholderText(QStringLiteral("192.168.0.10"));
	portSpin_ = new QSpinBox(connGroup_);
	portSpin_->setRange(1, 65535);
	portSpin_->setValue(502);
	pathEdit_ = new QLineEdit(QStringLiteral("1"), connGroup_);
	cpuEdit_ = new QLineEdit(QStringLiteral("lgx"), connGroup_);
	connectBtn_ = new QPushButton(connGroup_);
	disconnectBtn_ = new QPushButton(connGroup_);
	disconnectBtn_->setEnabled(false);

	protocolLabel_ = new QLabel(connGroup_);
	ipLabel_ = new QLabel(connGroup_);
	portLabel_ = new QLabel(connGroup_);
	pathLabel_ = new QLabel(connGroup_);
	cpuLabel_ = new QLabel(connGroup_);
	timeoutLabel_ = new QLabel(connGroup_);
	timeoutSpin_ = new QSpinBox(connGroup_);
	timeoutSpin_->setRange(1000, 120000);
	timeoutSpin_->setSingleStep(1000);
	timeoutSpin_->setValue(10000);
	timeoutSpin_->setSuffix(QStringLiteral(" ms"));

	connLayout->addWidget(protocolLabel_, 0, 0);
	connLayout->addWidget(protocolCombo_, 0, 1);
	connLayout->addWidget(ipLabel_, 0, 2);
	connLayout->addWidget(gatewayEdit_, 0, 3);
	connLayout->addWidget(portLabel_, 1, 0);
	connLayout->addWidget(portSpin_, 1, 1);
	connLayout->addWidget(pathLabel_, 1, 2);
	connLayout->addWidget(pathEdit_, 1, 3);
	connLayout->addWidget(cpuLabel_, 2, 0);
	connLayout->addWidget(cpuEdit_, 2, 1);
	connLayout->addWidget(connectBtn_, 2, 2);
	connLayout->addWidget(disconnectBtn_, 2, 3);
	connLayout->addWidget(timeoutLabel_, 3, 0);
	connLayout->addWidget(timeoutSpin_, 3, 1, 1, 3);

	tagGroup_ = new QGroupBox(this);
	auto* tagLayout = new QVBoxLayout(tagGroup_);
	auto* tagRow = new QHBoxLayout();
	tagNameEdit_ = new QLineEdit(tagGroup_);
	addTagBtn_ = new QPushButton(tagGroup_);
	removeTagBtn_ = new QPushButton(tagGroup_);
	tagRow->addWidget(tagNameEdit_);
	tagRow->addWidget(addTagBtn_);
	tagRow->addWidget(removeTagBtn_);

	tagTable_ = new QTableWidget(0, 3, tagGroup_);
	tagTable_->horizontalHeader()->setStretchLastSection(true);
	tagTable_->setSelectionBehavior(QAbstractItemView::SelectRows);
	tagTable_->setSelectionMode(QAbstractItemView::SingleSelection);

	formatLabel_ = new QLabel(tagGroup_);
	formatCombo_ = new QComboBox(tagGroup_);
	formatCombo_->addItem(QString(), static_cast<int>(ValueFormat::Hex));
	formatCombo_->addItem(QString(), static_cast<int>(ValueFormat::DecimalBytes));
	formatCombo_->addItem(QString(), static_cast<int>(ValueFormat::BinaryBytes));
	formatCombo_->addItem(QString(), static_cast<int>(ValueFormat::UInt16Le));
	formatCombo_->addItem(QString(), static_cast<int>(ValueFormat::Int32Le));

	valueEdit_ = new QLineEdit(tagGroup_);
	readBtn_ = new QPushButton(tagGroup_);
	writeBtn_ = new QPushButton(tagGroup_);
	pollCheck_ = new QCheckBox(tagGroup_);
	pollIntervalSpin_ = new QSpinBox(tagGroup_);
	pollIntervalSpin_->setRange(100, 60000);
	pollIntervalSpin_->setValue(1000);

	auto* formatRow = new QHBoxLayout();
	formatRow->addWidget(formatLabel_);
	formatRow->addWidget(formatCombo_, 1);

	auto* ioRow = new QHBoxLayout();
	ioRow->addWidget(valueEdit_, 1);
	ioRow->addWidget(readBtn_);
	ioRow->addWidget(writeBtn_);
	ioRow->addWidget(pollCheck_);
	ioRow->addWidget(pollIntervalSpin_);

	tagLayout->addLayout(tagRow);
	tagLayout->addWidget(tagTable_);
	tagLayout->addLayout(formatRow);
	tagLayout->addLayout(ioRow);

	logEdit_ = new QPlainTextEdit(this);
	logEdit_->setReadOnly(true);
	logEdit_->setMaximumBlockCount(500);
	logCaption_ = new QLabel(this);

	root->addWidget(connGroup_);
	root->addWidget(tagGroup_, 1);
	root->addWidget(logCaption_);
	root->addWidget(logEdit_, 1);

	connect(connectBtn_, &QPushButton::clicked, this, &PlcCommWidget::onConnectClicked);
	connect(disconnectBtn_, &QPushButton::clicked, this, &PlcCommWidget::onDisconnectClicked);
	connect(addTagBtn_, &QPushButton::clicked, this, &PlcCommWidget::onAddTagClicked);
	connect(removeTagBtn_, &QPushButton::clicked, this, &PlcCommWidget::onRemoveTagClicked);
	connect(readBtn_, &QPushButton::clicked, this, &PlcCommWidget::onReadClicked);
	connect(writeBtn_, &QPushButton::clicked, this, &PlcCommWidget::onWriteClicked);

	UiIconDecorators::apply(connectBtn_, UiIconId::Connect);
	UiIconDecorators::apply(disconnectBtn_, UiIconId::Disconnect);
	UiIconDecorators::apply(addTagBtn_, UiIconId::Add);
	UiIconDecorators::apply(removeTagBtn_, UiIconId::Delete);
	UiIconDecorators::apply(readBtn_, UiIconId::Read);
	UiIconDecorators::apply(writeBtn_, UiIconId::Write);

	connect(pollTimer_, &QTimer::timeout, this, &PlcCommWidget::onPollTimeout);
	connect(pollCheck_, &QCheckBox::toggled, this,
			[this](bool on)
			{
				if (on)
				{
					pollTimer_->start(pollIntervalSpin_->value());
				}
				else
				{
					pollTimer_->stop();
				}
			});
	connect(pollIntervalSpin_, QOverload<int>::of(&QSpinBox::valueChanged), this,
			[this](int ms)
			{
				if (pollTimer_->isActive())
				{
					pollTimer_->start(ms);
				}
			});
	connect(protocolCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
			[this](int)
			{
				pollTimer_->stop();
				pollCheck_->setChecked(false);
				controller_->disconnectPlc();
				updateProtocolFields();
			});

	connect(controller_, &PlcCommController::connectedChanged, this, &PlcCommWidget::onConnectedChanged);
	connect(controller_, &PlcCommController::tagAdded, this, &PlcCommWidget::onTagAdded);
	connect(controller_, &PlcCommController::tagRead, this, &PlcCommWidget::onTagRead);
	connect(controller_, &PlcCommController::logMessage, this, &PlcCommWidget::onLogMessage);
	connect(controller_, &PlcCommController::errorOccurred, this, &PlcCommWidget::onError);
	connect(formatCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
			&PlcCommWidget::onDisplayFormatChanged);

	controller_->startWorker();
	protocolCombo_->setCurrentIndex(0);
	applyLanguage();
}

PlcCommWidget::~PlcCommWidget()
{
	pollTimer_->stop();
	controller_->disconnectPlc();
	controller_->stopWorker();
}

void PlcCommWidget::setUseChinese(bool chinese)
{
	useChinese_ = chinese;
	syncWorkerLanguage();
}

void PlcCommWidget::applyLanguage()
{
	connGroup_->setTitle(i18n(QStringLiteral("Connection"), QStringLiteral("连接")));
	tagGroup_->setTitle(i18n(QStringLiteral("Tags"), QStringLiteral("标签")));
	logCaption_->setText(i18n(QStringLiteral("Log"), QStringLiteral("日志")));

	protocolLabel_->setText(i18n(QStringLiteral("Protocol"), QStringLiteral("协议")));
	ipLabel_->setText(i18n(QStringLiteral("IP"), QStringLiteral("IP")));
	portLabel_->setText(i18n(QStringLiteral("Port"), QStringLiteral("端口")));
	cpuLabel_->setText(i18n(QStringLiteral("CPU"), QStringLiteral("CPU")));
	timeoutLabel_->setText(i18n(QStringLiteral("Timeout"), QStringLiteral("超时")));
	timeoutSpin_->setSuffix(i18n(QStringLiteral(" ms"), QStringLiteral(" ms")));
	updateProtocolFields();

	const int protoIdx = protocolCombo_->currentIndex();
	protocolCombo_->setItemText(0, i18n(QStringLiteral("AB EIP"), QStringLiteral("AB EIP")));
	protocolCombo_->setItemText(1, i18n(QStringLiteral("Modbus TCP"), QStringLiteral("Modbus TCP")));
	protocolCombo_->setCurrentIndex(protoIdx);

	connectBtn_->setText(i18n(QStringLiteral("Connect"), QStringLiteral("连接")));
	disconnectBtn_->setText(i18n(QStringLiteral("Disconnect"), QStringLiteral("断开")));

	addTagBtn_->setText(i18n(QStringLiteral("Add"), QStringLiteral("添加")));
	removeTagBtn_->setText(i18n(QStringLiteral("Remove"), QStringLiteral("删除")));

	tagTable_->setHorizontalHeaderLabels({
		i18n(QStringLiteral("Handle"), QStringLiteral("句柄")),
		i18n(QStringLiteral("Name"), QStringLiteral("名称")),
		i18n(QStringLiteral("Last data"), QStringLiteral("最近数据")),
	});

	formatLabel_->setText(i18n(QStringLiteral("Display"), QStringLiteral("显示")));
	const int fmtIdx = formatCombo_->currentIndex();
	formatCombo_->setItemText(0, i18n(QStringLiteral("Hex"), QStringLiteral("十六进制")));
	formatCombo_->setItemText(1, i18n(QStringLiteral("Decimal (bytes)"), QStringLiteral("十进制(字节)")));
	formatCombo_->setItemText(2, i18n(QStringLiteral("Binary (bytes)"), QStringLiteral("二进制(字节)")));
	formatCombo_->setItemText(3, i18n(QStringLiteral("UInt16 LE"), QStringLiteral("UInt16 小端")));
	formatCombo_->setItemText(4, i18n(QStringLiteral("Int32 LE"), QStringLiteral("Int32 小端")));
	formatCombo_->setCurrentIndex(fmtIdx);
	updateValueEditPlaceholder();
	refreshAllDisplayedValues();
	readBtn_->setText(i18n(QStringLiteral("Read"), QStringLiteral("读")));
	writeBtn_->setText(i18n(QStringLiteral("Write"), QStringLiteral("写")));
	pollCheck_->setText(i18n(QStringLiteral("Poll"), QStringLiteral("轮询")));
	pollIntervalSpin_->setSuffix(i18n(QStringLiteral(" ms"), QStringLiteral(" ms")));

	syncWorkerLanguage();
}

QString PlcCommWidget::i18n(const QString& en, const QString& zh) const
{
	return useChinese_ ? zh : en;
}

void PlcCommWidget::syncWorkerLanguage()
{
	controller_->setUseChinese(useChinese_);
}

void PlcCommWidget::updateProtocolFields()
{
	const bool modbus = protocolCombo_->currentData().toInt() == static_cast<int>(PlcProtocol::ModbusTcp);
	cpuEdit_->setEnabled(!modbus);
	portSpin_->setEnabled(modbus);
	pathEdit_->setEnabled(true);

	if (modbus)
	{
		pathLabel_->setText(i18n(QStringLiteral("Unit ID"), QStringLiteral("单元 ID")));
		pathEdit_->setPlaceholderText(i18n(QStringLiteral("1"), QStringLiteral("1")));
		if (pathEdit_->text().contains(QLatin1Char(',')))
		{
			pathEdit_->setText(QStringLiteral("1"));
		}
	}
	else
	{
		pathLabel_->setText(i18n(QStringLiteral("Path"), QStringLiteral("Path")));
		pathEdit_->setPlaceholderText(QStringLiteral("1,0"));
		if (pathEdit_->text() == QStringLiteral("1"))
		{
			pathEdit_->setText(QStringLiteral("1,0"));
		}
	}

	tagNameEdit_->setPlaceholderText(
		modbus ? i18n(QStringLiteral("40001 or hr0 (holding reg)"), QStringLiteral("40001 或 hr0（保持寄存器）"))
			   : i18n(QStringLiteral("Tag name"), QStringLiteral("标签名")));
}

PlcConnectionConfig PlcCommWidget::currentConfig() const
{
	PlcConnectionConfig cfg;
	cfg.protocol = static_cast<PlcProtocol>(protocolCombo_->currentData().toInt());
	cfg.gateway = gatewayEdit_->text().trimmed().toStdString();
	cfg.port = static_cast<uint16_t>(portSpin_->value());
	cfg.path = pathEdit_->text().trimmed().toStdString();
	cfg.cpu = cpuEdit_->text().trimmed().toStdString();
	cfg.timeoutMs = timeoutSpin_->value();
	return cfg;
}

int PlcCommWidget::selectedHandle() const
{
	const int row = tagTable_->currentRow();
	if (row < 0)
	{
		return -1;
	}
	return tagTable_->item(row, 0)->text().toInt();
}

void PlcCommWidget::appendLog(const QString& line)
{
	logEdit_->appendPlainText(line);
}

void PlcCommWidget::onConnectClicked()
{
	if (gatewayEdit_->text().trimmed().isEmpty())
	{
		QMessageBox::warning(this, i18n(QStringLiteral("PLC"), QStringLiteral("PLC")),
							 i18n(QStringLiteral("Enter PLC IP address"), QStringLiteral("请输入 PLC IP")));
		return;
	}
	controller_->connectPlc(currentConfig());
}

void PlcCommWidget::onDisconnectClicked()
{
	pollTimer_->stop();
	pollCheck_->setChecked(false);
	controller_->disconnectPlc();
}

void PlcCommWidget::onAddTagClicked()
{
	PlcTagSpec spec;
	spec.name = tagNameEdit_->text().trimmed().toStdString();
	if (spec.name.empty())
	{
		QMessageBox::warning(this, i18n(QStringLiteral("PLC"), QStringLiteral("PLC")),
							 i18n(QStringLiteral("Enter a tag name"), QStringLiteral("请输入标签名")));
		return;
	}
	controller_->addTag(spec);
}

void PlcCommWidget::onRemoveTagClicked()
{
	const int handle = selectedHandle();
	if (handle < 0)
	{
		return;
	}
	controller_->removeTag(handle);
	rawByHandle_.remove(handle);
	for (int row = 0; row < tagTable_->rowCount(); ++row)
	{
		if (tagTable_->item(row, 0)->text().toInt() == handle)
		{
			tagTable_->removeRow(row);
			break;
		}
	}
}

void PlcCommWidget::onReadClicked()
{
	const int handle = selectedHandle();
	if (handle < 0)
	{
		QMessageBox::information(this, i18n(QStringLiteral("PLC"), QStringLiteral("PLC")),
								 i18n(QStringLiteral("Select a tag row"), QStringLiteral("请选择标签行")));
		return;
	}
	controller_->readTag(handle);
}

void PlcCommWidget::onWriteClicked()
{
	const int handle = selectedHandle();
	if (handle < 0)
	{
		QMessageBox::information(this, i18n(QStringLiteral("PLC"), QStringLiteral("PLC")),
								 i18n(QStringLiteral("Select a tag row"), QStringLiteral("请选择标签行")));
		return;
	}
	QByteArray bytes;
	if (!parseValueText(valueEdit_->text(), &bytes))
	{
		QMessageBox::warning(
			this, i18n(QStringLiteral("PLC"), QStringLiteral("PLC")),
			i18n(QStringLiteral("Invalid value for selected format"), QStringLiteral("当前格式下的数值无效")));
		return;
	}
	controller_->writeTag(handle, bytes);
}

void PlcCommWidget::onPollTimeout()
{
	QList<int> handles;
	handles.reserve(tagTable_->rowCount());
	for (int row = 0; row < tagTable_->rowCount(); ++row)
	{
		handles.append(tagTable_->item(row, 0)->text().toInt());
	}
	if (!handles.isEmpty())
	{
		controller_->pollTags(handles);
	}
}

void PlcCommWidget::onConnectedChanged(bool connected)
{
	connectBtn_->setEnabled(!connected);
	disconnectBtn_->setEnabled(connected);
	appendLog(connected ? i18n(QStringLiteral("Status: connected"), QStringLiteral("连接状态：已连接"))
						: i18n(QStringLiteral("Status: disconnected"), QStringLiteral("连接状态：未连接")));
}

void PlcCommWidget::onTagAdded(int handle, const QString& name)
{
	const int row = tagTable_->rowCount();
	tagTable_->insertRow(row);
	tagTable_->setItem(row, 0, new QTableWidgetItem(QString::number(handle)));
	tagTable_->setItem(row, 1, new QTableWidgetItem(name));
	tagTable_->setItem(row, 2, new QTableWidgetItem());
	tagTable_->selectRow(row);
}

void PlcCommWidget::onTagRead(int handle, const QByteArray& data, bool ok)
{
	if (!ok)
	{
		return;
	}
	rawByHandle_.insert(handle, data);
	refreshDisplayedValue(handle);
}

void PlcCommWidget::onDisplayFormatChanged(int /*index*/)
{
	updateValueEditPlaceholder();
	refreshAllDisplayedValues();
}

PlcCommWidget::ValueFormat PlcCommWidget::currentValueFormat() const
{
	return static_cast<ValueFormat>(formatCombo_->currentData().toInt());
}

QString PlcCommWidget::formatRawBytes(const QByteArray& data) const
{
	switch (currentValueFormat())
	{
	case ValueFormat::Hex:
		return formatHex(data);
	case ValueFormat::DecimalBytes:
		return formatDecimalBytes(data);
	case ValueFormat::BinaryBytes:
		return formatBinaryBytes(data);
	case ValueFormat::UInt16Le:
		return formatUInt16Le(data);
	case ValueFormat::Int32Le:
		return formatInt32Le(data);
	}
	return formatHex(data);
}

bool PlcCommWidget::parseValueText(const QString& text, QByteArray* out) const
{
	switch (currentValueFormat())
	{
	case ValueFormat::Hex:
	{
		bool ok = false;
		*out = parseHex(text, &ok);
		return ok;
	}
	case ValueFormat::DecimalBytes:
		return parseDecimalBytes(text, out);
	case ValueFormat::BinaryBytes:
		return parseBinaryBytes(text, out);
	case ValueFormat::UInt16Le:
		return parseUInt16Le(text, out);
	case ValueFormat::Int32Le:
		return parseInt32Le(text, out);
	}
	return false;
}

void PlcCommWidget::refreshDisplayedValue(int handle)
{
	const QByteArray data = rawByHandle_.value(handle);
	const QString formatted = formatRawBytes(data);

	if (selectedHandle() == handle)
	{
		valueEdit_->setText(formatted);
	}
	for (int row = 0; row < tagTable_->rowCount(); ++row)
	{
		if (tagTable_->item(row, 0)->text().toInt() == handle)
		{
			tagTable_->item(row, 2)->setText(formatted);
			break;
		}
	}
}

void PlcCommWidget::refreshAllDisplayedValues()
{
	for (auto it = rawByHandle_.constBegin(); it != rawByHandle_.constEnd(); ++it)
	{
		refreshDisplayedValue(it.key());
	}
}

void PlcCommWidget::updateValueEditPlaceholder()
{
	switch (currentValueFormat())
	{
	case ValueFormat::Hex:
		valueEdit_->setPlaceholderText(i18n(QStringLiteral("e.g. 01 02 FF"), QStringLiteral("如 01 02 FF")));
		break;
	case ValueFormat::DecimalBytes:
		valueEdit_->setPlaceholderText(i18n(QStringLiteral("e.g. 1 255 0"), QStringLiteral("如 1 255 0")));
		break;
	case ValueFormat::BinaryBytes:
		valueEdit_->setPlaceholderText(
			i18n(QStringLiteral("e.g. 00000001 11111111"), QStringLiteral("如 00000001 11111111")));
		break;
	case ValueFormat::UInt16Le:
		valueEdit_->setPlaceholderText(i18n(QStringLiteral("e.g. 1234"), QStringLiteral("如 1234")));
		break;
	case ValueFormat::Int32Le:
		valueEdit_->setPlaceholderText(i18n(QStringLiteral("e.g. -1 100000"), QStringLiteral("如 -1 100000")));
		break;
	}
}

void PlcCommWidget::onLogMessage(const QString& text)
{
	appendLog(text);
}

void PlcCommWidget::onError(const QString& text)
{
	appendLog(i18n(QStringLiteral("Error: %1").arg(text), QStringLiteral("错误: %1").arg(text)));
}

QWidget* createPlcCommWidget(QWidget* parent)
{
	return new PlcCommWidget(parent);
}
