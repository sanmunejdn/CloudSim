/// @file RobotAxisJogControllerDialog.cpp
/// @brief TCP / 轴点动控制器（速率点动）

#include "RobotAxisJogControllerDialog.h"

#include "RobotSimulationTypes.h"

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSizePolicy>
#include <QStyle>
#include <QTabWidget>
#include <QTimer>
#include <QVBoxLayout>
#include <cmath>

namespace
{
constexpr double kPi = 3.14159265358979323846;
} // namespace

const char* RobotAxisJogControllerDialog::toneKey(AxisTone tone)
{
	switch (tone)
	{
	case AxisTone::X:
		return "x";
	case AxisTone::Y:
		return "y";
	case AxisTone::Z:
		return "z";
	case AxisTone::Neutral:
	default:
		return "neutral";
	}
}

RobotAxisJogControllerDialog::RobotAxisJogControllerDialog(QWidget* parent)
	: QDialog(parent, Qt::Tool | Qt::WindowStaysOnTopHint)
{
	setObjectName(QStringLiteral("RobotAxisJogControllerDialog"));
	setAttribute(Qt::WA_DeleteOnClose, false);
	setMinimumWidth(360);
	applyDialogChrome();

	m_jogHoldRepeatTimer = new QTimer(this);
	m_jogHoldRepeatTimer->setTimerType(Qt::PreciseTimer);
	m_jogHoldRepeatTimer->setInterval(RobotSimulation::kPlaybackTimerIntervalMs);
	connect(m_jogHoldRepeatTimer, &QTimer::timeout, this,
			[this]()
			{
				if (m_jogHoldEmit)
				{
					m_jogHoldEmit(jogTickDtSec());
				}
			});

	auto* root = new QVBoxLayout(this);
	root->setContentsMargins(10, 10, 10, 10);
	root->setSpacing(8);

	m_tabs = new QTabWidget(this);
	m_tabs->setDocumentMode(true);
	root->addWidget(m_tabs);

	m_tcpPage = new QWidget(m_tabs);
	auto* tcpLayout = new QVBoxLayout(m_tcpPage);
	tcpLayout->setContentsMargins(8, 10, 8, 8);
	tcpLayout->setSpacing(10);

	m_settingsCard = new QFrame(m_tcpPage);
	m_settingsCard->setObjectName(QStringLiteral("jogCard"));
	auto* settingsLayout = new QVBoxLayout(m_settingsCard);
	settingsLayout->setContentsMargins(10, 10, 10, 10);
	settingsLayout->setSpacing(8);

	auto* frameRow = new QHBoxLayout();
	frameRow->setSpacing(8);
	m_frameLabel = new QLabel(m_settingsCard);
	m_frameLabel->setObjectName(QStringLiteral("jogFieldLabel"));
	m_frameCombo = new QComboBox(m_settingsCard);
	m_frameCombo->addItem(QString(), static_cast<int>(RobotAxisJogTcpFrame::Tool));
	m_frameCombo->addItem(QString(), static_cast<int>(RobotAxisJogTcpFrame::Base));
	m_frameCombo->addItem(QString(), static_cast<int>(RobotAxisJogTcpFrame::World));
	m_frameCombo->addItem(QString(), static_cast<int>(RobotAxisJogTcpFrame::User));
	m_frameCombo->setCurrentIndex(0);
	frameRow->addWidget(m_frameLabel);
	frameRow->addWidget(m_frameCombo, 1);
	settingsLayout->addLayout(frameRow);

	auto* rateRow = new QHBoxLayout();
	rateRow->setSpacing(8);
	m_transRateLabel = new QLabel(m_settingsCard);
	m_transRateLabel->setObjectName(QStringLiteral("jogFieldLabel"));
	m_transRateCombo = new QComboBox(m_settingsCard);
	// 示教常用线速度档（mm/s）
	for (const double v : {5.0, 10.0, 25.0, 50.0, 100.0, 200.0})
	{
		m_transRateCombo->addItem(QString::number(static_cast<int>(v)), v);
	}
	m_transRateCombo->setCurrentIndex(2); // 25 mm/s
	m_rotRateLabel = new QLabel(m_settingsCard);
	m_rotRateLabel->setObjectName(QStringLiteral("jogFieldLabel"));
	m_rotRateSpin = new QDoubleSpinBox(m_settingsCard);
	m_rotRateSpin->setRange(1.0, 180.0);
	m_rotRateSpin->setDecimals(1);
	m_rotRateSpin->setValue(15.0);
	m_rotRateSpin->setSuffix(QStringLiteral(" °/s"));
	rateRow->addWidget(m_transRateLabel);
	rateRow->addWidget(m_transRateCombo, 1);
	rateRow->addWidget(m_rotRateLabel);
	rateRow->addWidget(m_rotRateSpin, 1);
	settingsLayout->addLayout(rateRow);
	tcpLayout->addWidget(m_settingsCard);

	QFrame* transCard = makeSectionCard(m_tcpPage, &m_transSectionTitle);
	auto* transBody = qobject_cast<QVBoxLayout*>(transCard->layout());
	struct TcpAxisDef
	{
		const char* label;
		AxisTone tone;
		double dx, dy, dz, dex, dey, dez;
	};
	const TcpAxisDef transAxes[] = {
		{"X", AxisTone::X, 1, 0, 0, 0, 0, 0},
		{"Y", AxisTone::Y, 0, 1, 0, 0, 0, 0},
		{"Z", AxisTone::Z, 0, 0, 1, 0, 0, 0},
	};
	for (const TcpAxisDef& d : transAxes)
	{
		QPushButton* minus = nullptr;
		QPushButton* plus = nullptr;
		QWidget* row = makeAxisJogRow(transCard, QString::fromLatin1(d.label), d.tone, &minus, &plus);
		transBody->addWidget(row);
		m_tcpButtons.push_back(minus);
		m_tcpButtons.push_back(plus);
		bindJogHold(minus,
					[this, d](double dtSec)
					{
						const double v = m_transRateCombo->currentData().toDouble();
						const double t = v * dtSec;
						emitTcpDelta(-d.dx * t, -d.dy * t, -d.dz * t, 0, 0, 0);
					});
		bindJogHold(plus,
					[this, d](double dtSec)
					{
						const double v = m_transRateCombo->currentData().toDouble();
						const double t = v * dtSec;
						emitTcpDelta(d.dx * t, d.dy * t, d.dz * t, 0, 0, 0);
					});
	}
	tcpLayout->addWidget(transCard);

	QFrame* rotCard = makeSectionCard(m_tcpPage, &m_rotSectionTitle);
	auto* rotBody = qobject_cast<QVBoxLayout*>(rotCard->layout());
	const TcpAxisDef rotAxes[] = {
		{"Rx", AxisTone::X, 0, 0, 0, 1, 0, 0},
		{"Ry", AxisTone::Y, 0, 0, 0, 0, 1, 0},
		{"Rz", AxisTone::Z, 0, 0, 0, 0, 0, 1},
	};
	for (const TcpAxisDef& d : rotAxes)
	{
		QPushButton* minus = nullptr;
		QPushButton* plus = nullptr;
		QWidget* row = makeAxisJogRow(rotCard, QString::fromLatin1(d.label), d.tone, &minus, &plus);
		rotBody->addWidget(row);
		m_tcpButtons.push_back(minus);
		m_tcpButtons.push_back(plus);
		bindJogHold(minus,
					[this, d](double dtSec)
					{
						const double w = m_rotRateSpin->value();
						const double r = w * dtSec;
						emitTcpDelta(0, 0, 0, -d.dex * r, -d.dey * r, -d.dez * r);
					});
		bindJogHold(plus,
					[this, d](double dtSec)
					{
						const double w = m_rotRateSpin->value();
						const double r = w * dtSec;
						emitTcpDelta(0, 0, 0, d.dex * r, d.dey * r, d.dez * r);
					});
	}
	tcpLayout->addWidget(rotCard);
	tcpLayout->addStretch(1);
	m_tabs->addTab(m_tcpPage, QStringLiteral("TCP"));

	m_axisPage = new QWidget(m_tabs);
	m_axisLayout = new QVBoxLayout(m_axisPage);
	m_axisLayout->setContentsMargins(8, 10, 8, 8);
	m_axisLayout->setSpacing(8);

	auto* jointRateCard = new QFrame(m_axisPage);
	jointRateCard->setObjectName(QStringLiteral("jogCard"));
	auto* jointRateLayout = new QHBoxLayout(jointRateCard);
	jointRateLayout->setContentsMargins(10, 10, 10, 10);
	jointRateLayout->setSpacing(8);
	m_jointRateLabel = new QLabel(jointRateCard);
	m_jointRateLabel->setObjectName(QStringLiteral("jogFieldLabel"));
	m_jointRateSpin = new QDoubleSpinBox(jointRateCard);
	m_jointRateSpin->setRange(1.0, 90.0);
	m_jointRateSpin->setDecimals(1);
	m_jointRateSpin->setValue(15.0);
	m_jointRateSpin->setSuffix(QStringLiteral(" °/s"));
	jointRateLayout->addWidget(m_jointRateLabel);
	jointRateLayout->addWidget(m_jointRateSpin, 1);
	m_axisLayout->addWidget(jointRateCard);
	m_axisLayout->addStretch(1);
	m_tabs->addTab(m_axisPage, QStringLiteral("Axis"));

	retranslate();
}

RobotAxisJogControllerDialog::~RobotAxisJogControllerDialog() = default;

double RobotAxisJogControllerDialog::jogTickDtSec() const
{
	const int ms = m_jogHoldRepeatTimer ? m_jogHoldRepeatTimer->interval() : RobotSimulation::kPlaybackTimerIntervalMs;
	return static_cast<double>(ms) / 1000.0;
}

void RobotAxisJogControllerDialog::applyDialogChrome()
{
	// 跟全局 Palette，避免写死色与主题漂移；轴色仅作示教软提示
	setStyleSheet(QStringLiteral(
		"#RobotAxisJogControllerDialog {"
		"  background-color: palette(window);"
		"}"
		"#RobotAxisJogControllerDialog QFrame#jogCard {"
		"  background-color: palette(base);"
		"  border: 1px solid palette(mid);"
		"  border-radius: 8px;"
		"}"
		"#RobotAxisJogControllerDialog QLabel#jogSectionTitle {"
		"  color: palette(window-text);"
		"  font-weight: 600;"
		"  font-size: 12px;"
		"  padding-bottom: 2px;"
		"}"
		"#RobotAxisJogControllerDialog QLabel#jogFieldLabel {"
		"  color: palette(window-text);"
		"  font-size: 12px;"
		"}"
		"#RobotAxisJogControllerDialog QLabel#jogAxisChip[axisTone=\"x\"] {"
		"  color: #c0392b;"
		"  font-weight: 700;"
		"  min-width: 36px;"
		"}"
		"#RobotAxisJogControllerDialog QLabel#jogAxisChip[axisTone=\"y\"] {"
		"  color: #1e8449;"
		"  font-weight: 700;"
		"  min-width: 36px;"
		"}"
		"#RobotAxisJogControllerDialog QLabel#jogAxisChip[axisTone=\"z\"] {"
		"  color: #2471a3;"
		"  font-weight: 700;"
		"  min-width: 36px;"
		"}"
		"#RobotAxisJogControllerDialog QLabel#jogAxisChip[axisTone=\"neutral\"] {"
		"  color: palette(window-text);"
		"  font-weight: 600;"
		"  min-width: 36px;"
		"}"
		"#RobotAxisJogControllerDialog QPushButton[jogTone=\"x\"] {"
		"  border: 1px solid rgba(192, 57, 43, 90);"
		"}"
		"#RobotAxisJogControllerDialog QPushButton[jogTone=\"y\"] {"
		"  border: 1px solid rgba(30, 132, 73, 90);"
		"}"
		"#RobotAxisJogControllerDialog QPushButton[jogTone=\"z\"] {"
		"  border: 1px solid rgba(36, 113, 163, 90);"
		"}"
		"#RobotAxisJogControllerDialog QPushButton[btnRole=\"secondary\"] {"
		"  min-height: 34px;"
		"  font-weight: 600;"
		"}"
		"#RobotAxisJogControllerDialog QWidget#jogAxisRow {"
		"  background: transparent;"
		"}"));
}

QFrame* RobotAxisJogControllerDialog::makeSectionCard(QWidget* parent, QLabel** outTitle)
{
	auto* card = new QFrame(parent);
	card->setObjectName(QStringLiteral("jogCard"));
	auto* layout = new QVBoxLayout(card);
	layout->setContentsMargins(10, 10, 10, 10);
	layout->setSpacing(6);
	auto* title = new QLabel(card);
	title->setObjectName(QStringLiteral("jogSectionTitle"));
	layout->addWidget(title);
	if (outTitle)
	{
		*outTitle = title;
	}
	return card;
}

QWidget* RobotAxisJogControllerDialog::makeAxisJogRow(QWidget* parent, const QString& axisLabel, AxisTone tone,
													  QPushButton** outMinus, QPushButton** outPlus)
{
	auto* row = new QWidget(parent);
	row->setObjectName(QStringLiteral("jogAxisRow"));
	auto* hl = new QHBoxLayout(row);
	hl->setContentsMargins(0, 0, 0, 0);
	hl->setSpacing(8);

	QPushButton* minus = makeJogButton(QStringLiteral("−"), row, tone);
	auto* chip = new QLabel(axisLabel, row);
	chip->setObjectName(QStringLiteral("jogAxisChip"));
	chip->setProperty("axisTone", QLatin1String(toneKey(tone)));
	chip->setMinimumWidth(40);
	chip->setAlignment(Qt::AlignCenter);
	if (QStyle* st = chip->style())
	{
		st->unpolish(chip);
		st->polish(chip);
	}
	QPushButton* plus = makeJogButton(QStringLiteral("+"), row, tone);

	hl->addWidget(minus, 1);
	hl->addWidget(chip);
	hl->addWidget(plus, 1);

	if (outMinus)
	{
		*outMinus = minus;
	}
	if (outPlus)
	{
		*outPlus = plus;
	}
	return row;
}

QPushButton* RobotAxisJogControllerDialog::makeJogButton(const QString& text, QWidget* parent, AxisTone tone)
{
	auto* btn = new QPushButton(text, parent);
	btn->setProperty("btnRole", QLatin1String("secondary"));
	btn->setProperty("jogTone", QLatin1String(toneKey(tone)));
	btn->setMinimumHeight(36);
	btn->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
	btn->setAutoRepeat(false);
	if (QStyle* st = btn->style())
	{
		st->unpolish(btn);
		st->polish(btn);
	}
	return btn;
}

void RobotAxisJogControllerDialog::stopJogHold()
{
	if (m_jogHoldRepeatTimer)
	{
		m_jogHoldRepeatTimer->stop();
	}
	m_jogHoldEmit = nullptr;
}

void RobotAxisJogControllerDialog::bindJogHold(QPushButton* btn, std::function<void(double dtSec)> emitDt)
{
	if (!btn || !emitDt)
	{
		return;
	}
	connect(btn, &QPushButton::pressed, this,
			[this, emitDt]()
			{
				stopJogHold();
				m_jogHoldEmit = emitDt;
				// 按下即按速率积分；松开停止（与真实示教盒一致）
				emitDt(jogTickDtSec());
				if (m_jogHoldRepeatTimer)
				{
					m_jogHoldRepeatTimer->start();
				}
			});
	connect(btn, &QPushButton::released, this, [this]() { stopJogHold(); });
}

RobotAxisJogTcpFrame RobotAxisJogControllerDialog::tcpFrame() const
{
	if (!m_frameCombo)
	{
		return RobotAxisJogTcpFrame::Tool;
	}
	return static_cast<RobotAxisJogTcpFrame>(m_frameCombo->currentData().toInt());
}

void RobotAxisJogControllerDialog::setUseChinese(bool chinese)
{
	m_useChinese = chinese;
	retranslate();
}

void RobotAxisJogControllerDialog::setJointNames(const QStringList& names)
{
	m_jointNames = names;
	rebuildAxisButtons();
}

void RobotAxisJogControllerDialog::retranslate()
{
	setWindowTitle(m_useChinese ? QStringLiteral("点动控制器") : QStringLiteral("Jog controller"));
	if (m_tabs)
	{
		m_tabs->setTabText(0, m_useChinese ? QStringLiteral("TCP 移动") : QStringLiteral("TCP move"));
		m_tabs->setTabText(1, m_useChinese ? QStringLiteral("轴移动") : QStringLiteral("Axis move"));
	}
	if (m_frameLabel)
	{
		m_frameLabel->setText(m_useChinese ? QStringLiteral("坐标系") : QStringLiteral("Frame"));
	}
	if (m_frameCombo && m_frameCombo->count() >= 4)
	{
		const int keep = m_frameCombo->currentIndex();
		m_frameCombo->setItemText(0, m_useChinese ? QStringLiteral("TCP/工具") : QStringLiteral("Tool/TCP"));
		m_frameCombo->setItemText(1, m_useChinese ? QStringLiteral("基座") : QStringLiteral("Base"));
		m_frameCombo->setItemText(2, m_useChinese ? QStringLiteral("世界") : QStringLiteral("World"));
		m_frameCombo->setItemText(3, m_useChinese ? QStringLiteral("用户") : QStringLiteral("User"));
		m_frameCombo->setCurrentIndex(keep >= 0 ? keep : 0);
	}
	if (m_transRateLabel)
	{
		m_transRateLabel->setText(m_useChinese ? QStringLiteral("线速度") : QStringLiteral("Linear"));
	}
	if (m_transRateCombo)
	{
		m_transRateCombo->setToolTip(m_useChinese ? QStringLiteral("TCP 点动线速度（mm/s），按住按钮连续移动")
												  : QStringLiteral("TCP jog linear speed (mm/s), hold to move"));
	}
	if (m_rotRateLabel)
	{
		m_rotRateLabel->setText(m_useChinese ? QStringLiteral("角速度") : QStringLiteral("Angular"));
	}
	if (m_rotRateSpin)
	{
		m_rotRateSpin->setToolTip(m_useChinese ? QStringLiteral("TCP 点动角速度（°/s）")
											   : QStringLiteral("TCP jog angular speed (deg/s)"));
	}
	if (m_jointRateLabel)
	{
		m_jointRateLabel->setText(m_useChinese ? QStringLiteral("轴角速度") : QStringLiteral("Joint rate"));
	}
	if (m_jointRateSpin)
	{
		m_jointRateSpin->setToolTip(m_useChinese ? QStringLiteral("关节点动角速度（°/s）")
												 : QStringLiteral("Joint jog angular speed (deg/s)"));
	}
	if (m_transSectionTitle)
	{
		m_transSectionTitle->setText(m_useChinese ? QStringLiteral("平移") : QStringLiteral("Translation"));
	}
	if (m_rotSectionTitle)
	{
		m_rotSectionTitle->setText(m_useChinese ? QStringLiteral("旋转") : QStringLiteral("Rotation"));
	}
}

void RobotAxisJogControllerDialog::rebuildAxisButtons()
{
	if (!m_axisLayout)
	{
		return;
	}
	while (m_axisLayout->count() > 2)
	{
		QLayoutItem* item = m_axisLayout->takeAt(1);
		if (!item)
		{
			break;
		}
		if (QWidget* w = item->widget())
		{
			delete w;
		}
		delete item;
	}

	for (int i = 0; i < m_jointNames.size(); ++i)
	{
		auto* card = new QFrame(m_axisPage);
		card->setObjectName(QStringLiteral("jogCard"));
		auto* hl = new QHBoxLayout(card);
		hl->setContentsMargins(10, 8, 10, 8);
		hl->setSpacing(8);

		auto* name = new QLabel(m_jointNames[i], card);
		name->setObjectName(QStringLiteral("jogFieldLabel"));
		name->setMinimumWidth(72);
		QPushButton* minus = makeJogButton(QStringLiteral("−"), card, AxisTone::Neutral);
		QPushButton* plus = makeJogButton(QStringLiteral("+"), card, AxisTone::Neutral);
		minus->setMaximumWidth(72);
		plus->setMaximumWidth(72);
		hl->addWidget(name, 1);
		hl->addWidget(minus);
		hl->addWidget(plus);
		bindJogHold(minus, [this, i](double dtSec) { emitJointDelta(i, -1.0, dtSec); });
		bindJogHold(plus, [this, i](double dtSec) { emitJointDelta(i, 1.0, dtSec); });
		m_axisLayout->insertWidget(m_axisLayout->count() - 1, card);
	}
}

void RobotAxisJogControllerDialog::emitTcpDelta(double dxMm, double dyMm, double dzMm, double dExDeg, double dEyDeg,
												 double dEzDeg)
{
	emit jogTcpDeltaRequested(dxMm, dyMm, dzMm, dExDeg, dEyDeg, dEzDeg, tcpFrame());
}

void RobotAxisJogControllerDialog::emitJointDelta(int jointIndex, double signDegPerSec, double dtSec)
{
	const double rateDeg = m_jointRateSpin ? m_jointRateSpin->value() : 15.0;
	emit jogJointDeltaRequested(jointIndex, signDegPerSec * rateDeg * dtSec * kPi / 180.0);
}
