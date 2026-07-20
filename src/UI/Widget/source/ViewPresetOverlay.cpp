/// @file ViewPresetOverlay.cpp
/// @brief ViewPresetOverlay 实现

#include "ViewPresetOverlay.h"

#include <QFont>
#include <QGridLayout>
#include <QLabel>
#include <QSizePolicy>
#include <QToolButton>
#include <QVBoxLayout>

namespace
{
constexpr int kCubeBtnSize = 44;
constexpr int kIsoBtnHeight = 36;

QString presetTooltip(OsgScene::CameraViewPreset preset, bool chinese)
{
	if (chinese)
	{
		switch (preset)
		{
		case OsgScene::CameraViewPreset::Front:
			return QStringLiteral("正视 (+Y)");
		case OsgScene::CameraViewPreset::Back:
			return QStringLiteral("后视 (-Y)");
		case OsgScene::CameraViewPreset::Left:
			return QStringLiteral("左视 (-X)");
		case OsgScene::CameraViewPreset::Right:
			return QStringLiteral("右视 (+X)");
		case OsgScene::CameraViewPreset::Top:
			return QStringLiteral("俯视 (+Z)");
		case OsgScene::CameraViewPreset::Bottom:
			return QStringLiteral("仰视 (-Z)");
		case OsgScene::CameraViewPreset::Iso:
			return QStringLiteral("等轴测视图");
		}
	}
	switch (preset)
	{
	case OsgScene::CameraViewPreset::Front:
		return QStringLiteral("Front (+Y)");
	case OsgScene::CameraViewPreset::Back:
		return QStringLiteral("Back (-Y)");
	case OsgScene::CameraViewPreset::Left:
		return QStringLiteral("Left (-X)");
	case OsgScene::CameraViewPreset::Right:
		return QStringLiteral("Right (+X)");
	case OsgScene::CameraViewPreset::Top:
		return QStringLiteral("Top (+Z)");
	case OsgScene::CameraViewPreset::Bottom:
		return QStringLiteral("Bottom (-Z)");
	case OsgScene::CameraViewPreset::Iso:
		return QStringLiteral("Isometric view");
	}
	return QString();
}

QString presetShortLabel(OsgScene::CameraViewPreset preset, bool chinese)
{
	if (chinese)
	{
		switch (preset)
		{
		case OsgScene::CameraViewPreset::Front:
			return QStringLiteral("前");
		case OsgScene::CameraViewPreset::Back:
			return QStringLiteral("后");
		case OsgScene::CameraViewPreset::Left:
			return QStringLiteral("左");
		case OsgScene::CameraViewPreset::Right:
			return QStringLiteral("右");
		case OsgScene::CameraViewPreset::Top:
			return QStringLiteral("顶");
		case OsgScene::CameraViewPreset::Bottom:
			return QStringLiteral("底");
		case OsgScene::CameraViewPreset::Iso:
			return QStringLiteral("等轴");
		}
	}
	switch (preset)
	{
	case OsgScene::CameraViewPreset::Front:
		return QStringLiteral("F");
	case OsgScene::CameraViewPreset::Back:
		return QStringLiteral("B");
	case OsgScene::CameraViewPreset::Left:
		return QStringLiteral("L");
	case OsgScene::CameraViewPreset::Right:
		return QStringLiteral("R");
	case OsgScene::CameraViewPreset::Top:
		return QStringLiteral("T");
	case OsgScene::CameraViewPreset::Bottom:
		return QStringLiteral("D");
	case OsgScene::CameraViewPreset::Iso:
		return QStringLiteral("Iso");
	}
	return QString();
}

} // namespace

ViewPresetOverlay::ViewPresetOverlay(QWidget* parent) : QWidget(parent)
{
	setAttribute(Qt::WA_StyledBackground, true);

	auto* root = new QVBoxLayout(this);
	root->setContentsMargins(12, 10, 12, 12);
	root->setSpacing(8);

	m_titleLabel = new QLabel(this);
	m_titleLabel->setAlignment(Qt::AlignCenter);
	QFont titleFont = m_titleLabel->font();
	titleFont.setPointSize(9);
	titleFont.setLetterSpacing(QFont::AbsoluteSpacing, 1.2);
	m_titleLabel->setFont(titleFont);
	root->addWidget(m_titleLabel);

	auto* gridHost = new QWidget(this);
	gridHost->setObjectName(QStringLiteral("viewPresetCubeHost"));
	auto* grid = new QGridLayout(gridHost);
	grid->setContentsMargins(0, 0, 0, 0);
	grid->setHorizontalSpacing(5);
	grid->setVerticalSpacing(5);

	const auto addBtn = [this, grid](int row, int col, OsgScene::CameraViewPreset preset, bool emphasize = false)
	{
		return addPresetButton(grid, row, col, 1, 1, presetShortLabel(preset, false), presetShortLabel(preset, true),
							   preset, emphasize);
	};

	addBtn(0, 1, OsgScene::CameraViewPreset::Top);
	addBtn(1, 0, OsgScene::CameraViewPreset::Left);
	addBtn(1, 1, OsgScene::CameraViewPreset::Front, true);
	addBtn(1, 2, OsgScene::CameraViewPreset::Right);
	addBtn(1, 3, OsgScene::CameraViewPreset::Back);
	addBtn(2, 1, OsgScene::CameraViewPreset::Bottom);

	root->addWidget(gridHost);

	m_isoButton = addPresetButton(nullptr, 0, 0, 1, 1, QStringLiteral("Iso"), QStringLiteral("等轴"),
								  OsgScene::CameraViewPreset::Iso);
	m_isoButton->setMinimumHeight(kIsoBtnHeight);
	m_isoButton->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
	root->addWidget(m_isoButton);

	refreshButtonLabels();
	applyPanelStyle();
}

void ViewPresetOverlay::setUseChinese(bool chinese)
{
	m_useChinese = chinese;
	refreshButtonLabels();
}

void ViewPresetOverlay::setDarkTheme(bool dark)
{
	if (m_darkTheme == dark)
	{
		return;
	}
	m_darkTheme = dark;
	applyPanelStyle();
}

void ViewPresetOverlay::refreshButtonLabels()
{
	m_titleLabel->setText(m_useChinese ? QStringLiteral("视角") : QStringLiteral("VIEW"));

	for (QToolButton* btn : findChildren<QToolButton*>())
	{
		const auto preset = static_cast<OsgScene::CameraViewPreset>(btn->property("viewPreset").toInt());
		const bool isIso = preset == OsgScene::CameraViewPreset::Iso;
		btn->setText(presetShortLabel(preset, m_useChinese));
		btn->setToolTip(presetTooltip(preset, m_useChinese));
		if (isIso)
		{
			btn->setMinimumWidth(0);
		}
	}
}

void ViewPresetOverlay::applyPanelStyle()
{
	const QString panelBg =
		m_darkTheme ? QStringLiteral("rgba(32, 34, 38, 220)") : QStringLiteral("rgba(255, 255, 255, 235)");
	const QString panelBorder =
		m_darkTheme ? QStringLiteral("rgba(255, 255, 255, 0.10)") : QStringLiteral("rgba(0, 0, 0, 0.08)");
	const QString titleColor =
		m_darkTheme ? QStringLiteral("rgba(255, 255, 255, 0.45)") : QStringLiteral("rgba(0, 0, 0, 0.45)");
	const QString btnBg =
		m_darkTheme ? QStringLiteral("rgba(255, 255, 255, 0.06)") : QStringLiteral("rgba(0, 0, 0, 0.04)");
	const QString btnBorder =
		m_darkTheme ? QStringLiteral("rgba(255, 255, 255, 0.12)") : QStringLiteral("rgba(0, 0, 0, 0.10)");
	const QString btnText =
		m_darkTheme ? QStringLiteral("rgba(255, 255, 255, 0.88)") : QStringLiteral("rgba(0, 0, 0, 0.82)");
	const QString btnHover =
		m_darkTheme ? QStringLiteral("rgba(66, 130, 218, 0.42)") : QStringLiteral("rgba(0, 120, 215, 0.18)");
	const QString btnPressed =
		m_darkTheme ? QStringLiteral("rgba(42, 130, 218, 0.55)") : QStringLiteral("rgba(0, 120, 215, 0.28)");
	const QString frontBorder =
		m_darkTheme ? QStringLiteral("rgba(66, 163, 230, 0.75)") : QStringLiteral("rgba(0, 120, 215, 0.55)");
	const QString frontBg =
		m_darkTheme ? QStringLiteral("rgba(66, 130, 218, 0.22)") : QStringLiteral("rgba(0, 120, 215, 0.10)");

	m_titleLabel->setStyleSheet(QStringLiteral("color: %1; background: transparent; border: none;").arg(titleColor));

	setStyleSheet(QStringLiteral("ViewPresetOverlay {"
								 "  background-color: %1;"
								 "  border: 1px solid %2;"
								 "  border-radius: 0;"
								 "}"
								 "#viewPresetCubeHost { background: transparent; border: none; }"
								 "QToolButton {"
								 "  min-width: %3px; max-width: %3px;"
								 "  min-height: %3px; max-height: %3px;"
								 "  padding: 0;"
								 "  font-size: 13px;"
								 "  font-weight: 500;"
								 "  color: %4;"
								 "  background-color: %5;"
								 "  border: 1px solid %6;"
								 "  border-radius: 4px;"
								 "}"
								 "QToolButton:hover { background-color: %7; }"
								 "QToolButton:pressed { background-color: %8; }"
								 "QToolButton[emphasis=\"true\"] {"
								 "  background-color: %9;"
								 "  border: 1px solid %10;"
								 "  font-weight: 600;"
								 "}"
								 "QToolButton[iso=\"true\"] {"
								 "  min-width: 0; max-width: 16777215;"
								 "  min-height: %11px; max-height: %11px;"
								 "  border-radius: 4px;"
								 "  font-size: 12px;"
								 "  letter-spacing: 0.5px;"
								 "}")
					  .arg(panelBg, panelBorder, QString::number(kCubeBtnSize), btnText, btnBg, btnBorder, btnHover,
						   btnPressed, frontBg, frontBorder, QString::number(kIsoBtnHeight)));
}

QToolButton* ViewPresetOverlay::addPresetButton(QGridLayout* grid, int row, int col, int rowSpan, int colSpan,
												const QString& labelEn, const QString& labelZh,
												OsgScene::CameraViewPreset preset, bool emphasize)
{
	auto* btn = new QToolButton(this);
	btn->setText(m_useChinese ? labelZh : labelEn);
	btn->setToolButtonStyle(Qt::ToolButtonTextOnly);
	btn->setAutoRaise(false);
	btn->setCursor(Qt::PointingHandCursor);
	btn->setToolTip(presetTooltip(preset, m_useChinese));
	btn->setProperty("viewPreset", static_cast<int>(preset));
	if (emphasize)
	{
		btn->setProperty("emphasis", true);
	}
	if (preset == OsgScene::CameraViewPreset::Iso)
	{
		btn->setProperty("iso", true);
	}
	if (grid)
	{
		grid->addWidget(btn, row, col, rowSpan, colSpan, Qt::AlignCenter);
	}
	connect(btn, &QToolButton::clicked, this, [this, preset]() { emit presetRequested(preset); });
	return btn;
}
