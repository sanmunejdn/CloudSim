/// @file ViewportToolBar.cpp
/// @brief ViewportToolBar 实现

#include "ViewportToolBar.h"

#include "UiIconDecorators.h"
#include "UiIcons.h"

#include <QEvent>
#include <QLabel>
#include <QPainter>
#include <QPainterPath>
#include <QSignalBlocker>
#include <QStyle>
#include <QTimer>
#include <QToolButton>
#include <QVBoxLayout>

namespace
{
constexpr int kBtnSize = 32;
constexpr int kIconSize = 18;
constexpr int kBarSpacing = 6;
constexpr int kGroupGap = 14;
constexpr int kTopMargin = 8;
constexpr int kBtnRadius = 8;
constexpr int kTipShowDelayMs = 380;
constexpr int kTipRadius = 8;

struct ViewportButtonColors
{
	QColor normal;
	QColor hover;
	QColor pressed;
	QColor checked;
};

struct ViewportTipColors
{
	QColor background;
	QColor border;
	QColor title;
	QColor subtitle;
};

QColor viewportBaseColor(bool dark)
{
	return dark ? QColor(56, 61, 74) : QColor(214, 222, 235);
}

ViewportTipColors tipColorsForTheme(bool dark)
{
	if (dark)
	{
		return {
			QColor(44, 46, 50),
			QColor(72, 74, 78),
			QColor(242, 242, 247),
			QColor(142, 142, 147),
		};
	}
	return {
		QColor(255, 255, 255),
		QColor(218, 220, 222),
		QColor(28, 28, 30),
		QColor(99, 99, 102),
	};
}

QColor blendOnViewport(bool dark, int fgR, int fgG, int fgB, double alpha)
{
	const int bgR = dark ? 56 : 214;
	const int bgG = dark ? 61 : 222;
	const int bgB = dark ? 74 : 235;
	const auto mix = [alpha](int bg, int fg) { return static_cast<int>(bg + (fg - bg) * alpha + 0.5); };
	return QColor(mix(bgR, fgR), mix(bgG, fgG), mix(bgB, fgB));
}

ViewportButtonColors colorsForTheme(bool dark)
{
	if (dark)
	{
		return {
			blendOnViewport(dark, 255, 255, 255, 0.12),
			blendOnViewport(dark, 255, 255, 255, 0.22),
			blendOnViewport(dark, 255, 255, 255, 0.30),
			blendOnViewport(dark, 66, 130, 218, 0.55),
		};
	}
	return {
		blendOnViewport(dark, 255, 255, 255, 0.55),
		blendOnViewport(dark, 255, 255, 255, 0.72),
		blendOnViewport(dark, 255, 255, 255, 0.82),
		blendOnViewport(dark, 0, 120, 215, 0.30),
	};
}

class ViewportActionTip : public QWidget
{
public:
	explicit ViewportActionTip(QWidget* parent = nullptr)
		: QWidget(parent, Qt::ToolTip | Qt::FramelessWindowHint | Qt::NoDropShadowWindowHint)
	{
		setAttribute(Qt::WA_ShowWithoutActivating, true);
		setAttribute(Qt::WA_TransparentForMouseEvents, true);
		setAutoFillBackground(false);
		setAttribute(Qt::WA_OpaquePaintEvent, true);

		auto* layout = new QVBoxLayout(this);
		layout->setContentsMargins(12, 9, 12, 9);
		layout->setSpacing(2);

		m_titleLabel = new QLabel(this);
		m_titleLabel->setWordWrap(false);
		QFont titleFont = m_titleLabel->font();
		titleFont.setPointSize(10);
		titleFont.setWeight(QFont::DemiBold);
		m_titleLabel->setFont(titleFont);

		m_subtitleLabel = new QLabel(this);
		m_subtitleLabel->setWordWrap(false);
		QFont subtitleFont = m_subtitleLabel->font();
		subtitleFont.setPointSize(9);
		m_subtitleLabel->setFont(subtitleFont);

		layout->addWidget(m_titleLabel);
		layout->addWidget(m_subtitleLabel);

		connect(&m_showTimer, &QTimer::timeout, this, &ViewportActionTip::showPendingTip);
		m_showTimer.setSingleShot(true);

		applyTheme(false);
	}

	void setDarkTheme(bool dark)
	{
		if (m_darkTheme == dark)
		{
			return;
		}
		m_darkTheme = dark;
		applyTheme(dark);
		update();
	}

	void scheduleShow(QWidget* anchor, const QString& title, const QString& subtitle)
	{
		m_pendingAnchor = anchor;
		m_pendingTitle = title;
		m_pendingSubtitle = subtitle;
		m_showTimer.start(kTipShowDelayMs);
	}

	void hideTip()
	{
		m_showTimer.stop();
		m_pendingAnchor.clear();
		hide();
	}

protected:
	void paintEvent(QPaintEvent* /*event*/) override
	{
		const ViewportTipColors colors = tipColorsForTheme(m_darkTheme);

		QPainter p(this);
		p.setRenderHint(QPainter::Antialiasing, true);

		QPainterPath path;
		path.addRoundedRect(QRectF(rect()).adjusted(0.5, 0.5, -0.5, -0.5), kTipRadius, kTipRadius);
		p.fillPath(path, colors.background);

		QPen borderPen(colors.border, 1.0);
		p.setPen(borderPen);
		p.drawPath(path);
	}

private:
	void applyTheme(bool dark)
	{
		const ViewportTipColors colors = tipColorsForTheme(dark);
		m_titleLabel->setStyleSheet(
			QStringLiteral("color: %1; background: transparent; border: none;").arg(colors.title.name(QColor::HexRgb)));
		m_subtitleLabel->setStyleSheet(QStringLiteral("color: %1; background: transparent; border: none;")
										   .arg(colors.subtitle.name(QColor::HexRgb)));
	}

	void showPendingTip()
	{
		QWidget* anchor = m_pendingAnchor.data();
		if (!anchor || !anchor->isVisible())
		{
			return;
		}

		m_titleLabel->setText(m_pendingTitle);
		m_subtitleLabel->setText(m_pendingSubtitle);
		m_subtitleLabel->setVisible(!m_pendingSubtitle.isEmpty());
		adjustSize();

		const QPoint anchorGlobal = anchor->mapToGlobal(QPoint(0, 0));
		const int x = anchorGlobal.x() + (anchor->width() - width()) / 2;
		const int y = anchorGlobal.y() + anchor->height() + 8;
		move(x, y);
		show();
		raise();
	}

	QTimer m_showTimer;
	QPointer<QWidget> m_pendingAnchor;
	QString m_pendingTitle;
	QString m_pendingSubtitle;
	QLabel* m_titleLabel = nullptr;
	QLabel* m_subtitleLabel = nullptr;
	bool m_darkTheme = false;
};

class ViewportIconButton : public QToolButton
{
public:
	explicit ViewportIconButton(ViewportActionTip* tip, QWidget* parent = nullptr)
		: QToolButton(parent), m_actionTip(tip)
	{
		setFixedSize(kBtnSize, kBtnSize);
		setToolButtonStyle(Qt::ToolButtonIconOnly);
		setIconSize(QSize(kIconSize, kIconSize));
		setAutoRaise(false);
		setCursor(Qt::PointingHandCursor);
		setFocusPolicy(Qt::NoFocus);
		setAutoFillBackground(false);
		setAttribute(Qt::WA_OpaquePaintEvent, true);
		applyViewportPalette();
	}

	void setActionTipText(const QString& title, const QString& subtitle)
	{
		m_tipTitle = title;
		m_tipSubtitle = subtitle;
	}

	void setTextGlyph(const QString& glyph)
	{
		m_textGlyph = glyph;
		update();
	}

	void setChromeDark(bool dark)
	{
		m_darkTheme = dark;
		applyViewportPalette();
		update();
	}

private:
	void applyViewportPalette()
	{
		QPalette pal = palette();
		const QColor bg = viewportBaseColor(m_darkTheme);
		pal.setColor(QPalette::Window, bg);
		pal.setColor(QPalette::Button, bg);
		setPalette(pal);
	}

protected:
	void paintEvent(QPaintEvent* /*event*/) override
	{
		const ViewportButtonColors palette = colorsForTheme(m_darkTheme);
		QColor fill = palette.normal;
		if (isChecked())
		{
			fill = palette.checked;
		}
		else if (isDown())
		{
			fill = palette.pressed;
		}
		else if (underMouse())
		{
			fill = palette.hover;
		}

		QPainter p(this);
		p.setRenderHint(QPainter::Antialiasing, true);
		p.setPen(Qt::NoPen);
		p.fillRect(rect(), viewportBaseColor(m_darkTheme));

		QPainterPath path;
		path.addRoundedRect(QRectF(rect()), kBtnRadius, kBtnRadius);
		p.fillPath(path, fill);

		const QIcon icon = this->icon();
		if (!m_textGlyph.isEmpty())
		{
			QFont font = p.font();
			font.setPointSize(13);
			font.setWeight(QFont::DemiBold);
			p.setFont(font);
			p.setPen(m_darkTheme ? QColor(242, 242, 247) : QColor(28, 28, 30));
			p.drawText(rect(), Qt::AlignCenter, m_textGlyph);
		}
		else if (!icon.isNull())
		{
			const QRect iconRect = QStyle::alignedRect(layoutDirection(), Qt::AlignCenter, iconSize(), rect());
			icon.paint(&p, iconRect, Qt::AlignCenter, isEnabled() ? QIcon::Normal : QIcon::Disabled);
		}
	}

	void enterEvent(QEvent* event) override
	{
		QToolButton::enterEvent(event);
		update();
		if (m_actionTip && !m_tipTitle.isEmpty())
		{
			m_actionTip->scheduleShow(this, m_tipTitle, m_tipSubtitle);
		}
	}

	void leaveEvent(QEvent* event) override
	{
		QToolButton::leaveEvent(event);
		update();
		if (m_actionTip)
		{
			m_actionTip->hideTip();
		}
	}

	ViewportActionTip* m_actionTip = nullptr;
	QString m_tipTitle;
	QString m_tipSubtitle;
	QString m_textGlyph;
	bool m_darkTheme = false;
};

ViewportIconButton* createToolButton(ViewportActionTip* tip, QWidget* host)
{
	return new ViewportIconButton(tip, host);
}

void applyChromeToButton(QToolButton* btn, bool dark)
{
	static_cast<ViewportIconButton*>(btn)->setChromeDark(dark);
}

} // namespace

ViewportToolBar::ViewportToolBar(QWidget* host) : QObject(host), m_host(host)
{
	if (!m_host)
	{
		return;
	}

	auto* actionTip = new ViewportActionTip(m_host);
	m_actionTip = actionTip;

	m_focusBtn = createToolButton(actionTip, m_host);
	UiIconDecorators::apply(m_focusBtn, UiIconId::FocusCamera, UiIconDecorators::IconPlacement::IconOnly,
							UiIcons::Size::Medium);
	static_cast<ViewportIconButton*>(m_focusBtn)
		->setActionTipText(QStringLiteral("视角自适应"), QStringLiteral("Focus Camera"));

	m_wireBtn = createToolButton(actionTip, m_host);
	UiIconDecorators::apply(m_wireBtn, UiIconId::Wireframe, UiIconDecorators::IconPlacement::IconOnly,
							UiIcons::Size::Medium);
	static_cast<ViewportIconButton*>(m_wireBtn)->setActionTipText(QStringLiteral("线框模式"),
																  QStringLiteral("Wireframe"));
	m_wireBtn->setCheckable(true);

	m_captureBtn = createToolButton(actionTip, m_host);
	UiIconDecorators::apply(m_captureBtn, UiIconId::Screenshot, UiIconDecorators::IconPlacement::IconOnly,
							UiIcons::Size::Medium);
	static_cast<ViewportIconButton*>(m_captureBtn)
		->setActionTipText(QStringLiteral("截图"), QStringLiteral("Screenshot"));

	connect(m_focusBtn, &QToolButton::clicked, this, &ViewportToolBar::focusRequested);
	connect(m_wireBtn, &QToolButton::toggled, this,
			[this](bool on)
			{
				m_wireframeOn = on;
				emit wireframeToggled(on);
			});
	connect(m_captureBtn, &QToolButton::clicked, this, &ViewportToolBar::screenshotRequested);

	m_leftPanelBtn = createToolButton(actionTip, m_host);
	m_leftPanelBtn->setCheckable(true);
	m_leftPanelBtn->setChecked(true);
	static_cast<ViewportIconButton*>(m_leftPanelBtn)->setTextGlyph(QStringLiteral("\u00ab"));
	updateLeftPanelChrome(true);

	m_rightPanelBtn = createToolButton(actionTip, m_host);
	m_rightPanelBtn->setCheckable(true);
	m_rightPanelBtn->setChecked(true);
	static_cast<ViewportIconButton*>(m_rightPanelBtn)->setTextGlyph(QStringLiteral("\u00bb"));
	updateRightPanelChrome(true);

	connect(m_leftPanelBtn, &QToolButton::toggled, this,
			[this](const bool visible)
			{
				updateLeftPanelChrome(visible);
				emit leftPanelVisibilityToggled(visible);
			});
	connect(m_rightPanelBtn, &QToolButton::toggled, this,
			[this](const bool visible)
			{
				updateRightPanelChrome(visible);
				emit rightPanelVisibilityToggled(visible);
			});

	updateSidePanelButtonTips();

	m_host->installEventFilter(this);
	applyButtonStyle();
	showButtons();
}

void ViewportToolBar::setUseChinese(bool useChinese)
{
	m_useChinese = useChinese;
	updateSidePanelButtonTips();
}

void ViewportToolBar::setSidePanelToggleState(const bool leftVisible, const bool rightVisible)
{
	if (m_leftPanelBtn)
	{
		const QSignalBlocker blocker(m_leftPanelBtn);
		m_leftPanelBtn->setChecked(leftVisible);
		updateLeftPanelChrome(leftVisible);
	}
	if (m_rightPanelBtn)
	{
		const QSignalBlocker blocker(m_rightPanelBtn);
		m_rightPanelBtn->setChecked(rightVisible);
		updateRightPanelChrome(rightVisible);
	}
}

void ViewportToolBar::updateLeftPanelChrome(const bool visible)
{
	if (!m_leftPanelBtn)
	{
		return;
	}
	auto* btn = static_cast<ViewportIconButton*>(m_leftPanelBtn);
	btn->setTextGlyph(visible ? QStringLiteral("\u00ab") : QStringLiteral("\u00bb"));
	btn->setActionTipText(visible ? (m_useChinese ? QStringLiteral("\u9690\u85cf\u5de6\u4fa7\u9762\u677f")
												  : QStringLiteral("Hide left panel"))
								  : (m_useChinese ? QStringLiteral("\u663e\u793a\u5de6\u4fa7\u9762\u677f")
												  : QStringLiteral("Show left panel")),
						  m_useChinese ? QStringLiteral("\u5c5e\u6027\u3001\u8bbe\u5907")
									   : QStringLiteral("Property, Devices"));
}

void ViewportToolBar::updateRightPanelChrome(const bool visible)
{
	if (!m_rightPanelBtn)
	{
		return;
	}
	auto* btn = static_cast<ViewportIconButton*>(m_rightPanelBtn);
	btn->setTextGlyph(visible ? QStringLiteral("\u00bb") : QStringLiteral("\u00ab"));
	btn->setActionTipText(visible ? (m_useChinese ? QStringLiteral("\u9690\u85cf\u53f3\u4fa7\u9762\u677f")
												  : QStringLiteral("Hide right panel"))
								  : (m_useChinese ? QStringLiteral("\u663e\u793a\u53f3\u4fa7\u9762\u677f")
												  : QStringLiteral("Show right panel")),
						  m_useChinese ? QStringLiteral("\u5de5\u4f5c\u533a\u3001\u63d2\u4ef6")
									   : QStringLiteral("Workspace, Plugins"));
}

void ViewportToolBar::updateSidePanelButtonTips()
{
	updateLeftPanelChrome(m_leftPanelBtn && m_leftPanelBtn->isChecked());
	updateRightPanelChrome(m_rightPanelBtn && m_rightPanelBtn->isChecked());
}

void ViewportToolBar::setDarkTheme(bool dark)
{
	m_darkTheme = dark;
	refreshChrome();
}

void ViewportToolBar::refreshChrome()
{
	applyButtonStyle();
	if (auto* tip = static_cast<ViewportActionTip*>(m_actionTip))
	{
		tip->setDarkTheme(m_darkTheme);
	}
	raiseButtons();
	reposition();
}

void ViewportToolBar::showButtons()
{
	if (!m_focusBtn || !m_wireBtn || !m_captureBtn || !m_leftPanelBtn || !m_rightPanelBtn)
	{
		return;
	}
	m_focusBtn->show();
	m_wireBtn->show();
	m_captureBtn->show();
	m_leftPanelBtn->show();
	m_rightPanelBtn->show();
	raiseButtons();
	reposition();
}

bool ViewportToolBar::eventFilter(QObject* obj, QEvent* ev)
{
	if (obj == m_host)
	{
		if (ev->type() == QEvent::Resize || ev->type() == QEvent::Show)
		{
			raiseButtons();
			reposition();
		}
		else if (ev->type() == QEvent::Hide && m_actionTip)
		{
			static_cast<ViewportActionTip*>(m_actionTip)->hideTip();
		}
	}
	return QObject::eventFilter(obj, ev);
}

void ViewportToolBar::raiseButtons()
{
	if (m_focusBtn)
	{
		m_focusBtn->raise();
	}
	if (m_wireBtn)
	{
		m_wireBtn->raise();
	}
	if (m_captureBtn)
	{
		m_captureBtn->raise();
	}
	if (m_leftPanelBtn)
	{
		m_leftPanelBtn->raise();
	}
	if (m_rightPanelBtn)
	{
		m_rightPanelBtn->raise();
	}
}

void ViewportToolBar::reposition()
{
	if (!m_host || !m_focusBtn || !m_wireBtn || !m_captureBtn || !m_leftPanelBtn || !m_rightPanelBtn)
	{
		return;
	}
	const int viewGroupW = kBtnSize * 3 + kBarSpacing * 2;
	const int totalW = viewGroupW + kGroupGap + kBtnSize * 2 + kBarSpacing;
	const int x0 = (m_host->width() - totalW) / 2;
	m_focusBtn->move(x0, kTopMargin);
	m_wireBtn->move(x0 + kBtnSize + kBarSpacing, kTopMargin);
	m_captureBtn->move(x0 + (kBtnSize + kBarSpacing) * 2, kTopMargin);
	const int sideX = x0 + viewGroupW + kGroupGap;
	m_leftPanelBtn->move(sideX, kTopMargin);
	m_rightPanelBtn->move(sideX + kBtnSize + kBarSpacing, kTopMargin);
}

void ViewportToolBar::applyButtonStyle()
{
	applyChromeToButton(m_focusBtn, m_darkTheme);
	applyChromeToButton(m_wireBtn, m_darkTheme);
	applyChromeToButton(m_captureBtn, m_darkTheme);
	applyChromeToButton(m_leftPanelBtn, m_darkTheme);
	applyChromeToButton(m_rightPanelBtn, m_darkTheme);
}
