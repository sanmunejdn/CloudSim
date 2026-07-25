/// @file StyledDockTitleBar.cpp
/// @brief Dock 标题栏绘制与按钮交互

#include "StyledDockTitleBar.h"

#include "UiIconDecorators.h"
#include "UiIconId.h"
#include "UiIcons.h"

#include <QDockWidget>
#include <QEvent>
#include <QHBoxLayout>
#include <QLabel>
#include <QMouseEvent>
#include <QPainter>
#include <QStyleOption>
#include <QToolButton>

namespace
{
QToolButton* makeTitleButton(QWidget* parent, const QString& objectName, const QString& tip)
{
	auto* btn = new QToolButton(parent);
	btn->setObjectName(objectName);
	btn->setFixedSize(28, 28);
	btn->setCursor(Qt::ArrowCursor);
	btn->setToolTip(tip);
	btn->setFocusPolicy(Qt::NoFocus);
	btn->setAutoRaise(true);
	btn->setToolButtonStyle(Qt::ToolButtonIconOnly);
	return btn;
}
} // namespace

StyledDockTitleBar::StyledDockTitleBar(QDockWidget* dock) : QWidget(dock), m_dock(dock)
{
	setObjectName(QStringLiteral("StyledDockTitleBar"));
	setFixedHeight(36);
	setAttribute(Qt::WA_StyledBackground, true);

	m_title = new QLabel(this);
	m_title->setObjectName(QStringLiteral("StyledDockTitleLabel"));
	m_title->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);

	m_floatBtn = makeTitleButton(this, QStringLiteral("DockTitleFloatBtn"), QStringLiteral("悬浮"));
	m_closeBtn = makeTitleButton(this, QStringLiteral("DockTitleCloseBtn"), QStringLiteral("关闭"));
	// 使用共享资源库 PNG，避免手绘 QPixmap 在 HiDPI 下模糊/裁切
	UiIconDecorators::apply(m_floatBtn, UiIconId::DockFloat, UiIconDecorators::IconPlacement::IconOnly,
							UiIcons::Size::Small);
	UiIconDecorators::apply(m_closeBtn, UiIconId::Close, UiIconDecorators::IconPlacement::IconOnly,
							UiIcons::Size::Small);

	auto* row = new QHBoxLayout(this);
	row->setContentsMargins(12, 0, 6, 0);
	row->setSpacing(4);
	row->addWidget(m_title, 1);
	row->addWidget(m_floatBtn, 0, Qt::AlignVCenter);
	row->addWidget(m_closeBtn, 0, Qt::AlignVCenter);

	setStyleSheet(QStringLiteral(
		"QWidget#StyledDockTitleBar {"
		"  background-color: #F3F5F8;"
		"  border-bottom: 1px solid #D5DCE3;"
		"}"
		"QLabel#StyledDockTitleLabel {"
		"  color: #1A2332;"
		"  font-size: 13px;"
		"  font-weight: 700;"
		"}"
		"QToolButton#DockTitleFloatBtn, QToolButton#DockTitleCloseBtn {"
		"  background-color: transparent;"
		"  border: 1px solid transparent;"
		"  border-radius: 6px;"
		"  padding: 2px;"
		"  margin: 0px;"
		"}"
		"QToolButton#DockTitleFloatBtn:hover {"
		"  background-color: #E5EBF1;"
		"  border-color: #C9D3DD;"
		"}"
		"QToolButton#DockTitleFloatBtn:pressed {"
		"  background-color: #D8E0E8;"
		"}"
		"QToolButton#DockTitleCloseBtn:hover {"
		"  background-color: #FDE8E8;"
		"  border-color: #F0B4B4;"
		"}"
		"QToolButton#DockTitleCloseBtn:pressed {"
		"  background-color: #F8D0D0;"
		"}"));

	connect(m_floatBtn, &QToolButton::clicked, this, &StyledDockTitleBar::toggleFloating);
	connect(m_closeBtn, &QToolButton::clicked, this, &StyledDockTitleBar::requestClose);
	if (m_dock)
	{
		m_dock->installEventFilter(this);
		connect(m_dock, &QDockWidget::topLevelChanged, this, [this](bool) { syncFloatButton(); });
	}
	syncTitle();
	syncFloatButton();
}

bool StyledDockTitleBar::eventFilter(QObject* watched, QEvent* event)
{
	if (watched == m_dock && event && event->type() == QEvent::WindowTitleChange)
	{
		syncTitle();
	}
	return QWidget::eventFilter(watched, event);
}

void StyledDockTitleBar::syncTitle()
{
	if (m_title && m_dock)
	{
		m_title->setText(m_dock->windowTitle());
	}
}

void StyledDockTitleBar::syncFloatButton()
{
	if (!m_floatBtn || !m_dock)
	{
		return;
	}
	const bool floating = m_dock->isFloating();
	m_floatBtn->setToolTip(floating ? QStringLiteral("停靠") : QStringLiteral("悬浮"));
	m_floatBtn->setVisible(m_dock->features().testFlag(QDockWidget::DockWidgetFloatable));
	if (m_closeBtn)
	{
		m_closeBtn->setVisible(m_dock->features().testFlag(QDockWidget::DockWidgetClosable));
	}
}

void StyledDockTitleBar::toggleFloating()
{
	if (!m_dock || !m_dock->features().testFlag(QDockWidget::DockWidgetFloatable))
	{
		return;
	}
	m_dock->setFloating(!m_dock->isFloating());
}

void StyledDockTitleBar::requestClose()
{
	if (!m_dock)
	{
		return;
	}
	m_dock->close();
}

void StyledDockTitleBar::mousePressEvent(QMouseEvent* event)
{
	if (event->button() == Qt::LeftButton)
	{
		m_pressGlobal = event->globalPos();
		m_dragging = false;
	}
	QWidget::mousePressEvent(event);
}

void StyledDockTitleBar::mouseMoveEvent(QMouseEvent* event)
{
	if (!(event->buttons() & Qt::LeftButton) || !m_dock)
	{
		QWidget::mouseMoveEvent(event);
		return;
	}
	const QPoint delta = event->globalPos() - m_pressGlobal;
	if (!m_dragging && delta.manhattanLength() >= 6)
	{
		m_dragging = true;
		if (!m_dock->isFloating() && m_dock->features().testFlag(QDockWidget::DockWidgetFloatable))
		{
			m_dock->setFloating(true);
			m_pressGlobal = event->globalPos();
		}
	}
	if (m_dragging && m_dock->isFloating())
	{
		m_dock->move(m_dock->pos() + (event->globalPos() - m_pressGlobal));
		m_pressGlobal = event->globalPos();
	}
	QWidget::mouseMoveEvent(event);
}

void StyledDockTitleBar::mouseReleaseEvent(QMouseEvent* event)
{
	m_dragging = false;
	QWidget::mouseReleaseEvent(event);
}

void StyledDockTitleBar::mouseDoubleClickEvent(QMouseEvent* event)
{
	if (event->button() == Qt::LeftButton)
	{
		toggleFloating();
	}
	QWidget::mouseDoubleClickEvent(event);
}

void StyledDockTitleBar::paintEvent(QPaintEvent* event)
{
	QStyleOption opt;
	opt.initFrom(this);
	QPainter p(this);
	style()->drawPrimitive(QStyle::PE_Widget, &opt, &p, this);
	QWidget::paintEvent(event);
}

void applyStyledDockTitleBar(QDockWidget* dock)
{
	if (!dock)
	{
		return;
	}
	dock->setTitleBarWidget(new StyledDockTitleBar(dock));
}
