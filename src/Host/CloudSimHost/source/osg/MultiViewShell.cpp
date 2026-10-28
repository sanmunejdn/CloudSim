/// @file MultiViewShell.cpp
/// @brief 多视角分屏壳

#include "MultiViewShell.h"

#include "DetachedViewWindow.h"
#include "OsgWidget.h"
#include "SatelliteOsgView.h"

#include <QCursor>
#include <QEvent>
#include <QFrame>
#include <QHBoxLayout>
#include <QMenu>
#include <QSettings>
#include <QSplitter>
#include <QToolButton>
#include <QVBoxLayout>

#include <algorithm>

namespace
{
constexpr char kSettingsGroup[] = "CloudSim/MultiView";
}

MultiViewShell::MultiViewShell(OsgWidget* primary, QWidget* parent) : QWidget(parent), m_primary(primary)
{
	auto* outer = new QVBoxLayout(this);
	outer->setContentsMargins(0, 0, 0, 0);
	outer->setSpacing(0);
	m_rootSplitter = new QSplitter(Qt::Horizontal, this);
	m_rootSplitter->setChildrenCollapsible(false);
	outer->addWidget(m_rootSplitter, 1);
	rebuildLayout();
}

MultiViewShell::~MultiViewShell()
{
	closeAllDetachedWindows();
}

int MultiViewShell::slotCountFor(cloudsim::host::MultiViewLayout layout)
{
	using L = cloudsim::host::MultiViewLayout;
	switch (layout)
	{
	case L::SplitH:
	case L::SplitV:
		return 2;
	case L::Quad:
		return 4;
	case L::Single:
	default:
		return 1;
	}
}

void MultiViewShell::setViewLayout(cloudsim::host::MultiViewLayout layout)
{
	if (m_layout == layout)
	{
		return;
	}
	m_layout = layout;
	if (m_activeIndex >= slotCountFor(m_layout))
	{
		m_activeIndex = 0;
	}
	rebuildLayout();
	emit layoutChanged(m_layout);
}

void MultiViewShell::setActiveViewIndex(int index)
{
	const int n = m_cells.size();
	if (index < 0 || index >= n || index == m_activeIndex)
	{
		if (index >= 0 && index < n)
		{
			updateActiveChrome();
		}
		return;
	}
	m_activeIndex = index;
	updateActiveChrome();
	emit activeViewChanged(m_activeIndex);
}

void MultiViewShell::ensureSatelliteCount(int count)
{
	const int needSat = (std::max)(0, count - 1);
	while (m_cells.size() < count)
	{
		Cell cell;
		cell.frame = new QFrame(this);
		cell.frame->setFrameShape(QFrame::Box);
		cell.frame->setLineWidth(1);
		cell.frame->setObjectName(QStringLiteral("multiViewCell"));
		auto* cellLay = new QVBoxLayout(cell.frame);
		cellLay->setContentsMargins(1, 1, 1, 1);
		cellLay->setSpacing(0);

		auto* topBar = new QHBoxLayout();
		topBar->setContentsMargins(0, 0, 0, 0);
		topBar->addStretch(1);
		cell.cornerBtn = new QToolButton(cell.frame);
		cell.cornerBtn->setText(QStringLiteral("☰"));
		cell.cornerBtn->setAutoRaise(true);
		cell.cornerBtn->setFixedSize(22, 22);
		topBar->addWidget(cell.cornerBtn);
		cellLay->addLayout(topBar);

		cell.contentHost = new QWidget(cell.frame);
		auto* contentLay = new QVBoxLayout(cell.contentHost);
		contentLay->setContentsMargins(0, 0, 0, 0);
		contentLay->setSpacing(0);
		cellLay->addWidget(cell.contentHost, 1);

		const int idx = m_cells.size();
		m_cells.push_back(cell);
		wireCellActivation(idx);
		connect(cell.cornerBtn, &QToolButton::clicked, this, [this, idx]() { showCornerMenu(idx); });
	}

	while (static_cast<int>(m_cells.size()) > count)
	{
		Cell cell = m_cells.takeLast();
		if (cell.satellite)
		{
			cell.satellite->deleteLater();
			cell.satellite = nullptr;
		}
		if (cell.frame)
		{
			cell.frame->deleteLater();
		}
	}

	for (int i = 1; i < count; ++i)
	{
		if (!m_cells[i].satellite)
		{
			m_cells[i].satellite = createSatellite();
			if (auto* lay = m_cells[i].contentHost->layout())
			{
				lay->addWidget(m_cells[i].satellite);
			}
			connect(m_cells[i].satellite, &SatelliteOsgView::activated, this,
					[this, i]() { setActiveViewIndex(i); });
		}
	}
	destroySatellitesBeyond(needSat);
}

void MultiViewShell::destroySatellitesBeyond(int keep)
{
	for (int i = keep + 1; i < m_cells.size(); ++i)
	{
		if (m_cells[i].satellite)
		{
			m_cells[i].satellite->deleteLater();
			m_cells[i].satellite = nullptr;
		}
	}
}

SatelliteOsgView* MultiViewShell::createSatellite()
{
	auto* sat = new SatelliteOsgView(m_primary, this);
	sat->setOrthographicProjection(m_orthographic);
	return sat;
}

void MultiViewShell::setOrthographicProjection(bool on)
{
	m_orthographic = on;
	if (m_primary)
	{
		m_primary->setOrthographicProjection(on);
	}
	for (const Cell& cell : m_cells)
	{
		if (cell.satellite)
		{
			cell.satellite->setOrthographicProjection(on);
		}
	}
	for (const auto& p : m_detached)
	{
		if (p && p->satelliteView())
		{
			p->satelliteView()->setOrthographicProjection(on);
		}
	}
}

void MultiViewShell::placePrimaryInCell(int index)
{
	if (!m_primary || index < 0 || index >= m_cells.size())
	{
		return;
	}
	QWidget* host = m_cells[index].contentHost;
	if (!host)
	{
		return;
	}
	if (m_primary->parentWidget() != host)
	{
		m_primary->setParent(host);
	}
	if (auto* lay = host->layout())
	{
		if (lay->indexOf(m_primary) < 0)
		{
			lay->addWidget(m_primary);
		}
	}
	m_primary->show();
}

void MultiViewShell::rebuildLayout()
{
	const int paneCount = slotCountFor(m_layout);
	ensureSatelliteCount(paneCount);

	// 先摘格 frame，再清嵌套 splitter，避免连格子一起删掉
	for (auto& cell : m_cells)
	{
		if (cell.frame)
		{
			cell.frame->setParent(this);
			cell.frame->hide();
		}
	}
	while (m_rootSplitter->count() > 0)
	{
		QWidget* w = m_rootSplitter->widget(0);
		if (!w)
		{
			break;
		}
		w->setParent(nullptr);
		w->deleteLater();
	}

	using L = cloudsim::host::MultiViewLayout;

	placePrimaryInCell(0);

	if (m_layout == L::Single)
	{
		m_rootSplitter->setOrientation(Qt::Horizontal);
		m_cells[0].frame->show();
		m_rootSplitter->addWidget(m_cells[0].frame);
	}
	else if (m_layout == L::SplitH)
	{
		m_rootSplitter->setOrientation(Qt::Horizontal);
		m_cells[0].frame->show();
		m_cells[1].frame->show();
		m_rootSplitter->addWidget(m_cells[0].frame);
		m_rootSplitter->addWidget(m_cells[1].frame);
		m_rootSplitter->setSizes({1, 1});
	}
	else if (m_layout == L::SplitV)
	{
		m_rootSplitter->setOrientation(Qt::Vertical);
		m_cells[0].frame->show();
		m_cells[1].frame->show();
		m_rootSplitter->addWidget(m_cells[0].frame);
		m_rootSplitter->addWidget(m_cells[1].frame);
		m_rootSplitter->setSizes({1, 1});
	}
	else // Quad
	{
		m_rootSplitter->setOrientation(Qt::Vertical);
		auto* top = new QSplitter(Qt::Horizontal, m_rootSplitter);
		auto* bottom = new QSplitter(Qt::Horizontal, m_rootSplitter);
		top->setChildrenCollapsible(false);
		bottom->setChildrenCollapsible(false);
		for (int i = 0; i < 4; ++i)
		{
			m_cells[i].frame->show();
		}
		top->addWidget(m_cells[0].frame);
		top->addWidget(m_cells[1].frame);
		bottom->addWidget(m_cells[2].frame);
		bottom->addWidget(m_cells[3].frame);
		top->setSizes({1, 1});
		bottom->setSizes({1, 1});
		m_rootSplitter->addWidget(top);
		m_rootSplitter->addWidget(bottom);
		m_rootSplitter->setSizes({1, 1});
	}

	updateActiveChrome();
	refreshChrome();
}

void MultiViewShell::wireCellActivation(int index)
{
	if (index < 0 || index >= m_cells.size() || !m_cells[index].frame)
	{
		return;
	}
	m_cells[index].frame->installEventFilter(this);
	if (m_cells[index].contentHost)
	{
		m_cells[index].contentHost->installEventFilter(this);
	}
}

bool MultiViewShell::eventFilter(QObject* watched, QEvent* event)
{
	if (event && event->type() == QEvent::MouseButtonPress)
	{
		for (int i = 0; i < m_cells.size(); ++i)
		{
			if (watched == m_cells[i].frame || watched == m_cells[i].contentHost)
			{
				setActiveViewIndex(i);
				break;
			}
		}
	}
	return QWidget::eventFilter(watched, event);
}

void MultiViewShell::updateActiveChrome()
{
	for (int i = 0; i < m_cells.size(); ++i)
	{
		if (!m_cells[i].frame)
		{
			continue;
		}
		const bool active = (i == m_activeIndex);
		m_cells[i].frame->setStyleSheet(active ? QStringLiteral("QFrame#multiViewCell { border: 2px solid #3d8bfd; }")
											   : QStringLiteral("QFrame#multiViewCell { border: 1px solid #555; }"));
	}
}

void MultiViewShell::showCornerMenu(int cellIndex)
{
	if (cellIndex < 0 || cellIndex >= m_cells.size())
	{
		return;
	}
	QMenu menu(this);
	const auto addPreset = [&](const QString& en, const QString& zh, SatelliteOsgView::ViewPreset p)
	{
		menu.addAction(m_useChinese ? zh : en, this, [this, cellIndex, p]() { applyPresetToCell(cellIndex, p); });
	};
	addPreset(QStringLiteral("Front"), QStringLiteral("前视"), SatelliteOsgView::ViewPreset::Front);
	addPreset(QStringLiteral("Top"), QStringLiteral("顶视"), SatelliteOsgView::ViewPreset::Top);
	addPreset(QStringLiteral("Right"), QStringLiteral("右视"), SatelliteOsgView::ViewPreset::Right);
	addPreset(QStringLiteral("Isometric"), QStringLiteral("等轴"), SatelliteOsgView::ViewPreset::Iso);
	addPreset(QStringLiteral("Reset"), QStringLiteral("重置"), SatelliteOsgView::ViewPreset::Reset);

	if (cellIndex > 0 && m_layout != cloudsim::host::MultiViewLayout::Single)
	{
		menu.addSeparator();
		menu.addAction(m_useChinese ? QStringLiteral("关闭此格（回单视口）") : QStringLiteral("Close pane (single)"),
					   this, [this]() { setViewLayout(cloudsim::host::MultiViewLayout::Single); });
	}
	menu.exec(QCursor::pos());
}

void MultiViewShell::applyPresetToCell(int cellIndex, SatelliteOsgView::ViewPreset preset)
{
	if (cellIndex == 0)
	{
		applyPresetToPrimary(preset);
		return;
	}
	if (cellIndex > 0 && cellIndex < m_cells.size() && m_cells[cellIndex].satellite)
	{
		m_cells[cellIndex].satellite->applyViewPreset(preset);
	}
}

void MultiViewShell::applyPresetToPrimary(SatelliteOsgView::ViewPreset preset)
{
	if (!m_primary)
	{
		return;
	}
	using P = OsgScene::CameraViewPreset;
	switch (preset)
	{
	case SatelliteOsgView::ViewPreset::Front:
		m_primary->setCameraViewPreset(P::Front);
		break;
	case SatelliteOsgView::ViewPreset::Top:
		m_primary->setCameraViewPreset(P::Top);
		break;
	case SatelliteOsgView::ViewPreset::Right:
		m_primary->setCameraViewPreset(P::Right);
		break;
	case SatelliteOsgView::ViewPreset::Iso:
	case SatelliteOsgView::ViewPreset::Reset:
	default:
		m_primary->setCameraViewPreset(P::Iso);
		break;
	}
}

bool MultiViewShell::openDetachedViewWindow()
{
	// 清掉已关闭的悬空指针
	m_detached.erase(std::remove_if(m_detached.begin(), m_detached.end(),
									[](const QPointer<DetachedViewWindow>& p) { return p.isNull(); }),
					 m_detached.end());
	if (m_detached.size() >= kMaxDetached || !m_primary)
	{
		return false;
	}
	auto* dlg = new DetachedViewWindow(m_primary, window());
	dlg->setWindowTitle(m_useChinese ? QStringLiteral("视角窗口") : QStringLiteral("View Window"));
	if (dlg->satelliteView())
	{
		dlg->satelliteView()->setOrthographicProjection(m_orthographic);
	}
	m_detached.push_back(dlg);
	dlg->show();
	return true;
}

void MultiViewShell::closeAllDetachedWindows()
{
	for (const auto& p : m_detached)
	{
		if (p)
		{
			p->close();
		}
	}
	m_detached.clear();
}

int MultiViewShell::detachedWindowCount() const
{
	int n = 0;
	for (const auto& p : m_detached)
	{
		if (!p.isNull())
		{
			++n;
		}
	}
	return n;
}

void MultiViewShell::persistSettings(const QString& documentId) const
{
	if (documentId.isEmpty())
	{
		return;
	}
	QSettings s;
	s.beginGroup(QString::fromLatin1(kSettingsGroup));
	s.beginGroup(documentId);
	s.setValue(QStringLiteral("layout"), static_cast<int>(m_layout));
	s.endGroup();
	s.endGroup();
}

void MultiViewShell::restoreSettings(const QString& documentId)
{
	if (documentId.isEmpty())
	{
		return;
	}
	QSettings s;
	s.beginGroup(QString::fromLatin1(kSettingsGroup));
	s.beginGroup(documentId);
	const int v = s.value(QStringLiteral("layout"), static_cast<int>(cloudsim::host::MultiViewLayout::Single)).toInt();
	s.endGroup();
	s.endGroup();
	if (v >= 0 && v <= static_cast<int>(cloudsim::host::MultiViewLayout::Quad))
	{
		setViewLayout(static_cast<cloudsim::host::MultiViewLayout>(v));
	}
}

void MultiViewShell::setUseChinese(bool chinese)
{
	m_useChinese = chinese;
	refreshChrome();
}

void MultiViewShell::refreshChrome()
{
	for (auto& cell : m_cells)
	{
		if (cell.cornerBtn)
		{
			cell.cornerBtn->setToolTip(m_useChinese ? QStringLiteral("视角预设") : QStringLiteral("View presets"));
		}
	}
}
