/// @file DetachedViewWindow.cpp
/// @brief 浮动视角窗

#include "DetachedViewWindow.h"

#include "OsgWidget.h"
#include "SatelliteOsgView.h"

#include <QVBoxLayout>

DetachedViewWindow::DetachedViewWindow(OsgWidget* primary, QWidget* parent) : QDialog(parent)
{
	setAttribute(Qt::WA_DeleteOnClose, true);
	setWindowTitle(QStringLiteral("View"));
	setMinimumSize(400, 300);
	resize(640, 480);

	auto* layout = new QVBoxLayout(this);
	layout->setContentsMargins(0, 0, 0, 0);
	m_satellite = new SatelliteOsgView(primary, this);
	layout->addWidget(m_satellite, 1);
}

DetachedViewWindow::~DetachedViewWindow() = default;
