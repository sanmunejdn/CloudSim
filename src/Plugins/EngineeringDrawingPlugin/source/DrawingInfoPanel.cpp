/// @file DrawingInfoPanel.cpp
/// @brief 工程图右侧说明（二期）

#include "DrawingInfoPanel.h"

#include <QLabel>
#include <QVBoxLayout>

DrawingInfoPanel::DrawingInfoPanel(QWidget* parent) : QWidget(parent)
{
	auto* root = new QVBoxLayout(this);
	root->setContentsMargins(8, 8, 8, 8);
	m_label = new QLabel(this);
	m_label->setWordWrap(true);
	root->addWidget(m_label);
	root->addStretch(1);
	applyLanguage(true);
}

void DrawingInfoPanel::applyLanguage(bool useChinese)
{
	if (!m_label)
		return;
	m_label->setText(
		useChinese
			? QStringLiteral(
				  "二期功能：\n"
				  "· 第一/第三角法、轴测、中面剖视\n"
				  "· 拖视图、线性尺寸、局部放大框选\n"
				  "· 导出 SVG / DXF\n"
				  "中键或 Alt+拖拽平移，滚轮缩放。")
			: QStringLiteral(
				  "Phase 2:\n"
				  "- 1st/3rd angle, iso, mid-plane section\n"
				  "- Drag views, linear dims, detail box\n"
				  "- Export SVG / DXF\n"
				  "Middle/Alt-drag pan, wheel zoom."));
}
