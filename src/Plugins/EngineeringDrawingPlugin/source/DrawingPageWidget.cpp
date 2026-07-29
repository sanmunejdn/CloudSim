/// @file DrawingPageWidget.cpp
/// @brief 工程图中央页（仅画布）

#include "DrawingPageWidget.h"

#include <QVBoxLayout>

DrawingPageWidget::DrawingPageWidget(QWidget* parent) : QWidget(parent)
{
	auto* root = new QVBoxLayout(this);
	root->setContentsMargins(0, 0, 0, 0);
	root->setSpacing(0);
	m_canvas = new DrawingSheetCanvasWidget(this);
	root->addWidget(m_canvas, 1);
}

void DrawingPageWidget::applyLanguage(bool /*useChinese*/)
{
}
