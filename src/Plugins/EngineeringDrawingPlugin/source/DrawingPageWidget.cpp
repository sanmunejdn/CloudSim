/// @file DrawingPageWidget.cpp
/// @brief 工程图页工具栏（出图选项）与画布

#include "DrawingPageWidget.h"

#include <QCheckBox>
#include <QComboBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

DrawingPageWidget::DrawingPageWidget(QWidget* parent) : QWidget(parent)
{
	auto* root = new QVBoxLayout(this);
	root->setContentsMargins(0, 0, 0, 0);
	root->setSpacing(0);

	auto* toolbar = new QWidget(this);
	auto* bar = new QHBoxLayout(toolbar);
	bar->setContentsMargins(8, 6, 8, 6);
	bar->setSpacing(6);

	m_generateBtn = new QPushButton(QStringLiteral("生成图纸"), toolbar);
	m_angleCombo = new QComboBox(toolbar);
	m_angleCombo->addItem(QStringLiteral("第一角法"), 0);
	m_angleCombo->addItem(QStringLiteral("第三角法"), 1);
	m_isoCheck = new QCheckBox(QStringLiteral("轴测"), toolbar);
	m_isoCheck->setChecked(false);
	m_sectionCheck = new QCheckBox(QStringLiteral("剖视"), toolbar);
	m_sectionPlaneCombo = new QComboBox(toolbar);
	m_sectionPlaneCombo->addItem(QStringLiteral("正视中面"), 0);
	m_sectionPlaneCombo->addItem(QStringLiteral("俯视中面"), 1);
	m_sectionPlaneCombo->addItem(QStringLiteral("右视中面"), 2);
	m_gridCheck = new QCheckBox(QStringLiteral("网格"), toolbar);
	m_gridCheck->setChecked(true);
	m_fitBtn = new QPushButton(QStringLiteral("适应窗口"), toolbar);
	m_svgBtn = new QPushButton(QStringLiteral("导出 SVG"), toolbar);
	m_dxfBtn = new QPushButton(QStringLiteral("导出 DXF"), toolbar);

	bar->addWidget(m_generateBtn);
	bar->addWidget(new QLabel(QStringLiteral("投影"), toolbar));
	bar->addWidget(m_angleCombo);
	bar->addWidget(m_isoCheck);
	bar->addWidget(m_sectionCheck);
	bar->addWidget(m_sectionPlaneCombo);
	bar->addWidget(m_gridCheck);
	bar->addWidget(m_fitBtn);
	bar->addWidget(m_svgBtn);
	bar->addWidget(m_dxfBtn);
	bar->addStretch(1);

	m_canvas = new DrawingSheetCanvasWidget(this);
	connect(m_gridCheck, &QCheckBox::toggled, m_canvas, &DrawingSheetCanvasWidget::setGridVisible);
	connect(m_fitBtn, &QPushButton::clicked, m_canvas, &DrawingSheetCanvasWidget::fitToView);
	connect(m_generateBtn, &QPushButton::clicked, this, &DrawingPageWidget::generateRequested);
	connect(m_svgBtn, &QPushButton::clicked, this, &DrawingPageWidget::exportSvgRequested);
	connect(m_dxfBtn, &QPushButton::clicked, this, &DrawingPageWidget::exportDxfRequested);
	connect(m_angleCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int) {
		if (m_canvas)
			m_canvas->setProjectionMethod(thirdAngle() ? DrawingProjectionMethod::ThirdAngle
													   : DrawingProjectionMethod::FirstAngle);
	});

	root->addWidget(toolbar);
	root->addWidget(m_canvas, 1);
}

bool DrawingPageWidget::includeIso() const
{
	return m_isoCheck && m_isoCheck->isChecked();
}

bool DrawingPageWidget::includeSection() const
{
	return m_sectionCheck && m_sectionCheck->isChecked();
}

int DrawingPageWidget::sectionPlane() const
{
	return m_sectionPlaneCombo ? m_sectionPlaneCombo->currentData().toInt() : 0;
}

bool DrawingPageWidget::thirdAngle() const
{
	return m_angleCombo && m_angleCombo->currentData().toInt() == 1;
}

void DrawingPageWidget::applyLanguage(bool useChinese)
{
	if (m_generateBtn)
		m_generateBtn->setText(useChinese ? QStringLiteral("生成图纸") : QStringLiteral("Generate"));
	if (m_isoCheck)
		m_isoCheck->setText(useChinese ? QStringLiteral("轴测") : QStringLiteral("Iso"));
	if (m_sectionCheck)
		m_sectionCheck->setText(useChinese ? QStringLiteral("剖视") : QStringLiteral("Section"));
	if (m_gridCheck)
		m_gridCheck->setText(useChinese ? QStringLiteral("网格") : QStringLiteral("Grid"));
	if (m_fitBtn)
		m_fitBtn->setText(useChinese ? QStringLiteral("适应窗口") : QStringLiteral("Fit"));
	if (m_svgBtn)
		m_svgBtn->setText(useChinese ? QStringLiteral("导出 SVG") : QStringLiteral("Export SVG"));
	if (m_dxfBtn)
		m_dxfBtn->setText(useChinese ? QStringLiteral("导出 DXF") : QStringLiteral("Export DXF"));
}
