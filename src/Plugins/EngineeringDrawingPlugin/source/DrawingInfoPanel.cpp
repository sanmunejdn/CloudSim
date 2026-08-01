/// @file DrawingInfoPanel.cpp
/// @brief 选中实体特性编辑

#include "DrawingInfoPanel.h"

#include <QCheckBox>
#include <QColorDialog>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSignalBlocker>
#include <QVBoxLayout>

DrawingInfoPanel::DrawingInfoPanel(QWidget* parent) : QWidget(parent)
{
	setWindowTitle(QStringLiteral("属性"));
	auto* root = new QVBoxLayout(this);
	root->setContentsMargins(8, 8, 8, 8);
	root->setSpacing(6);
	m_title = new QLabel(this);
	m_title->setObjectName(QStringLiteral("PropTitle"));
	root->addWidget(m_title);
	m_hint = new QLabel(this);
	m_hint->setWordWrap(true);
	root->addWidget(m_hint);

	auto* form = new QFormLayout;
	m_layerCombo = new QComboBox(this);
	form->addRow(QStringLiteral("图层"), m_layerCombo);
	m_colorByLayer = new QCheckBox(QStringLiteral("颜色 ByLayer"), this);
	m_colorByBlock = new QCheckBox(QStringLiteral("颜色 ByBlock"), this);
	m_colorBtn = new QPushButton(QStringLiteral("颜色…"), this);
	auto* colorRow = new QHBoxLayout;
	colorRow->addWidget(m_colorByLayer, 1);
	colorRow->addWidget(m_colorByBlock, 1);
	colorRow->addWidget(m_colorBtn);
	form->addRow(colorRow);
	m_ltByLayer = new QCheckBox(QStringLiteral("线型 ByLayer"), this);
	m_ltByBlock = new QCheckBox(QStringLiteral("线型 ByBlock"), this);
	m_lineTypeCombo = new QComboBox(this);
	m_lineTypeCombo->addItem(QStringLiteral("Continuous"), static_cast<int>(SheetLineType::Continuous));
	m_lineTypeCombo->addItem(QStringLiteral("Dashed"), static_cast<int>(SheetLineType::Dashed));
	m_lineTypeCombo->addItem(QStringLiteral("Center"), static_cast<int>(SheetLineType::Center));
	m_lineTypeCombo->addItem(QStringLiteral("DashDot"), static_cast<int>(SheetLineType::DashDot));
	form->addRow(m_ltByLayer);
	form->addRow(m_ltByBlock);
	form->addRow(QStringLiteral("线型"), m_lineTypeCombo);
	m_lwByLayer = new QCheckBox(QStringLiteral("线宽 ByLayer"), this);
	m_lwByBlock = new QCheckBox(QStringLiteral("线宽 ByBlock"), this);
	m_widthSpin = new QDoubleSpinBox(this);
	m_widthSpin->setRange(0.05, 5.0);
	m_widthSpin->setDecimals(2);
	m_widthSpin->setSingleStep(0.05);
	m_widthSpin->setSuffix(QStringLiteral(" mm"));
	form->addRow(m_lwByLayer);
	form->addRow(m_lwByBlock);
	form->addRow(QStringLiteral("线宽"), m_widthSpin);
	m_showTol = new QCheckBox(QStringLiteral("公差覆盖"), this);
	m_tolPlus = new QDoubleSpinBox(this);
	m_tolPlus->setRange(0.0, 100.0);
	m_tolPlus->setDecimals(3);
	m_tolMinus = new QDoubleSpinBox(this);
	m_tolMinus->setRange(0.0, 100.0);
	m_tolMinus->setDecimals(3);
	form->addRow(m_showTol);
	form->addRow(QStringLiteral("上偏差"), m_tolPlus);
	form->addRow(QStringLiteral("下偏差"), m_tolMinus);
	root->addLayout(form);
	m_matchBtn = new QPushButton(QStringLiteral("匹配特性"), this);
	root->addWidget(m_matchBtn);
	root->addStretch(1);

	applyLanguage(true);

	connect(m_layerCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int) {
		if (!m_busy)
			applyUiToSelection();
	});
	connect(m_colorByLayer, &QCheckBox::toggled, this, [this](bool on) {
		if (m_busy)
			return;
		if (on && m_colorByBlock)
		{
			const QSignalBlocker b(m_colorByBlock);
			m_colorByBlock->setChecked(false);
		}
		applyUiToSelection();
	});
	connect(m_colorByBlock, &QCheckBox::toggled, this, [this](bool on) {
		if (m_busy)
			return;
		if (on && m_colorByLayer)
		{
			const QSignalBlocker b(m_colorByLayer);
			m_colorByLayer->setChecked(false);
		}
		applyUiToSelection();
	});
	connect(m_ltByLayer, &QCheckBox::toggled, this, [this](bool on) {
		if (m_busy)
			return;
		if (on && m_ltByBlock)
		{
			const QSignalBlocker b(m_ltByBlock);
			m_ltByBlock->setChecked(false);
		}
		applyUiToSelection();
	});
	connect(m_ltByBlock, &QCheckBox::toggled, this, [this](bool on) {
		if (m_busy)
			return;
		if (on && m_ltByLayer)
		{
			const QSignalBlocker b(m_ltByLayer);
			m_ltByLayer->setChecked(false);
		}
		applyUiToSelection();
	});
	connect(m_lwByLayer, &QCheckBox::toggled, this, [this](bool on) {
		if (m_busy)
			return;
		if (on && m_lwByBlock)
		{
			const QSignalBlocker b(m_lwByBlock);
			m_lwByBlock->setChecked(false);
		}
		applyUiToSelection();
	});
	connect(m_lwByBlock, &QCheckBox::toggled, this, [this](bool on) {
		if (m_busy)
			return;
		if (on && m_lwByLayer)
		{
			const QSignalBlocker b(m_lwByLayer);
			m_lwByLayer->setChecked(false);
		}
		applyUiToSelection();
	});
	connect(m_lineTypeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int) {
		if (!m_busy)
			applyUiToSelection();
	});
	connect(m_widthSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double) {
		if (!m_busy)
			applyUiToSelection();
	});
	connect(m_showTol, &QCheckBox::toggled, this, [this](bool) {
		if (!m_busy)
			applyUiToSelection();
	});
	connect(m_tolPlus, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double) {
		if (!m_busy)
			applyUiToSelection();
	});
	connect(m_tolMinus, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double) {
		if (!m_busy)
			applyUiToSelection();
	});
	connect(m_colorBtn, &QPushButton::clicked, this, [this]() {
		if (!m_canvas)
			return;
		SheetEntityStyle s;
		QString lid;
		if (!m_canvas->selectionStyle(s, lid))
			return;
		const QColor c = QColorDialog::getColor(s.color, this, QStringLiteral("实体颜色"));
		if (!c.isValid())
			return;
		s.color = c;
		s.colorByLayer = false;
		m_canvas->applyStyleToSelection(s, lid);
		refreshFromSelection();
	});
	connect(m_matchBtn, &QPushButton::clicked, this, [this]() {
		if (m_canvas)
			m_canvas->matchPropFromSelection();
	});
	applyLanguage(true);
	refreshFromSelection();
}

void DrawingInfoPanel::bindCanvas(DrawingSheetCanvasWidget* canvas)
{
	if (m_canvas)
		disconnect(m_canvas, nullptr, this, nullptr);
	m_canvas = canvas;
	if (!m_canvas)
		return;
	connect(m_canvas, &DrawingSheetCanvasWidget::selectionChanged, this, &DrawingInfoPanel::refreshFromSelection);
	connect(m_canvas, &DrawingSheetCanvasWidget::layersChanged, this, &DrawingInfoPanel::refreshFromSelection);
	connect(m_canvas, &DrawingSheetCanvasWidget::sheetChanged, this, &DrawingInfoPanel::refreshFromSelection);
	refreshFromSelection();
}

void DrawingInfoPanel::applyLanguage(bool useChinese)
{
	m_useChinese = useChinese;
	setWindowTitle(useChinese ? QStringLiteral("属性") : QStringLiteral("Properties"));
	if (m_title)
		m_title->setText(useChinese ? QStringLiteral("属性") : QStringLiteral("Properties"));
	if (m_hint)
		m_hint->setText(useChinese ? QStringLiteral("选中视图/尺寸/注释后可改图层与实体覆盖。")
								   : QStringLiteral("Select an entity to edit layer / overrides."));
	if (m_matchBtn)
		m_matchBtn->setText(useChinese ? QStringLiteral("匹配特性") : QStringLiteral("Match Prop"));
}

void DrawingInfoPanel::refreshFromSelection()
{
	m_busy = true;
	const QSignalBlocker b1(m_layerCombo);
	const QSignalBlocker b2(m_lineTypeCombo);
	const QSignalBlocker b3(m_widthSpin);
	const QSignalBlocker b4(m_showTol);
	const QSignalBlocker b5(m_tolPlus);
	const QSignalBlocker b6(m_tolMinus);
	if (m_layerCombo)
		m_layerCombo->clear();
	if (!m_canvas)
	{
		m_busy = false;
		return;
	}
	for (const auto& L : m_canvas->layers())
		m_layerCombo->addItem(L.name, L.id);

	SheetEntityStyle s;
	QString lid;
	const bool ok = m_canvas->selectionStyle(s, lid);
	setEnabled(ok);
	if (!ok)
	{
		m_busy = false;
		return;
	}
	const int li = m_layerCombo->findData(lid);
	if (li >= 0)
		m_layerCombo->setCurrentIndex(li);
	m_colorByLayer->setChecked(s.colorByLayer);
	m_colorByBlock->setChecked(s.colorByBlock);
	m_ltByLayer->setChecked(s.lineTypeByLayer);
	m_ltByBlock->setChecked(s.lineTypeByBlock);
	m_lwByLayer->setChecked(s.lineWidthByLayer);
	m_lwByBlock->setChecked(s.lineWidthByBlock);
	const int ti = m_lineTypeCombo->findData(static_cast<int>(s.lineType));
	if (ti >= 0)
		m_lineTypeCombo->setCurrentIndex(ti);
	m_widthSpin->setValue(s.lineWidthMm);
	m_colorBtn->setEnabled(!s.colorByLayer && !s.colorByBlock);
	m_lineTypeCombo->setEnabled(!s.lineTypeByLayer && !s.lineTypeByBlock);
	m_widthSpin->setEnabled(!s.lineWidthByLayer && !s.lineWidthByBlock);
	const bool isDim = m_canvas->selectedDimIndex() >= 0;
	m_showTol->setEnabled(isDim);
	m_tolPlus->setEnabled(isDim);
	m_tolMinus->setEnabled(isDim);
	if (isDim)
	{
		const auto& d = m_canvas->dimensions().at(m_canvas->selectedDimIndex());
		m_showTol->setChecked(d.tolOverride ? d.showTolerance : false);
		m_tolPlus->setValue(d.tolOverride ? d.tolPlus : 0.0);
		m_tolMinus->setValue(d.tolOverride ? d.tolMinus : 0.0);
	}
	m_busy = false;
}

void DrawingInfoPanel::applyUiToSelection()
{
	if (!m_canvas || m_busy)
		return;
	SheetEntityStyle s;
	QString lid;
	if (!m_canvas->selectionStyle(s, lid))
		return;
	s.colorByLayer = m_colorByLayer->isChecked();
	s.lineTypeByLayer = m_ltByLayer->isChecked();
	s.lineWidthByLayer = m_lwByLayer->isChecked();
	s.colorByBlock = m_colorByBlock && m_colorByBlock->isChecked();
	s.lineTypeByBlock = m_ltByBlock && m_ltByBlock->isChecked();
	s.lineWidthByBlock = m_lwByBlock && m_lwByBlock->isChecked();
	s.lineType = static_cast<SheetLineType>(m_lineTypeCombo->currentData().toInt());
	s.lineWidthMm = m_widthSpin->value();
	const QString newLayer = m_layerCombo->currentData().toString();
	m_canvas->applyStyleToSelection(s, newLayer);
	m_colorBtn->setEnabled(!s.colorByLayer && !s.colorByBlock);
	m_lineTypeCombo->setEnabled(!s.lineTypeByLayer && !s.lineTypeByBlock);
	m_widthSpin->setEnabled(!s.lineWidthByLayer && !s.lineWidthByBlock);
	if (m_canvas->selectedDimIndex() >= 0)
		m_canvas->setSelectedDimTolerance(m_showTol->isChecked(), m_tolPlus->value(), m_tolMinus->value(), true);
}
