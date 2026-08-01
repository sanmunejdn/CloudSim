/// @file DimStyleDialog.cpp

#include "DimStyleDialog.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSpinBox>
#include <QVBoxLayout>

DimStyleDialog::DimStyleDialog(DrawingSheetCanvasWidget* canvas, QWidget* parent)
	: QDialog(parent), m_canvas(canvas)
{
	setWindowTitle(QStringLiteral("标注样式"));
	setMinimumWidth(320);
	auto* root = new QVBoxLayout(this);
	auto* form = new QFormLayout;
	m_styleCombo = new QComboBox(this);
	m_textH = new QDoubleSpinBox(this);
	m_textH->setRange(1.0, 20.0);
	m_textH->setDecimals(2);
	m_textH->setSuffix(QStringLiteral(" mm"));
	m_arrow = new QDoubleSpinBox(this);
	m_arrow->setRange(0.5, 15.0);
	m_arrow->setDecimals(2);
	m_arrow->setSuffix(QStringLiteral(" mm"));
	m_prec = new QSpinBox(this);
	m_prec->setRange(0, 6);
	m_showTol = new QCheckBox(QStringLiteral("显示公差"), this);
	m_tolPlus = new QDoubleSpinBox(this);
	m_tolPlus->setRange(0.0, 10.0);
	m_tolPlus->setDecimals(3);
	m_tolMinus = new QDoubleSpinBox(this);
	m_tolMinus->setRange(0.0, 10.0);
	m_tolMinus->setDecimals(3);
	form->addRow(QStringLiteral("样式"), m_styleCombo);
	form->addRow(QStringLiteral("文字高"), m_textH);
	form->addRow(QStringLiteral("箭头"), m_arrow);
	form->addRow(QStringLiteral("精度"), m_prec);
	form->addRow(m_showTol);
	form->addRow(QStringLiteral("上偏差"), m_tolPlus);
	form->addRow(QStringLiteral("下偏差"), m_tolMinus);
	root->addLayout(form);

	auto* btns = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
	auto* applyBtn = btns->addButton(QStringLiteral("设为当前"), QDialogButtonBox::ActionRole);
	root->addWidget(btns);
	connect(btns, &QDialogButtonBox::accepted, this, [this]() {
		applyToCanvas();
		accept();
	});
	connect(btns, &QDialogButtonBox::rejected, this, &QDialog::reject);
	connect(applyBtn, &QPushButton::clicked, this, [this]() { applyToCanvas(); });
	connect(m_styleCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int) { loadFromCanvas(); });
	loadFromCanvas();
}

void DimStyleDialog::loadFromCanvas()
{
	if (!m_canvas)
		return;
	const QString curId = m_styleCombo->currentData().toString();
	m_styleCombo->blockSignals(true);
	m_styleCombo->clear();
	for (const auto& s : m_canvas->dimStyles())
		m_styleCombo->addItem(s.name.isEmpty() ? s.id : s.name, s.id);
	int idx = m_styleCombo->findData(curId.isEmpty() ? m_canvas->currentDimStyleId() : curId);
	if (idx < 0)
		idx = m_styleCombo->findData(m_canvas->currentDimStyleId());
	if (idx >= 0)
		m_styleCombo->setCurrentIndex(idx);
	m_styleCombo->blockSignals(false);

	const QString id = m_styleCombo->currentData().toString();
	for (const auto& s : m_canvas->dimStyles())
	{
		if (s.id != id)
			continue;
		m_textH->setValue(s.textHeightMm);
		m_arrow->setValue(s.arrowSizeMm);
		m_prec->setValue(s.precision);
		m_showTol->setChecked(s.showTolerance);
		m_tolPlus->setValue(s.tolPlus);
		m_tolMinus->setValue(s.tolMinus);
		break;
	}
}

void DimStyleDialog::applyToCanvas()
{
	if (!m_canvas)
		return;
	DrawingSheetCanvasWidget::DimStyle s;
	s.id = m_styleCombo->currentData().toString();
	if (s.id.isEmpty())
		s.id = QStringLiteral("Standard");
	s.name = m_styleCombo->currentText();
	s.textHeightMm = m_textH->value();
	s.arrowSizeMm = m_arrow->value();
	s.precision = m_prec->value();
	s.showTolerance = m_showTol->isChecked();
	s.tolPlus = m_tolPlus->value();
	s.tolMinus = m_tolMinus->value();
	m_canvas->updateDimStyle(s);
	m_canvas->setCurrentDimStyleId(s.id);
}
