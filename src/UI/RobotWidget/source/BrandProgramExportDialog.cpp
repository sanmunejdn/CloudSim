/// @file BrandProgramExportDialog.cpp
/// @brief 品牌导出对话框

#include "BrandProgramExportDialog.h"

#include <QComboBox>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QVBoxLayout>

namespace RobotWidget
{

QVector<BrandExportChoice> BrandProgramExportDialog::allBrands()
{
	return {
		{QStringLiteral("abb"), QStringLiteral("ABBExport"), QStringLiteral(".MOD"),
		 QStringLiteral("ABB RAPID (*.MOD)")},
		{QStringLiteral("air"), QStringLiteral("AIRExport"), QStringLiteral(".arl"),
		 QStringLiteral("AIR ARL (*.arl)")},
		{QStringLiteral("fanuc"), QStringLiteral("FANUCExport"), QStringLiteral(".LS"),
		 QStringLiteral("FANUC LS (*.LS)")},
		{QStringLiteral("inovance"), QStringLiteral("INOVANCEExport"), QStringLiteral(".pro"),
		 QStringLiteral("INOVANCE PRO (*.pro)")},
		{QStringLiteral("lineheating"), QStringLiteral("LineHeatingExport"), QStringLiteral(".LS"),
		 QStringLiteral("LineHeating LS (*.LS)")},
		{QStringLiteral("rokae"), QStringLiteral("ROKAEExport"), QStringLiteral(".mod"),
		 QStringLiteral("ROKAE MOD (*.mod)")},
	};
}

bool BrandProgramExportDialog::findBrand(const QString& brandId, BrandExportChoice* out)
{
	for (const BrandExportChoice& b : allBrands())
	{
		if (b.brandId == brandId)
		{
			if (out)
			{
				*out = b;
			}
			return true;
		}
	}
	return false;
}

BrandProgramExportDialog::BrandProgramExportDialog(const QVector<BrandExportProgramItem>& programs,
												   const QString& activeProgramId, QWidget* parent)
	: QDialog(parent)
{
	setWindowTitle(QStringLiteral("导出机器人程序"));
	resize(400, 160);

	m_programCombo = new QComboBox(this);
	int activeIdx = 0;
	for (int i = 0; i < programs.size(); ++i)
	{
		const BrandExportProgramItem& prog = programs[i];
		m_programCombo->addItem(prog.name, prog.id);
		if (prog.id == activeProgramId)
		{
			activeIdx = i;
		}
	}
	if (!programs.isEmpty())
	{
		m_programCombo->setCurrentIndex(activeIdx);
	}

	m_brandCombo = new QComboBox(this);
	m_brandCombo->addItem(QStringLiteral("ABB"), QStringLiteral("abb"));
	m_brandCombo->addItem(QStringLiteral("AIR (配天)"), QStringLiteral("air"));
	m_brandCombo->addItem(QStringLiteral("FANUC"), QStringLiteral("fanuc"));
	m_brandCombo->addItem(QStringLiteral("汇川 INOVANCE"), QStringLiteral("inovance"));
	m_brandCombo->addItem(QStringLiteral("线加热 LineHeating"), QStringLiteral("lineheating"));
	m_brandCombo->addItem(QStringLiteral("珞石 ROKAE"), QStringLiteral("rokae"));

	auto* form = new QFormLayout();
	form->addRow(QStringLiteral("程序"), m_programCombo);
	form->addRow(QStringLiteral("机器人品牌"), m_brandCombo);

	auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
	connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
	connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);

	auto* root = new QVBoxLayout(this);
	root->addLayout(form);
	root->addWidget(buttons);
}

BrandExportChoice BrandProgramExportDialog::selectedBrand() const
{
	const QString id = m_brandCombo ? m_brandCombo->currentData().toString() : QString();
	BrandExportChoice choice;
	findBrand(id, &choice);
	return choice;
}

QString BrandProgramExportDialog::selectedProgramId() const
{
	if (!m_programCombo || m_programCombo->currentIndex() < 0)
	{
		return {};
	}
	return m_programCombo->currentData().toString();
}

QString BrandProgramExportDialog::selectedProgramName() const
{
	if (!m_programCombo || m_programCombo->currentIndex() < 0)
	{
		return {};
	}
	return m_programCombo->currentText();
}

} // namespace RobotWidget
