#ifndef ROBOTWIDGET_BRANDPROGRAMEXPORTDIALOG_H
#define ROBOTWIDGET_BRANDPROGRAMEXPORTDIALOG_H

/// @file BrandProgramExportDialog.h
/// @brief 品牌程序导出：选择程序与机器人品牌

#include "robotwidget_global.h"

#include <QDialog>
#include <QString>
#include <QVector>

class QComboBox;

namespace RobotWidget
{

struct BrandExportChoice
{
	QString brandId;	// abb / air / fanuc / inovance / lineheating / rokae
	QString scriptStem; // ABBExport
	QString defaultExt; // .MOD
	QString filter;		// 对话框过滤器
};

struct BrandExportProgramItem
{
	QString id;
	QString name;
};

class ROBOTWIDGET_EXPORT BrandProgramExportDialog : public QDialog
{
	Q_OBJECT
public:
	explicit BrandProgramExportDialog(const QVector<BrandExportProgramItem>& programs,
									  const QString& activeProgramId, QWidget* parent = nullptr);

	BrandExportChoice selectedBrand() const;
	QString selectedProgramId() const;
	QString selectedProgramName() const;

	static QVector<BrandExportChoice> allBrands();
	static bool findBrand(const QString& brandId, BrandExportChoice* out);

private:
	QComboBox* m_programCombo = nullptr;
	QComboBox* m_brandCombo = nullptr;
};

} // namespace RobotWidget

#endif
