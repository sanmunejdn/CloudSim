/// @file AiAgentPickDialog.cpp
/// @brief AI Agent 缺参时的选择对话框

#include "Ai/AiAgentPickDialog.h"

#include "BackendTypeIds.h"
#include "IPluginDocument.h"

#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QLabel>
#include <QLatin1String>
#include <QVBoxLayout>

namespace AiAgentPickDialog
{
namespace
{
bool matchFilter(const QString& className, BackendKindFilter filter)
{
	const std::string cn = className.toStdString();
	const bool pc = backend_type::isPointCloudClassName(cn);
	const bool mesh = backend_type::isMeshClassName(cn);
	const bool brep = backend_type::isBrepWorkpieceClassName(cn);
	switch (filter)
	{
	case BackendKindFilter::Any:
		return true;
	case BackendKindFilter::PointCloud:
		return pc;
	case BackendKindFilter::Mesh:
		return mesh;
	case BackendKindFilter::Brep:
		return brep;
	case BackendKindFilter::PointCloudOrMesh:
		return pc || mesh;
	case BackendKindFilter::BrepOrMesh:
		return brep || mesh;
	}
	return true;
}
} // namespace

std::vector<BackendEntry> listBackends(IPluginDocument* doc, BackendKindFilter filter)
{
	std::vector<BackendEntry> out;
	if (!doc)
		return out;
	for (const std::string& id : doc->backendIds())
	{
		BackendEntry e;
		e.id = QString::fromStdString(id);
		e.className = QString::fromStdString(doc->backendClassName(id));
		if (!matchFilter(e.className, filter))
			continue;
		const QString name = QString::fromStdString(doc->backendDisplayName(id));
		e.label = QStringLiteral("%1 [%2] (%3)").arg(name, e.id, e.className);
		out.push_back(std::move(e));
	}
	return out;
}

bool pickOneBackend(QWidget* parent, const std::vector<BackendEntry>& entries, const QString& title, QString* outId)
{
	if (!outId || entries.empty())
		return false;
	QDialog dlg(parent);
	dlg.setWindowTitle(title);
	auto* layout = new QVBoxLayout(&dlg);
	layout->addWidget(new QLabel(QStringLiteral("请选择对象："), &dlg));
	auto* combo = new QComboBox(&dlg);
	for (const auto& e : entries)
		combo->addItem(e.label, e.id);
	layout->addWidget(combo);
	auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
	layout->addWidget(buttons);
	QObject::connect(buttons, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
	QObject::connect(buttons, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
	if (dlg.exec() != QDialog::Accepted)
		return false;
	*outId = combo->currentData().toString();
	return !outId->isEmpty();
}

bool pickSourceAndTarget(QWidget* parent, const std::vector<BackendEntry>& entries, const QString& title,
						 QString* outSourceId, QString* outTargetId)
{
	if (!outSourceId || !outTargetId || entries.size() < 2)
		return false;
	QDialog dlg(parent);
	dlg.setWindowTitle(title);
	auto* layout = new QVBoxLayout(&dlg);
	layout->addWidget(new QLabel(QStringLiteral("请选择源与目标对象："), &dlg));
	auto* form = new QFormLayout();
	auto* src = new QComboBox(&dlg);
	auto* tgt = new QComboBox(&dlg);
	for (const auto& e : entries)
	{
		src->addItem(e.label, e.id);
		tgt->addItem(e.label, e.id);
	}
	if (tgt->count() > 1)
		tgt->setCurrentIndex(1);
	form->addRow(QStringLiteral("源 (Source)"), src);
	form->addRow(QStringLiteral("目标 (Target)"), tgt);
	layout->addLayout(form);
	auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
	layout->addWidget(buttons);
	QObject::connect(buttons, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
	QObject::connect(buttons, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
	if (dlg.exec() != QDialog::Accepted)
		return false;
	*outSourceId = src->currentData().toString();
	*outTargetId = tgt->currentData().toString();
	if (outSourceId->isEmpty() || outTargetId->isEmpty() || *outSourceId == *outTargetId)
		return false;
	return true;
}

bool pickOpenFilePath(QWidget* parent, const QString& title, const QString& filter, QString* outPath)
{
	if (!outPath)
		return false;
	const QString path = QFileDialog::getOpenFileName(parent, title, QString(), filter);
	if (path.isEmpty())
		return false;
	*outPath = path;
	return true;
}

bool pickSaveFilePath(QWidget* parent, const QString& title, const QString& filter, const QString& defaultName,
					  QString* outPath)
{
	if (!outPath)
		return false;
	const QString path = QFileDialog::getSaveFileName(parent, title, defaultName, filter);
	if (path.isEmpty())
		return false;
	*outPath = path;
	return true;
}

bool pickExistingDirectory(QWidget* parent, const QString& title, QString* outDir)
{
	if (!outDir)
		return false;
	const QString dir = QFileDialog::getExistingDirectory(parent, title);
	if (dir.isEmpty())
		return false;
	*outDir = dir;
	return true;
}
} // namespace AiAgentPickDialog
