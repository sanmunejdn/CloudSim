/// @file AiConfirmPanel.cpp
/// @brief 按 Catalog args_schema 生成确认表单

#include "AiConfirmPanel.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QVBoxLayout>

#include <json.hpp>

namespace
{
QStringList objectIdsMatching(const nlohmann::json& snap, const QString& filter)
{
	QStringList ids;
	if (!snap.contains("objects") || !snap["objects"].is_array())
		return ids;
	for (const auto& o : snap["objects"])
	{
		const QString id = QString::fromStdString(o.value("id", ""));
		const QString cls = QString::fromStdString(o.value("class", ""));
		const QString name = QString::fromStdString(o.value("name", ""));
		const bool pc = cls.contains(QStringLiteral("PointCloud"), Qt::CaseInsensitive);
		const bool mesh = cls == QStringLiteral("Model");
		const bool brep = cls.contains(QStringLiteral("Brep"), Qt::CaseInsensitive);
		bool ok = true;
		if (filter == QStringLiteral("PointCloud"))
			ok = pc;
		else if (filter == QStringLiteral("Mesh"))
			ok = mesh;
		else if (filter == QStringLiteral("Brep"))
			ok = brep;
		else if (filter == QStringLiteral("PointCloudOrMesh"))
			ok = pc || mesh;
		else if (filter == QStringLiteral("BrepOrMesh"))
			ok = brep || mesh;
		if (ok && !id.isEmpty())
			ids << QStringLiteral("%1 [%2]").arg(name.isEmpty() ? id : name, id);
	}
	return ids;
}

QString idFromComboText(const QString& text)
{
	const int a = text.lastIndexOf(QLatin1Char('['));
	const int b = text.lastIndexOf(QLatin1Char(']'));
	if (a >= 0 && b > a)
		return text.mid(a + 1, b - a - 1);
	return text.trimmed();
}
} // namespace

AiConfirmPanel::AiConfirmPanel(QWidget* parent) : QWidget(parent)
{
	auto* root = new QVBoxLayout(this);
	root->setContentsMargins(0, 4, 0, 4);
	root->setSpacing(4);
	m_titleLabel = new QLabel(this);
	m_titleLabel->setWordWrap(true);
	m_riskLabel = new QLabel(this);
	m_riskLabel->setWordWrap(true);
	m_riskLabel->setStyleSheet(QStringLiteral("color: #b45309;"));
	m_formHost = new QWidget(this);
	m_form = new QFormLayout(m_formHost);
	m_form->setContentsMargins(0, 0, 0, 0);
	auto* btns = new QHBoxLayout();
	m_secondaryBtn = new QPushButton(this);
	m_secondaryBtn->hide();
	m_confirmBtn = new QPushButton(this);
	m_cancelBtn = new QPushButton(this);
	btns->addWidget(m_secondaryBtn);
	btns->addStretch(1);
	btns->addWidget(m_cancelBtn);
	btns->addWidget(m_confirmBtn);
	root->addWidget(m_titleLabel);
	root->addWidget(m_riskLabel);
	root->addWidget(m_formHost);
	root->addLayout(btns);
	connect(m_confirmBtn, &QPushButton::clicked, this,
			[this]()
			{
				emit accepted(m_pendingId, collectArgsJson());
				hidePanel();
			});
	connect(m_cancelBtn, &QPushButton::clicked, this,
			[this]()
			{
				const QString id = m_pendingId;
				hidePanel();
				emit rejected(id);
			});
	connect(m_secondaryBtn, &QPushButton::clicked, this,
			[this]()
			{
				const QString id = m_pendingId;
				hidePanel();
				emit secondaryClicked(id);
			});
	hidePanel();
	setUseChinese(true);
}

void AiConfirmPanel::setUseChinese(bool chinese)
{
	m_useChinese = chinese;
	m_defaultConfirmText = chinese ? QStringLiteral("确认执行") : QStringLiteral("Confirm");
	m_defaultCancelText = chinese ? QStringLiteral("取消") : QStringLiteral("Cancel");
	if (m_confirmBtn && m_confirmBtn->property("customLabel").toBool() == false)
		m_confirmBtn->setText(m_defaultConfirmText);
	if (m_cancelBtn)
		m_cancelBtn->setText(m_defaultCancelText);
}

void AiConfirmPanel::hidePanel()
{
	clearForm();
	m_pendingId.clear();
	if (m_secondaryBtn)
		m_secondaryBtn->hide();
	if (m_confirmBtn)
		m_confirmBtn->setProperty("customLabel", false);
	hide();
}

void AiConfirmPanel::showToolConfirm(const QString& pendingId, const QString& title, const QString& risk,
									 const QByteArray& argsSchemaJson, const QByteArray& proposedArgsJson,
									 const QByteArray& sceneSnapshotJson, const QString& confirmLabel,
									 const QString& secondaryLabel)
{
	m_pendingId = pendingId;
	m_titleLabel->setText(title);
	const bool high = risk.compare(QStringLiteral("high"), Qt::CaseInsensitive) == 0;
	const bool medium = risk.compare(QStringLiteral("medium"), Qt::CaseInsensitive) == 0;
	m_riskLabel->setVisible(high || medium);
	if (high)
	{
		m_riskLabel->setStyleSheet(QStringLiteral("color: #b91c1c; font-weight: 600;"));
		m_riskLabel->setText(m_useChinese ? QStringLiteral("高风险操作：请核对对象与路径后再确认。")
										  : QStringLiteral("High risk: review targets before confirm."));
	}
	else if (medium)
	{
		m_riskLabel->setStyleSheet(QStringLiteral("color: #b45309;"));
		m_riskLabel->setText(m_useChinese ? QStringLiteral("将修改场景或写入数据，请确认参数。")
										  : QStringLiteral("This will modify the scene; confirm parameters."));
	}
	else
		m_riskLabel->setText(QString());

	if (!confirmLabel.isEmpty())
	{
		m_confirmBtn->setText(confirmLabel);
		m_confirmBtn->setProperty("customLabel", true);
	}
	else
	{
		m_confirmBtn->setText(m_defaultConfirmText);
		m_confirmBtn->setProperty("customLabel", false);
	}
	m_cancelBtn->setText(m_defaultCancelText);
	if (!secondaryLabel.isEmpty())
	{
		m_secondaryBtn->setText(secondaryLabel);
		m_secondaryBtn->show();
	}
	else
		m_secondaryBtn->hide();

	rebuildForm(argsSchemaJson, proposedArgsJson, sceneSnapshotJson);
	show();
	updateConfirmEnabled();
}

void AiConfirmPanel::clearForm()
{
	m_fields.clear();
	while (m_form && m_form->rowCount() > 0)
		m_form->removeRow(0);
}

void AiConfirmPanel::rebuildForm(const QByteArray& argsSchemaJson, const QByteArray& proposedArgsJson,
								 const QByteArray& sceneSnapshotJson)
{
	clearForm();
	nlohmann::json schema = nlohmann::json::array();
	nlohmann::json proposed = nlohmann::json::object();
	nlohmann::json snap = nlohmann::json::object();
	try
	{
		if (!argsSchemaJson.isEmpty())
			schema = nlohmann::json::parse(argsSchemaJson.constData(), nullptr, true);
		if (!proposedArgsJson.isEmpty())
			proposed = nlohmann::json::parse(proposedArgsJson.constData(), nullptr, true);
		if (!sceneSnapshotJson.isEmpty())
			snap = nlohmann::json::parse(sceneSnapshotJson.constData(), nullptr, true);
	}
	catch (...)
	{
		return;
	}
	if (!schema.is_array())
		return;

	const QString selected = QString::fromStdString(snap.value("selected_backend_id", ""));

	for (const auto& field : schema)
	{
		if (!field.is_object())
			continue;
		const QString name = QString::fromStdString(field.value("name", ""));
		const QString type = QString::fromStdString(field.value("type", "string"));
		const QString label = QString::fromStdString(field.value("label", name.toStdString()));
		const bool required = field.value("required", false);
		FieldBind bind;
		bind.name = name;
		bind.type = type;
		bind.required = required;

		if (type == QStringLiteral("backend"))
		{
			auto* combo = new QComboBox(m_formHost);
			const QString filter = QString::fromStdString(field.value("filter", "Any"));
			const QStringList items = objectIdsMatching(snap, filter);
			combo->addItem(m_useChinese ? QStringLiteral("（请选择）") : QStringLiteral("(select)"), QString());
			for (const QString& it : items)
				combo->addItem(it, idFromComboText(it));
			QString pref;
			if (proposed.contains(name.toStdString()) && proposed[name.toStdString()].is_string())
				pref = QString::fromStdString(proposed[name.toStdString()].get<std::string>());
			if (pref.isEmpty())
				pref = selected;
			if (!pref.isEmpty())
			{
				const int idx = combo->findData(pref);
				if (idx >= 0)
					combo->setCurrentIndex(idx);
			}
			connect(combo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
					[this](int) { updateConfirmEnabled(); });
			bind.editor = combo;
			m_form->addRow(label, combo);
		}
		else if (type == QStringLiteral("backend_pair"))
		{
			auto* src = new QComboBox(m_formHost);
			auto* tgt = new QComboBox(m_formHost);
			const QString filter = QString::fromStdString(field.value("filter", "PointCloudOrMesh"));
			const QStringList items = objectIdsMatching(snap, filter);
			src->addItem(m_useChinese ? QStringLiteral("（源）") : QStringLiteral("(source)"), QString());
			tgt->addItem(m_useChinese ? QStringLiteral("（目标）") : QStringLiteral("(target)"), QString());
			for (const QString& it : items)
			{
				src->addItem(it, idFromComboText(it));
				tgt->addItem(it, idFromComboText(it));
			}
			auto pick = [&](QComboBox* c, const char* key)
			{
				if (proposed.contains(key) && proposed[key].is_string())
				{
					const QString v = QString::fromStdString(proposed[key].get<std::string>());
					const int idx = c->findData(v);
					if (idx >= 0)
						c->setCurrentIndex(idx);
				}
			};
			pick(src, "backend_id");
			pick(src, "source_backend_id");
			pick(tgt, "target_backend_id");
			connect(src, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
					[this](int) { updateConfirmEnabled(); });
			connect(tgt, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
					[this](int) { updateConfirmEnabled(); });
			bind.editor = src;
			bind.editor2 = tgt;
			const QString ls = QString::fromStdString(field.value("label_source", "源"));
			const QString lt = QString::fromStdString(field.value("label_target", "目标"));
			m_form->addRow(ls, src);
			m_form->addRow(lt, tgt);
		}
		else if (type == QStringLiteral("number"))
		{
			auto* spin = new QDoubleSpinBox(m_formHost);
			spin->setDecimals(4);
			spin->setRange(field.value("min", -1e12), field.value("max", 1e12));
			double v = field.value("default", 0.0);
			if (proposed.contains(name.toStdString()) && proposed[name.toStdString()].is_number())
				v = proposed[name.toStdString()].get<double>();
			spin->setValue(v);
			connect(spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
					[this](double) { updateConfirmEnabled(); });
			bind.editor = spin;
			m_form->addRow(label, spin);
		}
		else if (type == QStringLiteral("bool"))
		{
			auto* cb = new QCheckBox(m_formHost);
			bool v = field.value("default", false);
			if (proposed.contains(name.toStdString()) && proposed[name.toStdString()].is_boolean())
				v = proposed[name.toStdString()].get<bool>();
			cb->setChecked(v);
			bind.editor = cb;
			m_form->addRow(label, cb);
		}
		else if (type == QStringLiteral("enum"))
		{
			auto* combo = new QComboBox(m_formHost);
			QString cur = QString::fromStdString(field.value("default", ""));
			if (proposed.contains(name.toStdString()) && proposed[name.toStdString()].is_string())
				cur = QString::fromStdString(proposed[name.toStdString()].get<std::string>());
			if (field.contains("values") && field["values"].is_array())
			{
				for (const auto& v : field["values"])
				{
					if (!v.is_string())
						continue;
					const QString s = QString::fromStdString(v.get<std::string>());
					combo->addItem(s, s);
				}
			}
			const int idx = combo->findData(cur);
			if (idx >= 0)
				combo->setCurrentIndex(idx);
			bind.editor = combo;
			m_form->addRow(label, combo);
		}
		else if (type == QStringLiteral("file") || type == QStringLiteral("directory"))
		{
			auto* row = new QWidget(m_formHost);
			auto* lay = new QHBoxLayout(row);
			lay->setContentsMargins(0, 0, 0, 0);
			auto* edit = new QLineEdit(row);
			auto* browse = new QPushButton(m_useChinese ? QStringLiteral("浏览…") : QStringLiteral("Browse..."), row);
			if (proposed.contains(name.toStdString()) && proposed[name.toStdString()].is_string())
				edit->setText(QString::fromStdString(proposed[name.toStdString()].get<std::string>()));
			lay->addWidget(edit, 1);
			lay->addWidget(browse);
			const bool isDir = type == QStringLiteral("directory");
			connect(browse, &QPushButton::clicked, this,
					[this, edit, isDir]()
					{
						const QString p =
							isDir ? QFileDialog::getExistingDirectory(this, m_titleLabel->text())
								  : QFileDialog::getOpenFileName(this, m_titleLabel->text());
						if (!p.isEmpty())
						{
							edit->setText(p);
							updateConfirmEnabled();
						}
					});
			connect(edit, &QLineEdit::textChanged, this, [this](const QString&) { updateConfirmEnabled(); });
			bind.editor = edit;
			m_form->addRow(label, row);
		}
		else
		{
			auto* edit = new QLineEdit(m_formHost);
			if (proposed.contains(name.toStdString()) && proposed[name.toStdString()].is_string())
				edit->setText(QString::fromStdString(proposed[name.toStdString()].get<std::string>()));
			connect(edit, &QLineEdit::textChanged, this, [this](const QString&) { updateConfirmEnabled(); });
			bind.editor = edit;
			m_form->addRow(label, edit);
		}
		m_fields.push_back(bind);
	}
}

QByteArray AiConfirmPanel::collectArgsJson() const
{
	nlohmann::json args = nlohmann::json::object();
	for (const FieldBind& f : m_fields)
	{
		if (!f.editor)
			continue;
		if (f.type == QStringLiteral("backend"))
		{
			auto* c = qobject_cast<QComboBox*>(f.editor);
			if (c)
				args[f.name.toStdString()] = c->currentData().toString().toStdString();
		}
		else if (f.type == QStringLiteral("backend_pair"))
		{
			auto* src = qobject_cast<QComboBox*>(f.editor);
			auto* tgt = qobject_cast<QComboBox*>(f.editor2);
			if (src)
			{
				args["backend_id"] = src->currentData().toString().toStdString();
				args["source_backend_id"] = src->currentData().toString().toStdString();
			}
			if (tgt)
				args["target_backend_id"] = tgt->currentData().toString().toStdString();
		}
		else if (f.type == QStringLiteral("number"))
		{
			auto* s = qobject_cast<QDoubleSpinBox*>(f.editor);
			if (s)
				args[f.name.toStdString()] = s->value();
		}
		else if (f.type == QStringLiteral("bool"))
		{
			auto* c = qobject_cast<QCheckBox*>(f.editor);
			if (c)
				args[f.name.toStdString()] = c->isChecked();
		}
		else if (f.type == QStringLiteral("enum"))
		{
			auto* c = qobject_cast<QComboBox*>(f.editor);
			if (c)
				args[f.name.toStdString()] = c->currentData().toString().toStdString();
		}
		else if (f.type == QStringLiteral("file") || f.type == QStringLiteral("directory"))
		{
			if (auto* le = qobject_cast<QLineEdit*>(f.editor))
				args[f.name.toStdString()] = le->text().toStdString();
		}
		else if (auto* le = qobject_cast<QLineEdit*>(f.editor))
		{
			args[f.name.toStdString()] = le->text().toStdString();
		}
	}
	return QByteArray::fromStdString(args.dump());
}

void AiConfirmPanel::updateConfirmEnabled()
{
	bool ok = true;
	for (const FieldBind& f : m_fields)
	{
		if (!f.required || !f.editor)
			continue;
		if (f.type == QStringLiteral("backend"))
		{
			auto* c = qobject_cast<QComboBox*>(f.editor);
			if (!c || c->currentData().toString().isEmpty())
				ok = false;
		}
		else if (f.type == QStringLiteral("backend_pair"))
		{
			auto* src = qobject_cast<QComboBox*>(f.editor);
			auto* tgt = qobject_cast<QComboBox*>(f.editor2);
			if (!src || !tgt || src->currentData().toString().isEmpty() || tgt->currentData().toString().isEmpty())
				ok = false;
		}
		else if (f.type == QStringLiteral("file") || f.type == QStringLiteral("directory"))
		{
			auto* le = qobject_cast<QLineEdit*>(f.editor);
			if (!le || le->text().trimmed().isEmpty())
				ok = false;
		}
	}
	m_confirmBtn->setEnabled(ok);
}
