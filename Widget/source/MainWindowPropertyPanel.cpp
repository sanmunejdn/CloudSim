#include "MainWindow.h"

#include <memory>

#include <QList>
#include <QTreeWidget>
#include <QTreeWidgetItem>

#include <osg/Vec3f>

#include "BackendDataBase.h"
#include "BackendDataManager.h"
#include "BackendPropertyRow.h"
#include "MainWindow_p.h"
#include "MeshBackendData.h"
#include "PointCloudBackendData.h"
#include "OsgWidget.h"

#include "qttreepropertybrowser.h"
#include "qtvariantproperty.h"

using namespace mainwindow_detail;

namespace
{
bool propertyKeyUsesDoubleEditor(const QString& key)
{
	return key.startsWith(QStringLiteral("pose."))
		|| key.startsWith(QStringLiteral("rotation."))
		|| key.startsWith(QStringLiteral("color."))
		|| key.startsWith(QStringLiteral("motion.target.pose."))
		|| key.startsWith(QStringLiteral("motion.target.euler."))
		|| key == QStringLiteral("motion.speed")
		|| key == QStringLiteral("motion.acc")
		|| key == QStringLiteral("motion.blendRadius");
}
} // namespace

void MainWindow::appendPropertyBrowserRow(const QString& propertyKey, const QString& displayLabel, const QString& value, bool editable)
{
	if (!m_variantManager || !m_propertyBrowser)
	{
		return;
	}
	QtVariantProperty* prop = nullptr;
	const bool wantDouble = editable && propertyKeyUsesDoubleEditor(propertyKey);
	if (wantDouble)
	{
		bool ok = false;
		const double dv = value.toDouble(&ok);
		if (ok)
		{
			prop = m_variantManager->addProperty(QVariant::Double, displayLabel);
			m_variantManager->setAttribute(prop, QStringLiteral("decimals"), 6);
			m_variantManager->setValue(prop, dv);
		}
	}
	if (!prop)
	{
		prop = m_variantManager->addProperty(QVariant::String, displayLabel);
		m_variantManager->setValue(prop, value);
		if (!editable)
		{
			prop->setEnabled(false);
		}
	}
	prop->setWhatsThis(propertyKey);
	m_propertyBrowser->addProperty(prop);
}

QString MainWindow::propertyDisplayLabelForKey(const QString& key, const QString& labelEnFallback) const
{
	const auto tr = [this](const QString& en, const QString& zh) { return i18n(en, zh); };

	if (key == QStringLiteral("core.id"))
	{
		return tr(QStringLiteral("ID"), QStringLiteral("标识"));
	}
	if (key == QStringLiteral("core.name"))
	{
		return tr(QStringLiteral("Name"), QStringLiteral("名称"));
	}
	if (key == QStringLiteral("core.class"))
	{
		return tr(QStringLiteral("Class"), QStringLiteral("类型"));
	}
	if (key == QStringLiteral("ui.active_axis"))
	{
		return tr(QStringLiteral("Active axis"), QStringLiteral("活动轴"));
	}
	if (key == QStringLiteral("pose.x"))
	{
		return tr(QStringLiteral("Position X"), QStringLiteral("位置 X"));
	}
	if (key == QStringLiteral("pose.y"))
	{
		return tr(QStringLiteral("Position Y"), QStringLiteral("位置 Y"));
	}
	if (key == QStringLiteral("pose.z"))
	{
		return tr(QStringLiteral("Position Z"), QStringLiteral("位置 Z"));
	}
	if (key == QStringLiteral("rotation.x"))
	{
		return tr(QStringLiteral("Rotation X (°)"), QStringLiteral("旋转 X (°)"));
	}
	if (key == QStringLiteral("rotation.y"))
	{
		return tr(QStringLiteral("Rotation Y (°)"), QStringLiteral("旋转 Y (°)"));
	}
	if (key == QStringLiteral("rotation.z"))
	{
		return tr(QStringLiteral("Rotation Z (°)"), QStringLiteral("旋转 Z (°)"));
	}
	if (key == QStringLiteral("color.r"))
	{
		return tr(QStringLiteral("Color R"), QStringLiteral("颜色 R"));
	}
	if (key == QStringLiteral("color.g"))
	{
		return tr(QStringLiteral("Color G"), QStringLiteral("颜色 G"));
	}
	if (key == QStringLiteral("color.b"))
	{
		return tr(QStringLiteral("Color B"), QStringLiteral("颜色 B"));
	}
	if (key == QStringLiteral("color.a"))
	{
		return tr(QStringLiteral("Color A"), QStringLiteral("颜色 A"));
	}
	if (key == QStringLiteral("mesh.triangle_count"))
	{
		return tr(QStringLiteral("Triangle count"), QStringLiteral("三角形数"));
	}
	if (key == QStringLiteral("motion.target.pose.x"))
	{
		return tr(QStringLiteral("Target X (mm)"), QStringLiteral("目标 X (mm)"));
	}
	if (key == QStringLiteral("motion.target.pose.y"))
	{
		return tr(QStringLiteral("Target Y (mm)"), QStringLiteral("目标 Y (mm)"));
	}
	if (key == QStringLiteral("motion.target.pose.z"))
	{
		return tr(QStringLiteral("Target Z (mm)"), QStringLiteral("目标 Z (mm)"));
	}
	if (key == QStringLiteral("motion.target.euler.rx"))
	{
		return tr(QStringLiteral("Euler RX (deg)"), QStringLiteral("欧拉角 RX (deg)"));
	}
	if (key == QStringLiteral("motion.target.euler.ry"))
	{
		return tr(QStringLiteral("Euler RY (deg)"), QStringLiteral("欧拉角 RY (deg)"));
	}
	if (key == QStringLiteral("motion.target.euler.rz"))
	{
		return tr(QStringLiteral("Euler RZ (deg)"), QStringLiteral("欧拉角 RZ (deg)"));
	}
	if (key == QStringLiteral("motion.speed"))
	{
		return tr(QStringLiteral("Speed"), QStringLiteral("速度"));
	}
	if (key == QStringLiteral("motion.acc"))
	{
		return tr(QStringLiteral("Acceleration"), QStringLiteral("加速度"));
	}
	if (key == QStringLiteral("motion.axisConfig"))
	{
		return tr(QStringLiteral("Axis Configuration"), QStringLiteral("轴配置"));
	}
	if (key == QStringLiteral("motion.blendRadius"))
	{
		return tr(QStringLiteral("Blend Radius (mm)"), QStringLiteral("平滑半径 (mm)"));
	}
	// Unknown keys: show backend English label
	return labelEnFallback;
}

void MainWindow::updateInstructionPropertyPanel(const std::shared_ptr<RobotInstruction::Base>& instruction)
{
	if (!m_propertyBrowser || !m_variantManager)
	{
		return;
	}
	m_updatingPropertyBrowser = true;
	m_variantManager->clear();
	if (!instruction)
	{
		m_updatingPropertyBrowser = false;
		return;
	}

	appendPropertyBrowserRow(QStringLiteral("core.id"),
		propertyDisplayLabelForKey(QStringLiteral("core.id"), QStringLiteral("ID")),
		QString::fromStdString(instruction->id()), false);
	appendPropertyBrowserRow(QStringLiteral("core.name"),
		propertyDisplayLabelForKey(QStringLiteral("core.name"), QStringLiteral("Name")),
		QString::fromStdString(instruction->name()), false);

	const nlohmann::json rows = instruction->snapshotPropertyRows();
	if (rows.is_array())
	{
		for (const auto& r : rows)
		{
			if (!r.is_object())
			{
				continue;
			}
			const std::string keyStr = r.value(backend_property_json::kKey, std::string());
			if (keyStr.rfind("context.", 0) == 0
				|| keyStr.rfind("legacy.", 0) == 0
				|| keyStr == "motion.durationSec")
			{
				continue;
			}
			const std::string labelStr = r.value(backend_property_json::kLabelEn, std::string());
			const bool editable = r.value(backend_property_json::kEditable, false);
			const std::string valueStr = r.value(backend_property_json::kValue, std::string());
			const QString key = QString::fromStdString(keyStr);
			const QString label = propertyDisplayLabelForKey(key, QString::fromStdString(labelStr));
			appendPropertyBrowserRow(key, label, QString::fromStdString(valueStr), editable);
		}
	}

	m_updatingPropertyBrowser = false;
}

void MainWindow::syncOsgViewerFromPointCloudBackend(const std::shared_ptr<PointCloudBackendData>& pc)
{
	OsgWidget* osg = currentOsgWidget();
	if (!osg || !pc)
	{
		return;
	}
	const BackendVec3 pose = pc->pose();
	const BackendVec3 rot = pc->rotation();
	const BackendColor color = pc->color();
	osg->setSelectedPosition(osg::Vec3f(
		static_cast<float>(pose.x),
		static_cast<float>(pose.y),
		static_cast<float>(pose.z)));
	osg->setSelectedRotationEulerDeg(osg::Vec3f(
		static_cast<float>(rot.x),
		static_cast<float>(rot.y),
		static_cast<float>(rot.z)));
	osg->setSelectedColor(color.r, color.g, color.b, color.a);
	osg->update();
}

void MainWindow::syncOsgViewerFromMeshBackend(const std::shared_ptr<MeshBackendData>& mesh)
{
	OsgWidget* osg = currentOsgWidget();
	if (!osg || !mesh)
	{
		return;
	}
	const BackendVec3 pose = mesh->pose();
	const BackendVec3 rot = mesh->rotation();
	const BackendColor color = mesh->color();
	osg->setSelectedPosition(osg::Vec3f(
		static_cast<float>(pose.x),
		static_cast<float>(pose.y),
		static_cast<float>(pose.z)));
	osg->setSelectedRotationEulerDeg(osg::Vec3f(
		static_cast<float>(rot.x),
		static_cast<float>(rot.y),
		static_cast<float>(rot.z)));
	osg->setSelectedColor(color.r, color.g, color.b, color.a);
	osg->update();
}

void MainWindow::updatePropertyPanel(const std::shared_ptr<BackendDataBase>& data)
{
	if (!m_propertyBrowser || !m_variantManager)
	{
		return;
	}
	m_updatingPropertyBrowser = true;
	m_variantManager->clear();
	if (!data)
	{
		m_updatingPropertyBrowser = false;
		return;
	}

	const nlohmann::json rows = data->snapshotPropertyRows();
	if (!rows.is_array())
	{
		m_updatingPropertyBrowser = false;
		return;
	}
	for (const auto& r : rows)
	{
		if (!r.is_object())
		{
			continue;
		}
		const std::string keyStr = r.value(backend_property_json::kKey, std::string());
		const std::string labelStr = r.value(backend_property_json::kLabelEn, std::string());
		const bool editable = r.value(backend_property_json::kEditable, false);
		const std::string valueStr = r.value(backend_property_json::kValue, std::string());
		const QString key = QString::fromStdString(keyStr);
		const QString label = propertyDisplayLabelForKey(key, QString::fromStdString(labelStr));
		const QString val = QString::fromStdString(valueStr);
		appendPropertyBrowserRow(key, label, val, editable);
	}

	const bool showAxis = static_cast<bool>(std::dynamic_pointer_cast<PointCloudBackendData>(data));
	if (showAxis)
	{
		const QString axisKey = QStringLiteral("ui.active_axis");
		appendPropertyBrowserRow(
			axisKey,
			propertyDisplayLabelForKey(axisKey, QStringLiteral("Active axis")),
			m_activeAxisName,
			false);
	}

	m_updatingPropertyBrowser = false;
}

void MainWindow::onVariantPropertyValueChanged(QtProperty* property, const QVariant& value)
{
	if (m_updatingPropertyBrowser || !property)
	{
		return;
	}
	if (m_activeInstructionForProperty)
	{
		const QString propertyKey = property->whatsThis();
		if (propertyKey.isEmpty() || propertyKey.startsWith(QStringLiteral("core.")))
		{
			return;
		}
		QString valueText;
		if (value.type() == QVariant::Double)
		{
			valueText = QString::number(value.toDouble(), 'g', 12);
		}
		else
		{
			valueText = value.toString();
		}
		std::string err;
		if (!m_activeInstructionForProperty->applyPropertyChange(propertyKey.toStdString(), valueText.toStdString(), &err))
		{
			updateInstructionPropertyPanel(m_activeInstructionForProperty);
			return;
		}
		if (m_simulationCommandPage)
		{
			m_simulationCommandPage->refreshInstructionList();
		}
		refreshInstructionPoseAxes();
		updateInstructionPropertyPanel(m_activeInstructionForProperty);
		return;
	}
	if (!m_backendTree || !currentOsgWidget())
	{
		return;
	}

	const QList<QTreeWidgetItem*> selected = m_backendTree->selectedItems();
	if (selected.isEmpty() || selected.first() == m_backendRootItem)
	{
		return;
	}

	if (selected.first()->data(0, kRoleItemType).toInt() != kItemTypeBackend)
	{
		return;
	}
	const QString id = selected.first()->data(0, kRoleBackendId).toString();
	const auto data = activeBackend().getData(id.toStdString());
	if (!data)
	{
		return;
	}

	const QString propertyKey = property->whatsThis();
	if (propertyKey.isEmpty() || propertyKey.startsWith(QStringLiteral("ui.")))
	{
		return;
	}

	QString valueText;
	if (value.type() == QVariant::Double)
	{
		valueText = QString::number(value.toDouble(), 'g', 12);
	}
	else
	{
		valueText = value.toString();
	}

	const QByteArray keyBytes = propertyKey.toUtf8();
	const std::string keyUtf8(keyBytes.constData(), static_cast<std::size_t>(keyBytes.size()));
	const QByteArray valueBytes = valueText.toUtf8();
	const std::string valueUtf8(valueBytes.constData(), static_cast<std::size_t>(valueBytes.size()));

	std::string err;
	if (!data->applyPropertyChange(keyUtf8, valueUtf8, &err))
	{
		updatePropertyPanel(data);
		return;
	}

	if (auto pc = std::dynamic_pointer_cast<PointCloudBackendData>(data))
	{
		syncOsgViewerFromPointCloudBackend(pc);
	}
	else if (auto mesh = std::dynamic_pointer_cast<MeshBackendData>(data))
	{
		syncOsgViewerFromMeshBackend(mesh);
	}

	updatePropertyPanel(data);
}
