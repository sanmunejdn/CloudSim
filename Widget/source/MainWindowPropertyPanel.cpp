#include "MainWindow.h"

#include <memory>
#include <sstream>
#include <iomanip>
#include <cmath>

#include <QList>
#include <QTreeWidget>
#include <QTreeWidgetItem>

#include <osg/Vec3f>
#include <osg/Matrixd>

#include "BackendDataBase.h"
#include "BackendDataManager.h"
#include "BackendPropertyRow.h"
#include "DocumentPage.h"
#include "BackendPropertySchema.h"
#include "MainWindow_p.h"
#include "MainWindowSelectionService.h"
#include "MeshBackendData.h"
#include "PointCloudBackendData.h"
#include "OsgWidget.h"
#include "RobotInstructionPropertySchema.h"
#include "RunLogger.h"

#include "../../PropertyCore/inc/PropertyTypes.h"

#include "qttreepropertybrowser.h"
#include "qtvariantproperty.h"

using namespace mainwindow_detail;

namespace
{
const property_core::PropertyDescriptor* instructionPropertyDescriptorForKey(const QString& key)
{
	const std::string keyStd = key.toStdString();
	return RobotInstruction::findInstructionPropertyDescriptor(keyStd);
}

const property_core::PropertyDescriptor* backendPropertyDescriptorForKey(const QString& key)
{
	const std::string keyStd = key.toStdString();
	return backend_property_schema::findAnyBackendPropertyDescriptor(keyStd);
}

const property_core::PropertyDescriptor* panelPropertyDescriptorForKey(const QString& key)
{
	if (const property_core::PropertyDescriptor* d = instructionPropertyDescriptorForKey(key))
	{
		return d;
	}
	return backendPropertyDescriptorForKey(key);
}

QString snapshotPropertyValueForKey(const nlohmann::json& rows, const QString& key)
{
	if (!rows.is_array() || key.isEmpty())
	{
		return {};
	}
	const std::string want = key.toStdString();
	for (const auto& r : rows)
	{
		if (!r.is_object())
		{
			continue;
		}
		if (r.value(backend_property_json::kKey, std::string()) == want)
		{
			return QString::fromStdString(r.value(backend_property_json::kValue, std::string()));
		}
	}
	return {};
}

int propertyEditorTypeForKey(const QString& key, bool editable)
{
	if (!editable)
	{
		return QVariant::String;
	}
	if (const property_core::PropertyDescriptor* descriptor = instructionPropertyDescriptorForKey(key))
	{
		switch (descriptor->type)
		{
		case property_core::PropertyType::Bool: return QVariant::Bool;
		case property_core::PropertyType::Int: return QVariant::Int;
		case property_core::PropertyType::Double: return QVariant::Double;
		default: return QVariant::String;
		}
	}
	if (const property_core::PropertyDescriptor* descriptor = backendPropertyDescriptorForKey(key))
	{
		switch (descriptor->type)
		{
		case property_core::PropertyType::Bool: return QVariant::Bool;
		case property_core::PropertyType::Int: return QVariant::Int;
		case property_core::PropertyType::Double: return QVariant::Double;
		default: return QVariant::String;
		}
	}
	return QVariant::String;
}

QString variantValueToString(const QVariant& value)
{
	switch (value.type())
	{
	case QVariant::Double:
		return QString::number(value.toDouble(), 'g', 12);
	case QVariant::Int:
		return QString::number(value.toInt());
	case QVariant::Bool:
		return value.toBool() ? QStringLiteral("true") : QStringLiteral("false");
	default:
		return value.toString();
	}
}

bool isPoseComponentKey(const QString& key)
{
	return key == QStringLiteral("pose.x") || key == QStringLiteral("pose.y") || key == QStringLiteral("pose.z");
}

bool isRotationComponentKey(const QString& key)
{
	return key == QStringLiteral("rotation.x") || key == QStringLiteral("rotation.y") || key == QStringLiteral("rotation.z");
}

bool updateVec3ComponentFromKey(const QString& key, const QString& valueText, BackendVec3& ioVec)
{
	bool ok = false;
	const double v = valueText.toDouble(&ok);
	if (!ok)
	{
		return false;
	}
	if (key.endsWith(QStringLiteral(".x")))
	{
		ioVec.x = v;
		return true;
	}
	if (key.endsWith(QStringLiteral(".y")))
	{
		ioVec.y = v;
		return true;
	}
	if (key.endsWith(QStringLiteral(".z")))
	{
		ioVec.z = v;
		return true;
	}
	return false;
}

std::string formatWorldMatrixCompact(const osg::Matrixd& m)
{
	std::ostringstream oss;
	oss << std::fixed << std::setprecision(6);
	for (int r = 0; r < 4; ++r)
	{
		oss << '[';
		for (int c = 0; c < 4; ++c)
		{
			if (c != 0)
			{
				oss << ", ";
			}
			oss << m(r, c);
		}
		oss << ']';
		if (r != 3)
		{
			oss << ' ';
		}
	}
	return oss.str();
}

bool matrixHasInvalidNumber(const osg::Matrixd& m)
{
	for (int r = 0; r < 4; ++r)
	{
		for (int c = 0; c < 4; ++c)
		{
			const double v = m(r, c);
			if (!std::isfinite(v))
			{
				return true;
			}
		}
	}
	return false;
}

void logBackendWorldMatrixAfterPropertyWrite(
	const char* sourceTag,
	const std::string& propertyKey,
	const std::string& propertyValue,
	OsgWidget* osg,
	const BackendDataBase& data,
	bool syncApplied)
{
	if (!osg)
	{
		return;
	}
	osg::Matrixd world;
	if (!osg->getBackendRootWorldMatrix(data.id(), world))
	{
		RunLogger::debug(std::string("[PropertySyncDBG] ") + sourceTag + " id=" + data.id()
			+ " key=" + propertyKey + " value=" + propertyValue
			+ " getBackendRootWorldMatrix failed; hasBranch="
			+ (osg->hasBackendObjectBranch(data.id()) ? "true" : "false")
			+ " syncApplied=" + (syncApplied ? "true" : "false"));
		RunLogger::flush();
		return;
	}
	const BackendVec3 p = data.pose();
	const BackendVec3 r = data.rotation();
	RunLogger::debug(std::string("[PropertySyncDBG] ") + sourceTag + " id=" + data.id()
		+ " key=" + propertyKey + " value=" + propertyValue
		+ " syncApplied=" + (syncApplied ? "true" : "false")
		+ " pose=(" + std::to_string(p.x) + "," + std::to_string(p.y) + "," + std::to_string(p.z) + ")"
		+ " euler=(" + std::to_string(r.x) + "," + std::to_string(r.y) + "," + std::to_string(r.z) + ")"
		+ " world=" + formatWorldMatrixCompact(world));
	if (matrixHasInvalidNumber(world))
	{
		RunLogger::error(std::string("[PropertySyncDBG] ") + sourceTag + " id=" + data.id()
			+ " world matrix contains NaN/Inf");
	}
	RunLogger::flush();
}
} // namespace

void MainWindow::appendPropertyBrowserRow(const QString& propertyKey, const QString& displayLabel, const QString& value, bool editable)
{
	if (!m_variantManager || !m_propertyBrowser)
	{
		return;
	}
	QtVariantProperty* prop = nullptr;
	const int editorType = propertyEditorTypeForKey(propertyKey, editable);
	if (editorType == QVariant::Double)
	{
		bool ok = false;
		const double dv = value.toDouble(&ok);
		if (ok)
		{
			prop = m_variantManager->addProperty(QVariant::Double, displayLabel);
			m_variantManager->setAttribute(prop, QStringLiteral("decimals"), 6);
			m_variantManager->setValue(prop, dv);
			if (const property_core::PropertyDescriptor* d = panelPropertyDescriptorForKey(propertyKey))
			{
				if (d->constraints.rangeDouble.enabled)
				{
					m_variantManager->setAttribute(prop, QStringLiteral("minimum"), d->constraints.rangeDouble.minValue);
					m_variantManager->setAttribute(prop, QStringLiteral("maximum"), d->constraints.rangeDouble.maxValue);
				}
			}
		}
	}
	else if (editorType == QVariant::Int)
	{
		bool ok = false;
		const int iv = value.toInt(&ok);
		if (ok)
		{
			prop = m_variantManager->addProperty(QVariant::Int, displayLabel);
			m_variantManager->setValue(prop, iv);
		}
	}
	else if (editorType == QVariant::Bool)
	{
		const QString lower = value.trimmed().toLower();
		if (lower == QStringLiteral("true")
			|| lower == QStringLiteral("false")
			|| lower == QStringLiteral("1")
			|| lower == QStringLiteral("0"))
		{
			prop = m_variantManager->addProperty(QVariant::Bool, displayLabel);
			m_variantManager->setValue(prop, lower == QStringLiteral("true") || lower == QStringLiteral("1"));
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
	if (key == QStringLiteral("pose.frame"))
	{
		return tr(QStringLiteral("Pose reference frame"), QStringLiteral("位姿参考系"));
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
	if (key == QStringLiteral("follow.targetName"))
	{
		return tr(QStringLiteral("Follow target (object name)"), QStringLiteral("跟随目标（对象名称）"));
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
	if (const property_core::PropertyDescriptor* descriptor = panelPropertyDescriptorForKey(key))
	{
		return QString::fromStdString(descriptor->label);
	}
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

void MainWindow::syncOsgViewerFromPointCloudBackend(const std::shared_ptr<PointCloudBackendData>& pc, bool applyColor)
{
	OsgWidget* osg = currentOsgWidget();
	if (!osg || !pc)
	{
		return;
	}
	const std::string backendId = pc->id();
	osg->syncSelectionForBackendId(backendId);
	if (!osg->hasBackendObjectBranch(backendId))
	{
		return;
	}
	// Use backend pose authority to avoid gizmo-frame side effects on nested branches.
	const bool synced = osg->syncOuterPatFromBackend(*pc);
	if (!synced)
	{
		RunLogger::debug(std::string("[PropertySyncDBG] PointCloud id=") + pc->id()
			+ " syncOuterPatFromBackend returned false");
		RunLogger::flush();
	}
	if (applyColor)
	{
		const BackendColor color = pc->color();
		osg->setSelectedColor(color.r, color.g, color.b, color.a);
	}
	osg->requestRedraw();
	logBackendWorldMatrixAfterPropertyWrite("PointCloud", "n/a", "n/a", osg, *pc, synced);
}

void MainWindow::syncOsgViewerFromMeshBackend(const std::shared_ptr<MeshBackendData>& mesh, bool applyColor)
{
	OsgWidget* osg = currentOsgWidget();
	if (!osg || !mesh)
	{
		return;
	}
	const std::string backendId = mesh->id();
	osg->syncSelectionForBackendId(backendId);
	if (!osg->hasBackendObjectBranch(backendId))
	{
		return;
	}
	// Keep world-matrix sync path consistent with follow/kinematics updates.
	const bool synced = osg->syncOuterPatFromBackend(*mesh);
	if (!synced)
	{
		RunLogger::debug(std::string("[PropertySyncDBG] Mesh id=") + mesh->id()
			+ " syncOuterPatFromBackend returned false");
		RunLogger::flush();
	}
	if (applyColor)
	{
		const BackendColor color = mesh->color();
		osg->setSelectedColor(color.r, color.g, color.b, color.a);
	}
	osg->requestRedraw();
	logBackendWorldMatrixAfterPropertyWrite("Mesh", "n/a", "n/a", osg, *mesh, synced);
}

void MainWindow::updatePropertyPanel(const std::shared_ptr<BackendDataBase>& data)
{
	if (!m_propertyBrowser || !m_variantManager)
	{
		return;
	}
	if (data && m_followTargetNameDebounceTimer.isActive()
		&& QString::fromStdString(data->id()) == m_followTargetNameDebounceBackendId)
	{
		return;
	}
	if (!data)
	{
		m_followTargetNameDebounceTimer.stop();
		m_followTargetNameDebounceBackendId.clear();
		m_followTargetNameDebounceText.clear();
	}
	else if (!m_followTargetNameDebounceBackendId.isEmpty()
		&& QString::fromStdString(data->id()) != m_followTargetNameDebounceBackendId)
	{
		m_followTargetNameDebounceTimer.stop();
		m_followTargetNameDebounceBackendId.clear();
		m_followTargetNameDebounceText.clear();
	}
	m_updatingPropertyBrowser = true;
	m_variantManager->clear();
	if (!data)
	{
		m_updatingPropertyBrowser = false;
		return;
	}

	DocumentPage* docPage = currentPage();
	const BackendDataManager* propMgr = docPage ? &docPage->backend() : nullptr;
	const nlohmann::json rows = data->snapshotPropertyRows(propMgr);
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
		bool editable = false;
		if (const auto itEd = r.find(backend_property_json::kEditable); itEd != r.end())
		{
			if (itEd->is_boolean())
			{
				editable = itEd->get<bool>();
			}
			else if (itEd->is_number_integer())
			{
				editable = (itEd->get<int>() != 0);
			}
		}
		if (keyStr == "follow.targetName")
		{
			editable = true;
		}
		const std::string valueStr = r.value(backend_property_json::kValue, std::string());
		const QString key = QString::fromStdString(keyStr);
		const QString label = propertyDisplayLabelForKey(key, QString::fromStdString(labelStr));
		QString val = QString::fromStdString(valueStr);
		if (isPoseComponentKey(key))
		{
			const BackendVec3 p = data->poseInFrame(data->poseReferenceFrame(), propMgr);
			if (key == QStringLiteral("pose.x")) val = QString::number(p.x, 'g', 12);
			if (key == QStringLiteral("pose.y")) val = QString::number(p.y, 'g', 12);
			if (key == QStringLiteral("pose.z")) val = QString::number(p.z, 'g', 12);
		}
		else if (isRotationComponentKey(key))
		{
			const BackendVec3 rFrame = data->rotationInFrame(data->poseReferenceFrame(), propMgr);
			if (key == QStringLiteral("rotation.x")) val = QString::number(rFrame.x, 'g', 12);
			if (key == QStringLiteral("rotation.y")) val = QString::number(rFrame.y, 'g', 12);
			if (key == QStringLiteral("rotation.z")) val = QString::number(rFrame.z, 'g', 12);
		}
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
		const QString valueText = variantValueToString(value);
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
	if (!m_selectionState.hasBackendSelection())
	{
		return;
	}
	const std::shared_ptr<BackendDataBase> data = MainWindowSelectionService::selectedBackendData(*this);
	if (!data)
	{
		return;
	}

	const QString propertyKey = property->whatsThis();
	if (propertyKey == QStringLiteral("follow.targetName"))
	{
		m_followTargetNameDebounceBackendId = QString::fromStdString(data->id());
		m_followTargetNameDebounceText = variantValueToString(value);
		m_followTargetNameDebounceTimer.start(400);
		return;
	}
	if (propertyKey.isEmpty() || propertyKey.startsWith(QStringLiteral("ui.")))
	{
		return;
	}
	const QString valueText = variantValueToString(value);

	const QByteArray keyBytes = propertyKey.toUtf8();
	const std::string keyUtf8(keyBytes.constData(), static_cast<std::size_t>(keyBytes.size()));
	const QByteArray valueBytes = valueText.toUtf8();
	const std::string valueUtf8(valueBytes.constData(), static_cast<std::size_t>(valueBytes.size()));

	DocumentPage* docPage = currentPage();
	BackendDataManager* propMgr = docPage ? &docPage->backend() : nullptr;

	const nlohmann::json rowsBefore = data->snapshotPropertyRows(propMgr);
	const QString oldValue = snapshotPropertyValueForKey(rowsBefore, propertyKey);

	std::string err;
	bool applyOk = false;
	if (isPoseComponentKey(propertyKey))
	{
		const BackendPoseReferenceFrame frame = data->poseReferenceFrame();
		BackendVec3 poseFrame = data->poseInFrame(frame, propMgr);
		if (!updateVec3ComponentFromKey(propertyKey, valueText, poseFrame))
		{
			updatePropertyPanel(data);
			return;
		}
		data->setPoseInFrame(poseFrame, frame, propMgr);
		applyOk = true;
	}
	else if (isRotationComponentKey(propertyKey))
	{
		const BackendPoseReferenceFrame frame = data->poseReferenceFrame();
		BackendVec3 rotFrame = data->rotationInFrame(frame, propMgr);
		if (!updateVec3ComponentFromKey(propertyKey, valueText, rotFrame))
		{
			updatePropertyPanel(data);
			return;
		}
		data->setRotationInFrame(rotFrame, frame, propMgr);
		applyOk = true;
	}
	else
	{
		applyOk = data->applyPropertyChange(keyUtf8, valueUtf8, &err, propMgr);
	}
	if (!applyOk)
	{
		RunLogger::debug(std::string("[PropertyCommitDBG] apply failed id=") + data->id()
			+ " key=" + keyUtf8
			+ " old=" + oldValue.toStdString()
			+ " new=" + valueUtf8
			+ " err=" + err);
		RunLogger::flush();
		updatePropertyPanel(data);
		return;
	}
	RunLogger::debug(std::string("[PropertyCommitDBG] apply ok id=") + data->id()
		+ " key=" + keyUtf8
		+ " old=" + oldValue.toStdString()
		+ " new=" + valueUtf8);
	RunLogger::flush();

	const property_core::PropertyDescriptor* backendDesc = backendPropertyDescriptorForKey(propertyKey);
	const quint32 semanticU = backendDesc != nullptr
		? property_core::semanticFlagsBits(backendDesc->semanticFlags)
		: property_core::semanticFlagsBits(property_core::PropertySemanticFlags::LegacyFullCommitBehavior);

	const bool followKey = propertyKey.startsWith(QStringLiteral("follow."));
	const bool legacy = (semanticU
			& property_core::semanticFlagsBits(property_core::PropertySemanticFlags::LegacyFullCommitBehavior))
		!= 0U;
	const bool needOsgSync = followKey || legacy
		|| (semanticU
			& property_core::semanticFlagsBits(property_core::PropertySemanticFlags::AffectsBackendRootWorldXform))
			!= 0U
		|| (semanticU & property_core::semanticFlagsBits(property_core::PropertySemanticFlags::AffectsColorOnly)) != 0U;
	RunLogger::debug(std::string("[PropertyCommitDBG] semantics id=") + data->id()
		+ " key=" + keyUtf8
		+ " semanticBits=" + std::to_string(static_cast<unsigned long long>(semanticU))
		+ " needOsgSync=" + (needOsgSync ? "true" : "false")
		+ " followKey=" + (followKey ? "true" : "false")
		+ " legacy=" + (legacy ? "true" : "false"));
	RunLogger::flush();

	if (followKey)
	{
		afterBackendFollowPropertyEdited(propertyKey, valueText);
	}
	else if (docPage && propMgr)
	{
		docPage->markFollowAttachmentDirtyFromBackendMove(*propMgr, data->id());
	}

	if (needOsgSync)
	{
		RunLogger::debug(std::string("[PropertySyncDBG] commit id=") + data->id()
			+ " key=" + keyUtf8 + " value=" + valueUtf8
			+ " needOsgSync=true followKey=" + (followKey ? "true" : "false"));
		RunLogger::flush();
		const bool applyColor = (semanticU
				& property_core::semanticFlagsBits(property_core::PropertySemanticFlags::AffectsColorOnly))
			!= 0U;
		if (auto pc = std::dynamic_pointer_cast<PointCloudBackendData>(data))
		{
			syncOsgViewerFromPointCloudBackend(pc, applyColor);
		}
		else if (auto mesh = std::dynamic_pointer_cast<MeshBackendData>(data))
		{
			syncOsgViewerFromMeshBackend(mesh, applyColor);
		}
	}

	schedulePropertyPanelCommitRefresh(data);

	emit backendPropertyCommitted(QString::fromStdString(data->id()), propertyKey, oldValue, valueText, semanticU);
}

void MainWindow::flushFollowTargetNamePropertyEdit()
{
	if (!m_backendTree || !currentOsgWidget() || !m_selectionState.hasBackendSelection())
	{
		m_followTargetNameDebounceBackendId.clear();
		m_followTargetNameDebounceText.clear();
		return;
	}
	const QString selId = m_selectionState.selectedBackendId();
	if (selId != m_followTargetNameDebounceBackendId)
	{
		m_followTargetNameDebounceBackendId.clear();
		m_followTargetNameDebounceText.clear();
		return;
	}
	const std::shared_ptr<BackendDataBase> data = MainWindowSelectionService::selectedBackendData(*this);
	if (!data || QString::fromStdString(data->id()) != m_followTargetNameDebounceBackendId)
	{
		m_followTargetNameDebounceBackendId.clear();
		m_followTargetNameDebounceText.clear();
		return;
	}

	DocumentPage* docPage = currentPage();
	BackendDataManager* propMgr = docPage ? &docPage->backend() : nullptr;

	const nlohmann::json rowsBefore = data->snapshotPropertyRows(propMgr);
	const QString propertyKey = QStringLiteral("follow.targetName");
	const QString oldValue = snapshotPropertyValueForKey(rowsBefore, propertyKey);

	const QByteArray keyBytes = propertyKey.toUtf8();
	const std::string keyUtf8(keyBytes.constData(), static_cast<std::size_t>(keyBytes.size()));
	const QByteArray valueBytes = m_followTargetNameDebounceText.toUtf8();
	const std::string valueUtf8(valueBytes.constData(), static_cast<std::size_t>(valueBytes.size()));

	std::string err;
	if (!data->applyPropertyChange(keyUtf8, valueUtf8, &err, propMgr))
	{
		m_followTargetNameDebounceBackendId.clear();
		updatePropertyPanel(data);
		return;
	}

	const property_core::PropertyDescriptor* backendDesc = backendPropertyDescriptorForKey(propertyKey);
	const quint32 semanticU = backendDesc != nullptr
		? property_core::semanticFlagsBits(backendDesc->semanticFlags)
		: property_core::semanticFlagsBits(property_core::PropertySemanticFlags::LegacyFullCommitBehavior);

	afterBackendFollowPropertyEdited(propertyKey, m_followTargetNameDebounceText);
	if (auto pc = std::dynamic_pointer_cast<PointCloudBackendData>(data))
	{
		syncOsgViewerFromPointCloudBackend(pc, false);
	}
	else if (auto mesh = std::dynamic_pointer_cast<MeshBackendData>(data))
	{
		syncOsgViewerFromMeshBackend(mesh, false);
	}

	emit backendPropertyCommitted(QString::fromStdString(data->id()), propertyKey, oldValue, m_followTargetNameDebounceText, semanticU);

	m_followTargetNameDebounceBackendId.clear();
	m_followTargetNameDebounceText.clear();
	updatePropertyPanel(data);
}
