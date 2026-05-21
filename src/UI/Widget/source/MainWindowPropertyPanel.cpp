#include "MainWindow.h"

#include <algorithm>
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
#include "RobotCoordinateFrames.h"
#include "RobotInstructionPropertySchema.h"
#include "RobotInstructionProgram.h"
#include "RunLogger.h"
#include "../RobotWidget/inc/RobotInstructionPlanningHelpers.h"
#include "../RobotWidget/inc/SimulationCommandWidget.h"

#include "../../Data/PropertyCore/inc/PropertyTypes.h"

#include "qttreepropertybrowser.h"
#include "qtvariantproperty.h"

using namespace mainwindow_detail;

namespace
{
bool instructionUsesActiveUserFrame(const RobotInstruction::Base& ins)
{
	return RobotCoordinate::instructionTargetDisplayUsesUserFrame(ins.extensionProperties());
}

void instructionTcpInBase(const RobotInstruction::Base& ins, BackendMat4& out)
{
	const RobotInstruction::Vec3 p = ins.pose();
	const RobotInstruction::Vec3 e = ins.eulerDeg();
	out = RobotCoordinate::tcpInBaseFromPose(p.x, p.y, p.z, e.x, e.y, e.z);
}

void setInstructionTcpInBase(RobotInstruction::Base& ins, const BackendMat4& T_base_tcp)
{
	double pos[3]{};
	double euler[3]{};
	RobotCoordinate::poseEulerFromTcpInBase(T_base_tcp, pos, euler);
	RobotInstruction::Vec3 p{};
	p.x = pos[0];
	p.y = pos[1];
	p.z = pos[2];
	ins.setPose(p);
	RobotInstruction::Vec3 e{};
	e.x = euler[0];
	e.y = euler[1];
	e.z = euler[2];
	ins.setEulerDeg(e);
}

BackendMat4 instructionTcpForDisplay(const MainWindow& mw, const RobotInstruction::Base& ins)
{
	const BackendMat4 T_base_tcp = [&]() {
		BackendMat4 t{};
		instructionTcpInBase(ins, t);
		return t;
	}();
	if (!instructionUsesActiveUserFrame(ins))
	{
		return T_base_tcp;
	}
	DocumentPage* doc = mw.currentPage();
	if (!doc)
	{
		return T_base_tcp;
	}
	const int instIdx = mw.currentSimulationRobotInstanceIndex();
	if (instIdx < 0)
	{
		return T_base_tcp;
	}
	const RobotCoordinate::RobotCoordinateFrameSet& frames = doc->robotCoordinateFramesForInstance(instIdx);
	const RobotCoordinate::RobotUserFrame* uf =
		RobotCoordinate::resolveUserFrameForExtension(frames, ins.extensionProperties());
	if (!uf)
	{
		return T_base_tcp;
	}
	const BackendMat4 T_base_user = RobotCoordinate::frameToMat4(uf->T_base_user);
	return RobotCoordinate::tcpInUserFromBaseTcp(T_base_user, T_base_tcp);
}

bool isMotionTargetPoseKey(const QString& key)
{
	return key == QStringLiteral("motion.target.pose.x") || key == QStringLiteral("motion.target.pose.y")
		|| key == QStringLiteral("motion.target.pose.z") || key == QStringLiteral("motion.target.euler.rx")
		|| key == QStringLiteral("motion.target.euler.ry") || key == QStringLiteral("motion.target.euler.rz");
}

bool isInstructionPanelManagedExtensionKey(const std::string& keyStr)
{
	if (keyStr == RobotCoordinate::kExtMotionToolFrameId || keyStr == RobotCoordinate::kExtMotionUserFrameId
		|| keyStr == RobotCoordinate::kExtMotionTargetFrame)
	{
		return true;
	}
	if (keyStr.rfind("render.", 0) == 0 || keyStr.rfind("context.", 0) == 0)
	{
		return true;
	}
	return false;
}
} // namespace

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
		case property_core::PropertyType::Enum: return QtVariantPropertyManager::enumTypeId();
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
	if (key == QStringLiteral("motion.tool.frameId") || key == QStringLiteral("motion.user.frameId")
		|| key == QStringLiteral("motion.target.frame"))
	{
		return QtVariantPropertyManager::enumTypeId();
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

QString instructionEnumTokenFromValue(const QString& propertyKey, const QVariant& value)
{
	const property_core::PropertyDescriptor* descriptor = instructionPropertyDescriptorForKey(propertyKey);
	if (!descriptor || descriptor->type != property_core::PropertyType::Enum)
	{
		return variantValueToString(value);
	}
	const int idx = value.toInt();
	if (idx >= 0 && idx < static_cast<int>(descriptor->constraints.enumConstraint.options.size()))
	{
		return QString::fromStdString(descriptor->constraints.enumConstraint.options[static_cast<size_t>(idx)]);
	}
	return variantValueToString(value);
}

QString axisConfigEnumDisplayName(const QString& propertyKey, const QString& token, bool chinese)
{
	if (propertyKey == QStringLiteral("motion.axisConfig.preset"))
	{
		if (token == QStringLiteral("AUTO")) return chinese ? QStringLiteral("自动") : QStringLiteral("Auto");
		if (token == QStringLiteral("ELBOW_UP")) return chinese ? QStringLiteral("肘上") : QStringLiteral("Elbow up");
		if (token == QStringLiteral("ELBOW_DOWN")) return chinese ? QStringLiteral("肘下") : QStringLiteral("Elbow down");
		if (token == QStringLiteral("WRIST_FLIP")) return chinese ? QStringLiteral("腕翻") : QStringLiteral("Wrist flip");
		if (token == QStringLiteral("WRIST_NO_FLIP")) return chinese ? QStringLiteral("腕不翻") : QStringLiteral("Wrist no-flip");
		if (token == QStringLiteral("ELBOW_UP_WRIST_NO_FLIP")) return chinese ? QStringLiteral("肘上/腕不翻") : QStringLiteral("Elbow up, wrist no-flip");
		if (token == QStringLiteral("ELBOW_UP_WRIST_FLIP")) return chinese ? QStringLiteral("肘上/腕翻") : QStringLiteral("Elbow up, wrist flip");
		if (token == QStringLiteral("ELBOW_DOWN_WRIST_NO_FLIP")) return chinese ? QStringLiteral("肘下/腕不翻") : QStringLiteral("Elbow down, wrist no-flip");
		if (token == QStringLiteral("ELBOW_DOWN_WRIST_FLIP")) return chinese ? QStringLiteral("肘下/腕翻") : QStringLiteral("Elbow down, wrist flip");
		if (token == QStringLiteral("CUSTOM")) return chinese ? QStringLiteral("自定义") : QStringLiteral("Custom");
	}
	if (propertyKey == QStringLiteral("motion.axisConfig.elbow"))
	{
		if (token == QStringLiteral("AUTO")) return chinese ? QStringLiteral("自动") : QStringLiteral("Auto");
		if (token == QStringLiteral("UP")) return chinese ? QStringLiteral("肘上") : QStringLiteral("Up");
		if (token == QStringLiteral("DOWN")) return chinese ? QStringLiteral("肘下") : QStringLiteral("Down");
	}
	if (propertyKey == QStringLiteral("motion.axisConfig.wrist"))
	{
		if (token == QStringLiteral("AUTO")) return chinese ? QStringLiteral("自动") : QStringLiteral("Auto");
		if (token == QStringLiteral("NO_FLIP")) return chinese ? QStringLiteral("腕不翻") : QStringLiteral("No flip");
		if (token == QStringLiteral("FLIP")) return chinese ? QStringLiteral("腕翻") : QStringLiteral("Flip");
	}
	if (propertyKey == QStringLiteral("motion.axisConfig.arm"))
	{
		if (token == QStringLiteral("AUTO")) return chinese ? QStringLiteral("自动") : QStringLiteral("Auto");
		if (token == QStringLiteral("FRONT")) return chinese ? QStringLiteral("臂前") : QStringLiteral("Front");
		if (token == QStringLiteral("BACK")) return chinese ? QStringLiteral("臂后") : QStringLiteral("Back");
	}
	if (propertyKey == QStringLiteral("motion.axisConfig.turn.j1")
		|| propertyKey == QStringLiteral("motion.axisConfig.turn.j4")
		|| propertyKey == QStringLiteral("motion.axisConfig.turn.j6"))
	{
		if (token == QStringLiteral("AUTO")) return chinese ? QStringLiteral("自动") : QStringLiteral("Auto");
		return chinese ? QStringLiteral("转 %1").arg(token) : QStringLiteral("Turn %1").arg(token);
	}
	return token;
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

void MainWindow::appendPropertyBrowserRow(
	const QString& propertyKey,
	const QString& displayLabel,
	const QString& value,
	bool editable,
	const std::vector<std::string>* enumOptionTokens,
	const QStringList* enumDisplayNames,
	const QString& toolTip)
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
	else if (editorType == QtVariantPropertyManager::enumTypeId())
	{
		const property_core::PropertyDescriptor* descriptor = instructionPropertyDescriptorForKey(propertyKey);
		const std::vector<std::string>* optionsPtr = nullptr;
		std::vector<std::string> descriptorOptions;
		if (enumOptionTokens && !enumOptionTokens->empty())
		{
			optionsPtr = enumOptionTokens;
		}
		else if (descriptor && descriptor->type == property_core::PropertyType::Enum)
		{
			descriptorOptions = descriptor->constraints.enumConstraint.options;
			optionsPtr = &descriptorOptions;
		}
		if (optionsPtr && !optionsPtr->empty())
		{
			const std::vector<std::string>& options = *optionsPtr;
			QStringList enumNames;
			int selectedIndex = 0;
			const QString valueTrim = value.trimmed();
			for (size_t i = 0; i < options.size(); ++i)
			{
				const QString token = QString::fromStdString(options[i]);
				if (enumDisplayNames && static_cast<int>(i) < enumDisplayNames->size()
					&& !enumDisplayNames->at(static_cast<int>(i)).isEmpty())
				{
					enumNames << enumDisplayNames->at(static_cast<int>(i));
				}
				else
				{
					enumNames << axisConfigEnumDisplayName(propertyKey, token, m_useChinese);
				}
				if (token.compare(valueTrim, Qt::CaseInsensitive) == 0)
				{
					selectedIndex = static_cast<int>(i);
				}
			}
			if (!enumNames.isEmpty())
			{
				if (selectedIndex >= enumNames.size())
				{
					selectedIndex = 0;
				}
				prop = m_variantManager->addProperty(QtVariantPropertyManager::enumTypeId(), displayLabel);
				m_variantManager->setAttribute(prop, QStringLiteral("enumNames"), enumNames);
				m_variantManager->setValue(prop, selectedIndex);
				QStringList tokenList;
				tokenList.reserve(static_cast<int>(options.size()));
				for (const std::string& opt : options)
				{
					tokenList << QString::fromStdString(opt);
				}
				m_propertyEnumTokens.insert(prop, tokenList);
			}
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
	if (!toolTip.isEmpty())
	{
		prop->setToolTip(toolTip);
	}
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
	if (key == QStringLiteral("motion.target.frame"))
	{
		return tr(QStringLiteral("Target frame"), QStringLiteral("目标坐标系"));
	}
	if (key == QStringLiteral("motion.tool.frameId"))
	{
		return tr(QStringLiteral("Tool frame"), QStringLiteral("工具坐标系"));
	}
	if (key == QStringLiteral("motion.user.frameId"))
	{
		return tr(QStringLiteral("User frame"), QStringLiteral("用户坐标系"));
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
	if (key == QStringLiteral("motion.axisConfig") || key == QStringLiteral("motion.axisConfig.preset"))
	{
		return tr(QStringLiteral("Axis config preset"), QStringLiteral("轴配置预设"));
	}
	if (key == QStringLiteral("motion.axisConfig.elbow"))
	{
		return tr(QStringLiteral("Elbow posture"), QStringLiteral("肘部姿态"));
	}
	if (key == QStringLiteral("motion.axisConfig.wrist"))
	{
		return tr(QStringLiteral("Wrist posture"), QStringLiteral("腕部姿态"));
	}
	if (key == QStringLiteral("motion.axisConfig.arm"))
	{
		return tr(QStringLiteral("Arm posture"), QStringLiteral("臂形前后"));
	}
	if (key == QStringLiteral("motion.axisConfig.turn.j1"))
	{
		return tr(QStringLiteral("J1 turn (rev)"), QStringLiteral("J1 转数"));
	}
	if (key == QStringLiteral("motion.axisConfig.turn.j4"))
	{
		return tr(QStringLiteral("J4 turn (rev)"), QStringLiteral("J4 转数"));
	}
	if (key == QStringLiteral("motion.axisConfig.turn.j6"))
	{
		return tr(QStringLiteral("J6 turn (rev)"), QStringLiteral("J6 转数"));
	}
	if (key == QStringLiteral("motion.blendRadius"))
	{
		return tr(QStringLiteral("Blend Radius (mm)"), QStringLiteral("平滑半径 (mm)"));
	}
	if (key == QStringLiteral("motion.pointIndex"))
	{
		return tr(QStringLiteral("Waypoint index"), QStringLiteral("点位编号"));
	}
	if (const property_core::PropertyDescriptor* descriptor = panelPropertyDescriptorForKey(key))
	{
		return QString::fromStdString(descriptor->label);
	}
	return labelEnFallback;
}

QString MainWindow::instructionEnumTokenFromProperty(QtProperty* property, const QVariant& value) const
{
	if (property)
	{
		const auto it = m_propertyEnumTokens.constFind(property);
		if (it != m_propertyEnumTokens.constEnd())
		{
			const int idx = value.toInt();
			if (idx >= 0 && idx < it->size())
			{
				return it->at(idx);
			}
		}
	}
	return instructionEnumTokenFromValue(property ? property->whatsThis() : QString(), value);
}

void MainWindow::invalidateFeasibleAxisConfigurationCache()
{
	m_cachedFeasibleAxisInstructionId.clear();
	m_cachedFeasibleAxisFingerprint.clear();
	m_cachedFeasibleAxisSeedJointRad.clear();
	m_cachedFeasibleAxisOptions = {};
}

void MainWindow::applySuggestedAxisPresetFromSeedIfNeeded(
	const std::shared_ptr<RobotInstruction::Base>& instruction,
	const QVector<double>& seedJointRad,
	const RobotInstruction::FeasibleMotionAxisConfigurationOptions& feasible)
{
	if (!instruction || !instruction->hasMotionAxisConfigurationProperty() || feasible.presetTokens.empty()
		|| seedJointRad.isEmpty())
	{
		return;
	}
	const auto tokenAllowed = [&feasible](const std::string& token) {
		return std::find(feasible.presetTokens.begin(), feasible.presetTokens.end(), token)
			!= feasible.presetTokens.end();
	};
	const auto pickFallback = [&]() -> std::string {
		if (tokenAllowed("AUTO"))
		{
			return "AUTO";
		}
		return feasible.presetTokens.front();
	};

	RobotInstruction::MotionAxisConfiguration cur = instruction->motionAxisConfiguration();
	const std::string curPreset = cur.preset;
	const bool presetLocked = !curPreset.empty() && curPreset != "AUTO" && tokenAllowed(curPreset);

	DocumentPage* doc = currentPage();
	if (!doc || !simulationCommandPage())
	{
		return;
	}
	const int instIdx = simulationCommandPage()->currentRobotInstanceIndex();
	if (instIdx < 0)
	{
		return;
	}
	const QStringList jnames = doc->robotRevoluteJointNamesForInstance(instIdx);
	std::vector<std::string> jointNames;
	jointNames.reserve(jnames.size());
	for (const QString& jn : jnames)
	{
		jointNames.push_back(jn.toStdString());
	}
	std::vector<double> seed(seedJointRad.size());
	for (int i = 0; i < seedJointRad.size(); ++i)
	{
		seed[static_cast<size_t>(i)] = seedJointRad[i];
	}
	const RobotInstruction::JointConfigurationClass observed =
		RobotInstruction::classifyJointConfiguration(seed, jointNames, &seed);
	if (!presetLocked)
	{
		std::string suggested = RobotInstruction::suggestMotionAxisPresetToken(observed);
		if (!tokenAllowed(suggested))
		{
			suggested = pickFallback();
		}
		if (suggested != curPreset)
		{
			instruction->applyPropertyChange("motion.axisConfig.preset", suggested, nullptr);
			cur = instruction->motionAxisConfiguration();
		}
	}
	const auto turnAllowed = [](const std::vector<std::string>& allowed, const std::string& tok) {
		return std::find(allowed.begin(), allowed.end(), tok) != allowed.end();
	};
	const auto applyTurnIfAuto = [&](const char* key, const int currentTurn, const int observedTurn,
									  const std::vector<std::string>& allowed) {
		if (currentTurn != RobotInstruction::kMotionAxisTurnAuto)
		{
			return;
		}
		const std::string tok = RobotInstruction::jointTurnToToken(observedTurn);
		if (!turnAllowed(allowed, tok))
		{
			return;
		}
		instruction->applyPropertyChange(key, tok, nullptr);
	};
	applyTurnIfAuto("motion.axisConfig.turn.j1", cur.turnJ1, observed.turnJ1, feasible.turnJ1Tokens);
	applyTurnIfAuto("motion.axisConfig.turn.j4", cur.turnJ4, observed.turnJ4, feasible.turnJ4Tokens);
	applyTurnIfAuto("motion.axisConfig.turn.j6", cur.turnJ6, observed.turnJ6, feasible.turnJ6Tokens);
}

void MainWindow::updateInstructionPropertyPanel(
	const std::shared_ptr<RobotInstruction::Base>& instruction,
	const bool refreshFeasibleAxisOptions)
{
	if (!m_propertyBrowser || !m_variantManager)
	{
		return;
	}
	m_updatingPropertyBrowser = true;
	m_propertyEnumTokens.clear();
	m_variantManager->clear();
	if (!instruction)
	{
		m_activeInstructionForProperty.reset();
		m_updatingPropertyBrowser = false;
		return;
	}
	m_activeInstructionForProperty = instruction;

	appendPropertyBrowserRow(QStringLiteral("core.id"),
		propertyDisplayLabelForKey(QStringLiteral("core.id"), QStringLiteral("ID")),
		QString::fromStdString(instruction->id()), false);
	appendPropertyBrowserRow(QStringLiteral("core.name"),
		propertyDisplayLabelForKey(QStringLiteral("core.name"), QStringLiteral("Name")),
		QString::fromStdString(instruction->name()), false);

	if (RobotInstruction::isMotionWaypointType(instruction->type()))
	{
		const int pointIndex = RobotInstruction::motionPointIndex(*instruction);
		QString pointValue = QStringLiteral("-");
		if (pointIndex > 0)
		{
			pointValue = QString::fromStdString(RobotInstruction::formatMotionPointName(pointIndex));
			pointValue += i18n(QStringLiteral(" (Point %1)"), QStringLiteral("（第 %1 点）")).arg(pointIndex);
		}
		appendPropertyBrowserRow(QStringLiteral("motion.pointIndex"),
			propertyDisplayLabelForKey(QStringLiteral("motion.pointIndex"), QStringLiteral("Waypoint index")),
			pointValue,
			false);
	}

	const nlohmann::json rows = instruction->snapshotPropertyRows();

	RobotInstruction::FeasibleMotionAxisConfigurationOptions feasibleAxis;
	QVector<double> seedJointRad;
	if (RobotInstruction::isMotionWaypointType(instruction->type()))
	{
		if (refreshFeasibleAxisOptions)
		{
			feasibleAxis = feasibleMotionAxisConfigurationOptionsForInstruction(instruction, &seedJointRad);
		}
		else
		{
			feasibleAxis = m_cachedFeasibleAxisOptions;
		}
		const auto ensureToken = [&](const char* key, const std::vector<std::string>& allowed, const QString& current) {
			if (allowed.empty() || !refreshFeasibleAxisOptions)
			{
				return;
			}
			const std::string cur = current.trimmed().toUpper().toStdString();
			if (cur.empty()
				|| std::find(allowed.begin(), allowed.end(), cur) != allowed.end())
			{
				return;
			}
			std::string fallback = allowed.front();
			if (key == std::string("motion.axisConfig.preset") && !seedJointRad.isEmpty())
			{
				DocumentPage* doc = currentPage();
				if (doc && simulationCommandPage())
				{
					const int instIdx = simulationCommandPage()->currentRobotInstanceIndex();
					if (instIdx >= 0)
					{
						const QStringList jnames = doc->robotRevoluteJointNamesForInstance(instIdx);
						std::vector<std::string> jointNames;
						std::vector<double> seed(seedJointRad.size());
						for (int i = 0; i < seedJointRad.size(); ++i)
						{
							seed[static_cast<size_t>(i)] = seedJointRad[i];
						}
						for (const QString& jn : jnames)
						{
							jointNames.push_back(jn.toStdString());
						}
						const std::string suggested = RobotInstruction::suggestMotionAxisPresetToken(
							RobotInstruction::classifyJointConfiguration(seed, jointNames, &seed));
						if (std::find(allowed.begin(), allowed.end(), suggested) != allowed.end())
						{
							fallback = suggested;
						}
						else if (std::find(allowed.begin(), allowed.end(), std::string("AUTO")) != allowed.end())
						{
							fallback = "AUTO";
						}
					}
				}
			}
			instruction->applyPropertyChange(key, fallback, nullptr);
		};
		ensureToken(
			"motion.axisConfig.preset",
			feasibleAxis.presetTokens,
			snapshotPropertyValueForKey(rows, QStringLiteral("motion.axisConfig.preset")));
		ensureToken(
			"motion.axisConfig.turn.j1",
			feasibleAxis.turnJ1Tokens,
			snapshotPropertyValueForKey(rows, QStringLiteral("motion.axisConfig.turn.j1")));
		ensureToken(
			"motion.axisConfig.turn.j4",
			feasibleAxis.turnJ4Tokens,
			snapshotPropertyValueForKey(rows, QStringLiteral("motion.axisConfig.turn.j4")));
		ensureToken(
			"motion.axisConfig.turn.j6",
			feasibleAxis.turnJ6Tokens,
			snapshotPropertyValueForKey(rows, QStringLiteral("motion.axisConfig.turn.j6")));
	}

	nlohmann::json rowsAfter = instruction->snapshotPropertyRows();
	if (RobotInstruction::isMotionWaypointType(instruction->type()))
	{
		const QString presetMid = snapshotPropertyValueForKey(rowsAfter, QStringLiteral("motion.axisConfig.preset"));
		const bool customAxisMode = presetMid.compare(QStringLiteral("CUSTOM"), Qt::CaseInsensitive) == 0;
		if (customAxisMode)
		{
			const auto ensureToken = [&](const char* key, const std::vector<std::string>& allowed, const QString& current) {
				if (allowed.empty())
				{
					return;
				}
				const std::string cur = current.trimmed().toUpper().toStdString();
				if (cur.empty()
					|| std::find(allowed.begin(), allowed.end(), cur) != allowed.end())
				{
					return;
				}
				instruction->applyPropertyChange(key, allowed.front(), nullptr);
			};
			ensureToken(
				"motion.axisConfig.elbow",
				feasibleAxis.elbowTokens,
				snapshotPropertyValueForKey(rowsAfter, QStringLiteral("motion.axisConfig.elbow")));
			ensureToken(
				"motion.axisConfig.wrist",
				feasibleAxis.wristTokens,
				snapshotPropertyValueForKey(rowsAfter, QStringLiteral("motion.axisConfig.wrist")));
			ensureToken(
				"motion.axisConfig.arm",
				feasibleAxis.armTokens,
				snapshotPropertyValueForKey(rowsAfter, QStringLiteral("motion.axisConfig.arm")));
			rowsAfter = instruction->snapshotPropertyRows();
		}
	}

	const QString presetAfter = snapshotPropertyValueForKey(rowsAfter, QStringLiteral("motion.axisConfig.preset"));
	const bool customAxisModeAfter = presetAfter.compare(QStringLiteral("CUSTOM"), Qt::CaseInsensitive) == 0;

	if (instruction->hasPoseProperty())
	{
		const auto& ext = instruction->extensionProperties();
		DocumentPage* doc = currentPage();
		const int instIdx = simulationCommandPage() ? simulationCommandPage()->currentRobotInstanceIndex() : -1;
		std::vector<std::string> toolTokens = { "active" };
		std::vector<std::string> userTokens = { "active" };
		QStringList toolEnumNames;
		QStringList userEnumNames;
		toolEnumNames << i18n(QStringLiteral("Active (follow robot)"), QStringLiteral("跟随当前工具"));
		userEnumNames << i18n(QStringLiteral("Active (follow robot)"), QStringLiteral("跟随当前用户系"));
		if (doc && instIdx >= 0)
		{
			const RobotCoordinate::RobotCoordinateFrameSet& frames =
				doc->robotCoordinateFramesForInstance(instIdx);
			for (const RobotCoordinate::RobotToolFrame& tf : frames.toolFrames)
			{
				toolTokens.push_back(tf.id);
				QString label = QString::fromStdString(tf.name);
				if (!tf.id.empty() && label != QString::fromStdString(tf.id))
				{
					label += QStringLiteral(" (") + QString::fromStdString(tf.id) + QLatin1Char(')');
				}
				toolEnumNames << label;
			}
			for (const RobotCoordinate::RobotUserFrame& uf : frames.userFrames)
			{
				userTokens.push_back(uf.id);
				QString label = QString::fromStdString(uf.name);
				if (!uf.id.empty() && label != QString::fromStdString(uf.id))
				{
					label += QStringLiteral(" (") + QString::fromStdString(uf.id) + QLatin1Char(')');
				}
				userEnumNames << label;
			}
		}
		QString toolVal = QStringLiteral("active");
		const auto itTool = ext.find(RobotCoordinate::kExtMotionToolFrameId);
		if (itTool != ext.end() && !itTool->second.empty())
		{
			toolVal = QString::fromStdString(itTool->second);
		}
		appendPropertyBrowserRow(
			QStringLiteral("motion.tool.frameId"),
			propertyDisplayLabelForKey(QStringLiteral("motion.tool.frameId"), QStringLiteral("Tool frame")),
			toolVal,
			true,
			&toolTokens,
			&toolEnumNames,
			i18n(
				QStringLiteral(
					"When changing the tool frame, the TCP position in space is kept; joint angles are "
					"recomputed automatically."),
				QStringLiteral("切换工具系时，系统将保持工具尖端（TCP）空间位置不变，自动重新计算关节角度。")));
		QString userVal = QStringLiteral("active");
		const auto itUser = ext.find(RobotCoordinate::kExtMotionUserFrameId);
		if (itUser != ext.end() && !itUser->second.empty())
		{
			userVal = QString::fromStdString(itUser->second);
		}
		appendPropertyBrowserRow(
			QStringLiteral("motion.user.frameId"),
			propertyDisplayLabelForKey(QStringLiteral("motion.user.frameId"), QStringLiteral("User frame")),
			userVal,
			true,
			&userTokens,
			&userEnumNames);
		QString frameVal = QStringLiteral("base");
		const auto itFr = ext.find(RobotCoordinate::kExtMotionTargetFrame);
		if (itFr != ext.end() && !itFr->second.empty())
		{
			frameVal = QString::fromStdString(itFr->second);
			if (frameVal == QStringLiteral("active_user"))
			{
				frameVal = QStringLiteral("user");
			}
		}
		static const std::vector<std::string> frameTokens = { "base", "user" };
		const QStringList frameEnumNames = {
			i18n(QStringLiteral("Robot base (TCP)"), QStringLiteral("机器人基座 (TCP)")),
			i18n(QStringLiteral("User frame (TCP)"), QStringLiteral("用户坐标系 (TCP)")),
		};
		appendPropertyBrowserRow(
			QStringLiteral("motion.target.frame"),
			propertyDisplayLabelForKey(QStringLiteral("motion.target.frame"), QStringLiteral("Target frame")),
			frameVal,
			true,
			&frameTokens,
			&frameEnumNames);
	}

	const BackendMat4 T_display = instructionTcpForDisplay(*this, *instruction);

	if (rowsAfter.is_array())
	{
		for (const auto& r : rowsAfter)
		{
			if (!r.is_object())
			{
				continue;
			}
			const std::string keyStr = r.value(backend_property_json::kKey, std::string());
			if (isInstructionPanelManagedExtensionKey(keyStr) || keyStr.rfind("legacy.", 0) == 0
				|| keyStr == "motion.durationSec" || keyStr == RobotInstruction::kMotionPointIndexKey)
			{
				continue;
			}
			if (!customAxisModeAfter
				&& (keyStr == "motion.axisConfig.elbow" || keyStr == "motion.axisConfig.wrist"
					|| keyStr == "motion.axisConfig.arm"))
			{
				continue;
			}
			const std::string labelStr = r.value(backend_property_json::kLabelEn, std::string());
			const bool editable = r.value(backend_property_json::kEditable, false);
			std::string valueStr = r.value(backend_property_json::kValue, std::string());
			const QString key = QString::fromStdString(keyStr);
			if (isMotionTargetPoseKey(key))
			{
				const RobotCoordinate::RobotRigidFrame disp = RobotCoordinate::mat4ToFrame(T_display);
				if (key == QStringLiteral("motion.target.pose.x"))
				{
					valueStr = std::to_string(disp.positionMm[0]);
				}
				else if (key == QStringLiteral("motion.target.pose.y"))
				{
					valueStr = std::to_string(disp.positionMm[1]);
				}
				else if (key == QStringLiteral("motion.target.pose.z"))
				{
					valueStr = std::to_string(disp.positionMm[2]);
				}
				else if (key == QStringLiteral("motion.target.euler.rx"))
				{
					valueStr = std::to_string(disp.eulerDeg[0]);
				}
				else if (key == QStringLiteral("motion.target.euler.ry"))
				{
					valueStr = std::to_string(disp.eulerDeg[1]);
				}
				else if (key == QStringLiteral("motion.target.euler.rz"))
				{
					valueStr = std::to_string(disp.eulerDeg[2]);
				}
			}
			QString label = propertyDisplayLabelForKey(key, QString::fromStdString(labelStr));
			if (instructionUsesActiveUserFrame(*instruction) && isMotionTargetPoseKey(key))
			{
				DocumentPage* doc = currentPage();
				if (doc && simulationCommandPage())
				{
					const int instIdx = simulationCommandPage()->currentRobotInstanceIndex();
					if (instIdx >= 0)
					{
						const RobotCoordinate::RobotCoordinateFrameSet& frames =
							doc->robotCoordinateFramesForInstance(instIdx);
						if (const RobotCoordinate::RobotUserFrame* uf = RobotCoordinate::resolveUserFrameForExtension(
								frames, instruction->extensionProperties()))
						{
							label += QStringLiteral(" (%1)").arg(QString::fromStdString(uf->name));
						}
					}
				}
			}
			else if (isMotionTargetPoseKey(key))
			{
				const auto& extPose = instruction->extensionProperties();
				const auto itToolId = extPose.find(RobotCoordinate::kExtMotionToolFrameId);
				if (itToolId != extPose.end() && !itToolId->second.empty() && itToolId->second != "active")
				{
					label += i18n(QStringLiteral(" (TCP mm)"), QStringLiteral(" (TCP mm)"));
				}
			}
			const std::vector<std::string>* enumOverride = nullptr;
			if (key == QStringLiteral("motion.axisConfig.preset") && !feasibleAxis.presetTokens.empty())
			{
				enumOverride = &feasibleAxis.presetTokens;
			}
			else if (key == QStringLiteral("motion.axisConfig.elbow") && !feasibleAxis.elbowTokens.empty())
			{
				enumOverride = &feasibleAxis.elbowTokens;
			}
			else if (key == QStringLiteral("motion.axisConfig.wrist") && !feasibleAxis.wristTokens.empty())
			{
				enumOverride = &feasibleAxis.wristTokens;
			}
			else if (key == QStringLiteral("motion.axisConfig.arm") && !feasibleAxis.armTokens.empty())
			{
				enumOverride = &feasibleAxis.armTokens;
			}
			else if (key == QStringLiteral("motion.axisConfig.turn.j1") && !feasibleAxis.turnJ1Tokens.empty())
			{
				enumOverride = &feasibleAxis.turnJ1Tokens;
			}
			else if (key == QStringLiteral("motion.axisConfig.turn.j4") && !feasibleAxis.turnJ4Tokens.empty())
			{
				enumOverride = &feasibleAxis.turnJ4Tokens;
			}
			else if (key == QStringLiteral("motion.axisConfig.turn.j6") && !feasibleAxis.turnJ6Tokens.empty())
			{
				enumOverride = &feasibleAxis.turnJ6Tokens;
			}
			appendPropertyBrowserRow(key, label, QString::fromStdString(valueStr), editable, enumOverride);
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
		QString valueText = instructionEnumTokenFromProperty(property, value);
		if (propertyKey == QStringLiteral("motion.target.frame"))
		{
			std::string tok = valueText.toStdString();
			if (tok == "active_user")
			{
				tok = "user";
			}
			m_activeInstructionForProperty->setExtensionProperty(
				RobotCoordinate::kExtMotionTargetFrame, tok);
			invalidateFeasibleAxisConfigurationCache();
			updateInstructionPropertyPanel(m_activeInstructionForProperty, false);
			refreshInstructionPoseAxes();
			return;
		}
		if (propertyKey == QStringLiteral("motion.tool.frameId"))
		{
			m_activeInstructionForProperty->setExtensionProperty(
				RobotCoordinate::kExtMotionToolFrameId, valueText.toStdString());
			if (DocumentPage* doc = currentPage())
			{
				const int instIdx = simulationCommandPage()
					? simulationCommandPage()->currentRobotInstanceIndex()
					: -1;
				if (instIdx >= 0)
				{
					const RobotCoordinate::RobotCoordinateFrameSet& frames =
						doc->robotCoordinateFramesForInstance(instIdx);
					if (const RobotCoordinate::RobotToolFrame* tool = RobotCoordinate::resolveToolFrameForExtension(
							frames, m_activeInstructionForProperty->extensionProperties()))
					{
						const BackendMat4 toolMat = RobotCoordinate::frameToMat4(tool->T_flange_tool);
						m_activeInstructionForProperty->setExtensionProperty(
							RobotCoordinate::kExtContextToolFrameMat4,
							RobotCoordinate::encodeMat4Csv(toolMat));
						m_activeInstructionForProperty->setExtensionProperty(
							"context.activeToolFrameId", tool->id);
						const std::string flangeLink =
							RobotCoordinate::effectiveFlangeLinkName(frames, *tool);
						if (!flangeLink.empty())
						{
							m_activeInstructionForProperty->setExtensionProperty(
								"context.flangeLinkName", flangeLink);
						}
					}
				}
			}
			int changedMotionIndex = -1;
			if (simulationCommandPage())
			{
				const std::vector<std::shared_ptr<RobotInstruction::Base>> program =
					simulationCommandPage()->instructions(simulationCommandPage()->currentRobotBackendId());
				const std::vector<const RobotInstruction::Base*> motions =
					RobotInstruction::collectMotionInstructions(program);
				for (size_t i = 0; i < motions.size(); ++i)
				{
					if (motions[i] && motions[i]->id() == m_activeInstructionForProperty->id())
					{
						changedMotionIndex = static_cast<int>(i);
						break;
					}
				}
				if (changedMotionIndex >= 0)
				{
					RobotInstructionPlanning::invalidateTaughtJointsFromMotionIndexForward(
						motions, changedMotionIndex);
				}
				else
				{
					RobotInstructionPlanning::invalidateTaughtJointsForToolFrameChange(
						*m_activeInstructionForProperty);
				}
			}
			else
			{
				RobotInstructionPlanning::invalidateTaughtJointsForToolFrameChange(
					*m_activeInstructionForProperty);
			}
			syncInstructionRenderMatricesFromPose(m_activeInstructionForProperty);
			invalidateFeasibleAxisConfigurationCache();
			applyRobotPoseForInstructionPreview(m_activeInstructionForProperty);
			// 工具系切换已触发整链预览 IK；延后可行轴枚举，避免与预览重复多初值求解造成卡顿。
			updateInstructionPropertyPanel(m_activeInstructionForProperty, false);
			refreshRobotCoordinateFrameOverlays(m_activeInstructionForProperty);
			refreshInstructionPoseAxes();
			if (m_runInfoPage)
			{
				QString toolLabel = valueText;
				if (DocumentPage* doc = currentPage())
				{
					const int instIdx = simulationCommandPage()
						? simulationCommandPage()->currentRobotInstanceIndex()
						: -1;
					if (instIdx >= 0)
					{
						const RobotCoordinate::RobotCoordinateFrameSet& frames =
							doc->robotCoordinateFramesForInstance(instIdx);
						if (const RobotCoordinate::RobotToolFrame* tool = RobotCoordinate::resolveToolFrameForExtension(
								frames, m_activeInstructionForProperty->extensionProperties()))
						{
							toolLabel = QString::fromStdString(tool->name);
						}
					}
				}
				m_runInfoPage->appendInfo(
					i18n(QStringLiteral("Tool frame → %1; TCP target unchanged, flange IK and preview updated.")
							.arg(toolLabel),
						QStringLiteral("工具系 → %1；TCP 目标未变，已重算法兰 IK 与预览。").arg(toolLabel)));
			}
			return;
		}
		if (propertyKey == QStringLiteral("motion.user.frameId"))
		{
			m_activeInstructionForProperty->setExtensionProperty(
				RobotCoordinate::kExtMotionUserFrameId, valueText.toStdString());
			updateInstructionPropertyPanel(m_activeInstructionForProperty, false);
			refreshInstructionPoseAxes();
			return;
		}
		if (isMotionTargetPoseKey(propertyKey))
		{
			BackendMat4 T_disp = instructionTcpForDisplay(*this, *m_activeInstructionForProperty);
			RobotCoordinate::RobotRigidFrame disp = RobotCoordinate::mat4ToFrame(T_disp);
			bool ok = false;
			const double v = valueText.toDouble(&ok);
			if (ok)
			{
				if (propertyKey == QStringLiteral("motion.target.pose.x"))
				{
					disp.positionMm[0] = v;
				}
				else if (propertyKey == QStringLiteral("motion.target.pose.y"))
				{
					disp.positionMm[1] = v;
				}
				else if (propertyKey == QStringLiteral("motion.target.pose.z"))
				{
					disp.positionMm[2] = v;
				}
				else if (propertyKey == QStringLiteral("motion.target.euler.rx"))
				{
					disp.eulerDeg[0] = v;
				}
				else if (propertyKey == QStringLiteral("motion.target.euler.ry"))
				{
					disp.eulerDeg[1] = v;
				}
				else if (propertyKey == QStringLiteral("motion.target.euler.rz"))
				{
					disp.eulerDeg[2] = v;
				}
			}
			BackendMat4 T_base_tcp;
			if (instructionUsesActiveUserFrame(*m_activeInstructionForProperty))
			{
				DocumentPage* doc = currentPage();
				BackendMat4 T_base_user = BackendMat4::identity();
				if (doc && simulationCommandPage())
				{
					const int instIdx = simulationCommandPage()->currentRobotInstanceIndex();
					if (instIdx >= 0)
					{
						const RobotCoordinate::RobotCoordinateFrameSet& frames =
							doc->robotCoordinateFramesForInstance(instIdx);
						if (const RobotCoordinate::RobotUserFrame* uf = RobotCoordinate::resolveUserFrameForExtension(
								frames, m_activeInstructionForProperty->extensionProperties()))
						{
							T_base_user = RobotCoordinate::frameToMat4(uf->T_base_user);
						}
					}
				}
				const BackendMat4 T_user_tcp = RobotCoordinate::frameToMat4(disp);
				T_base_tcp = RobotCoordinate::tcpInBaseFromUserTcp(T_base_user, T_user_tcp);
			}
			else
			{
				T_base_tcp = RobotCoordinate::frameToMat4(disp);
			}
			setInstructionTcpInBase(*m_activeInstructionForProperty, T_base_tcp);
			syncInstructionRenderMatricesFromPose(m_activeInstructionForProperty);
		}
		else
		{
			std::string err;
			if (!m_activeInstructionForProperty->applyPropertyChange(
					propertyKey.toStdString(), valueText.toStdString(), &err))
			{
				updateInstructionPropertyPanel(m_activeInstructionForProperty);
				return;
			}
		}
		if (simulationCommandPage())
		{
			simulationCommandPage()->refreshInstructionList();
		}
		refreshInstructionPoseAxes();
		applyRobotPoseForInstructionPreview(m_activeInstructionForProperty);
		const bool axisConfigOnly = propertyKey.startsWith(QStringLiteral("motion.axisConfig"));
		if (!axisConfigOnly)
		{
			invalidateFeasibleAxisConfigurationCache();
		}
		updateInstructionPropertyPanel(m_activeInstructionForProperty, !axisConfigOnly);
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
	if (isPoseComponentKey(propertyKey) || isRotationComponentKey(propertyKey))
	{
		syncRobotKinematicsAfterPoseEdit(data);
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
