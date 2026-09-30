/// @file OsgSceneNodeI18n.cpp
/// @brief 场景层级 OSG 节点名/类型中文映射

#include "OsgSceneNodeI18n.h"

#include "BackendTypeIds.h"

#include <QHash>
#include <QLatin1String>

namespace
{
const QHash<QString, QString>& osgClassNameZhMap()
{
	static const QHash<QString, QString> map = {
		{QStringLiteral("Group"), QStringLiteral("组")},
		{QStringLiteral("MatrixTransform"), QStringLiteral("矩阵变换")},
		{QStringLiteral("PositionAttitudeTransform"), QStringLiteral("位姿变换")},
		{QStringLiteral("Geode"), QStringLiteral("几何节点")},
		{QStringLiteral("Geometry"), QStringLiteral("几何体")},
		{QStringLiteral("Camera"), QStringLiteral("相机")},
		{QStringLiteral("AutoTransform"), QStringLiteral("自动变换")},
		{QStringLiteral("Switch"), QStringLiteral("开关节点")},
		{QStringLiteral("LOD"), QStringLiteral("细节层次")},
		{QStringLiteral("LightSource"), QStringLiteral("光源")},
	};
	return map;
}

const QHash<QString, QString>& osgNodeNameZhMap()
{
	static const QHash<QString, QString> map = {
		{QStringLiteral("SceneContent"), QStringLiteral("场景内容")},
		{QStringLiteral("BackendObjects"), QStringLiteral("后端对象")},
		{QStringLiteral("RobotAssembly"), QStringLiteral("机器人装配")},
		{QStringLiteral("TrajectoryOverlay"), QStringLiteral("轨迹叠加层")},
		{QStringLiteral("TcpTeachSceneOverlay"), QStringLiteral("TCP示教场景叠加")},
		{QStringLiteral("GizmoOverlay"), QStringLiteral("变换罗盘叠加")},
		{QStringLiteral("Annotations"), QStringLiteral("注释")},
		{QStringLiteral("InstructionPoseAxes"), QStringLiteral("指令位姿轴")},
		{QStringLiteral("LINE_TargetAxis"), QStringLiteral("直线目标轴")},
		{QStringLiteral("PTP_TargetAxis"), QStringLiteral("点到点目标轴")},
		{QStringLiteral("TcpTeachCompass"), QStringLiteral("TCP示教罗盘")},
		{QStringLiteral("TcpTeachMount"), QStringLiteral("TCP示教挂载")},
		{QStringLiteral("TcpTeachWorld"), QStringLiteral("TCP示教世界")},
		{QStringLiteral("TcpTeachGizmoOverlay"), QStringLiteral("TCP示教罗盘叠加")},
		{QStringLiteral("RobotHierarchy"), QStringLiteral("机器人层级")},
		{QStringLiteral("meshWireOverlay"), QStringLiteral("网格线框叠加")},
		{QStringLiteral("RosZUp_to_OsgYUp"), QStringLiteral("ROS Z上→OSG Y上")},
		{QStringLiteral("LinkFrameAxes"), QStringLiteral("连杆坐标轴")},
		{QStringLiteral("JointRotationAxis"), QStringLiteral("关节旋转轴")},
		{QLatin1String(backend_type::kCatalogPointCloud), QStringLiteral("点云")},
		{QLatin1String(backend_type::kCatalogModel), QStringLiteral("模型")},
	};
	return map;
}
} // namespace

namespace OsgSceneNodeI18n
{
QString translateClassName(const QString& cls, const bool useChinese)
{
	if (!useChinese)
	{
		return cls;
	}
	return osgClassNameZhMap().value(cls, cls);
}

QString translateNodeName(const QString& name, const bool useChinese)
{
	if (!useChinese || name.isEmpty())
	{
		return name;
	}
	const auto exact = osgNodeNameZhMap().constFind(name);
	if (exact != osgNodeNameZhMap().constEnd())
	{
		return exact.value();
	}
	if (name.startsWith(QStringLiteral("RobotToolFrame_")))
	{
		return QStringLiteral("机器人工具坐标系_") + name.mid(15);
	}
	if (name.startsWith(QStringLiteral("RobotUserFrame_")))
	{
		return QStringLiteral("机器人用户坐标系_") + name.mid(15);
	}
	if (name.startsWith(QStringLiteral("Joint")) && name.length() > 5)
	{
		bool ok = false;
		name.mid(5).toInt(&ok);
		if (ok)
		{
			return QStringLiteral("关节") + name.mid(5);
		}
	}
	if (name.startsWith(QStringLiteral("Link")) && name.length() > 4)
	{
		bool ok = false;
		name.mid(4).toInt(&ok);
		if (ok)
		{
			return QStringLiteral("连杆") + name.mid(4);
		}
	}
	if (name.startsWith(QStringLiteral("Axis_Visual_")))
	{
		return QStringLiteral("轴可视化_") + name.mid(12);
	}
	auto replaceSuffix = [&name](const QString& enSuffix, const QString& zhSuffix) -> QString
	{
		if (name.endsWith(enSuffix))
		{
			return name.left(name.size() - enSuffix.size()) + zhSuffix;
		}
		return QString();
	};
	if (QString r = replaceSuffix(QStringLiteral("_JointContent"), QStringLiteral("_关节内容")); !r.isEmpty())
	{
		return r;
	}
	if (QString r = replaceSuffix(QStringLiteral("_OriginMarker"), QStringLiteral("_原点标记")); !r.isEmpty())
	{
		return r;
	}
	if (QString r = replaceSuffix(QStringLiteral("_BackendVisual"), QStringLiteral("_后端可视化")); !r.isEmpty())
	{
		return r;
	}
	if (QString r = replaceSuffix(QStringLiteral("_Geometry"), QStringLiteral("_几何")); !r.isEmpty())
	{
		return r;
	}
	if (QString r = replaceSuffix(QStringLiteral("_Container"), QStringLiteral("_容器")); !r.isEmpty())
	{
		return r;
	}
	if (QString r = replaceSuffix(QStringLiteral("_Visual"), QStringLiteral("_可视化")); !r.isEmpty())
	{
		return r;
	}
	return name;
}
} // namespace OsgSceneNodeI18n
