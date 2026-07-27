/// @file UiIcons.cpp
/// @brief UiIcons 实现

#include "UiIcons.h"

#include <QHash>
#include <QIcon>
#include <QPixmap>
#include <QSettings>
#include <QString>
#include <QtDebug>

inline uint qHash(UiIconId id, uint seed = 0) noexcept
{
	return ::qHash(static_cast<int>(id), seed);
}

static void initCloudsimIconResources()
{
	Q_INIT_RESOURCE(cloudsim_icons);
}

namespace
{
bool g_resourcesInitialized = false;
UiIcons::Theme g_explicitTheme = UiIcons::Theme::Auto;
bool g_hasExplicitTheme = false;

using CacheKey = quint64;

CacheKey makeCacheKey(UiIconId id, UiIcons::Size size, UiIcons::Theme theme)
{
	return (static_cast<quint64>(id) << 16) | (static_cast<quint64>(static_cast<int>(size)) << 8) |
		   static_cast<quint64>(static_cast<int>(theme));
}

QString themeFolder(UiIcons::Theme theme)
{
	return theme == UiIcons::Theme::Dark ? QStringLiteral("dark") : QStringLiteral("light");
}

UiIcons::Theme themeFromSettings()
{
	QSettings settings;
	settings.beginGroup(QStringLiteral("Appearance"));
	const QString value = settings.value(QStringLiteral("theme"), QStringLiteral("light")).toString();
	return value.compare(QStringLiteral("dark"), Qt::CaseInsensitive) == 0 ? UiIcons::Theme::Dark
																		   : UiIcons::Theme::Light;
}

UiIcons::Theme resolveTheme(UiIcons::Theme requested)
{
	if (requested != UiIcons::Theme::Auto)
	{
		return requested;
	}
	if (g_hasExplicitTheme)
	{
		return g_explicitTheme;
	}
	return themeFromSettings();
}

const QHash<UiIconId, QString>& iconBasenameMap()
{
	static const QHash<UiIconId, QString> map = {
		{UiIconId::NewDocument, QStringLiteral("new_document")},
		{UiIconId::OpenProject, QStringLiteral("open_project")},
		{UiIconId::SaveProject, QStringLiteral("save_project")},
		{UiIconId::OpenModel, QStringLiteral("open_model")},
		{UiIconId::OpenPointCloud, QStringLiteral("open_point_cloud")},
		{UiIconId::Exit, QStringLiteral("exit")},
		{UiIconId::Undo, QStringLiteral("undo")},
		{UiIconId::Redo, QStringLiteral("redo")},
		{UiIconId::Delete, QStringLiteral("delete")},
		{UiIconId::Clear, QStringLiteral("clear")},
		{UiIconId::Add, QStringLiteral("add")},
		{UiIconId::Rename, QStringLiteral("rename")},
		{UiIconId::Duplicate, QStringLiteral("duplicate")},
		{UiIconId::Run, QStringLiteral("run")},
		{UiIconId::Stop, QStringLiteral("stop")},
		{UiIconId::Export, QStringLiteral("export")},
		{UiIconId::Ptp, QStringLiteral("ptp")},
		{UiIconId::Line, QStringLiteral("line")},
		{UiIconId::TcpDragTeach, QStringLiteral("tcp_drag")},
		{UiIconId::Wait, QStringLiteral("wait")},
		{UiIconId::If, QStringLiteral("if")},
		{UiIconId::While, QStringLiteral("while")},
		{UiIconId::SetDo, QStringLiteral("set_do")},
		{UiIconId::SetAo, QStringLiteral("set_ao")},
		{UiIconId::Apply, QStringLiteral("apply")},
		{UiIconId::Reset, QStringLiteral("reset")},
		{UiIconId::SaveTemplate, QStringLiteral("save_template")},
		{UiIconId::LoadTemplate, QStringLiteral("load_template")},
		{UiIconId::NewPathPlan, QStringLiteral("new_path_plan")},
		{UiIconId::PickEdge, QStringLiteral("pick_edge")},
		{UiIconId::PickFace, QStringLiteral("pick_face")},
		{UiIconId::Discretize, QStringLiteral("discretize")},
		{UiIconId::Refresh, QStringLiteral("refresh")},
		{UiIconId::FillRecipe, QStringLiteral("fill_recipe")},
		{UiIconId::EmitProgram, QStringLiteral("emit_program")},
		{UiIconId::ViewMode, QStringLiteral("view_mode")},
		{UiIconId::ObjectSelect, QStringLiteral("object_select")},
		{UiIconId::PointPick, QStringLiteral("point_pick")},
		{UiIconId::LinePick, QStringLiteral("line_pick")},
		{UiIconId::FacePick, QStringLiteral("face_pick")},
		{UiIconId::TransformWorld, QStringLiteral("transform_world")},
		{UiIconId::TransformLocal, QStringLiteral("transform_local")},
		{UiIconId::Send, QStringLiteral("send")},
		{UiIconId::Settings, QStringLiteral("settings")},
		{UiIconId::RobotPlaceholder, QStringLiteral("robot_placeholder")},
		{UiIconId::Connect, QStringLiteral("connect")},
		{UiIconId::Disconnect, QStringLiteral("disconnect")},
		{UiIconId::Read, QStringLiteral("read")},
		{UiIconId::Write, QStringLiteral("write")},
		{UiIconId::ClearLog, QStringLiteral("clear_log")},
		{UiIconId::SetActive, QStringLiteral("set_active")},
		{UiIconId::FocusCamera, QStringLiteral("focus_camera")},
		{UiIconId::Wireframe, QStringLiteral("wireframe")},
		{UiIconId::Screenshot, QStringLiteral("screenshot")},
		{UiIconId::Close, QStringLiteral("close")},
		{UiIconId::DockFloat, QStringLiteral("dock_float")},
	};
	return map;
}

void ensureResourcesInitialized()
{
	if (g_resourcesInitialized)
	{
		return;
	}
	initCloudsimIconResources();
	g_resourcesInitialized = true;
}

QHash<CacheKey, QIcon>& iconCache()
{
	static QHash<CacheKey, QIcon> cache;
	return cache;
}

} // namespace

namespace UiIcons
{
void setTheme(Theme theme)
{
	g_explicitTheme = theme;
	g_hasExplicitTheme = theme != Theme::Auto;
}

Theme currentTheme()
{
	return resolveTheme(Theme::Auto);
}

void invalidateCache()
{
	iconCache().clear();
}

QIcon icon(UiIconId id, Size size, Theme theme)
{
	ensureResourcesInitialized();

	const Theme resolved = resolveTheme(theme);
	const CacheKey key = makeCacheKey(id, size, resolved);
	const auto cacheIt = iconCache().constFind(key);
	if (cacheIt != iconCache().constEnd())
	{
		return cacheIt.value();
	}

	const QString basename = iconBasenameMap().value(id);
	if (basename.isEmpty())
	{
		qWarning() << "[UiIcons] unknown icon id" << static_cast<int>(id);
		return {};
	}

	const int px = static_cast<int>(size);
	const QString folder = themeFolder(resolved);
	const QString path1x =
		QStringLiteral(":/cloudsim/icons/%1/%2_%3.png").arg(folder, basename, QString::number(px));
	const QString path2x =
		QStringLiteral(":/cloudsim/icons/%1/%2_%3.png").arg(folder, basename, QString::number(px * 2));

	QIcon result;
	const QPixmap pm1x(path1x);
	if (pm1x.isNull())
	{
		qWarning() << "[UiIcons] missing resource" << path1x;
		return {};
	}
	result.addPixmap(pm1x);

	// 高分屏用 @2x 位图，避免把 24px 硬拉大发糊/发锯
	QPixmap pm2x(path2x);
	if (!pm2x.isNull())
	{
		pm2x.setDevicePixelRatio(2.0);
		result.addPixmap(pm2x);
	}

	iconCache().insert(key, result);
	return result;
}

} // namespace UiIcons
