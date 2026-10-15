/// @file ProgressUiText.cpp
/// @brief 进度文案本地化

#include "ProgressUiText.h"

namespace
{
QString mapExact(const QString& text, bool zh)
{
	struct Pair
	{
		const char* en;
		const char* cn;
	};
	static const Pair kExact[] = {
		{"CGAL read / decode...", "正在读取 / 解码…"},
		{"Meshing B-rep...", "正在网格化 B-rep…"},
		{"Meshing B-rep parts...", "正在网格化 B-rep 零件…"},
		{"Meshing B-rep assembly...", "正在网格化 B-rep 装配…"},
		{"Slicing B-rep parts...", "正在切片 B-rep 零件…"},
		{"Reading model...", "正在读取模型…"},
		{"Reading URDF / building scene...", "正在读取 URDF / 构建场景…"},
		{"Reading project...", "正在读取工程…"},
		{"Loading JSON...", "正在加载 JSON…"},
		{"Assembly model import", "装配模型导入"},
		{"Running...", "进行中…"},
		{"Done", "完成"},
		{"done", "完成"},
		{"failed", "失败"},
		{"Running ICP...", "正在运行 ICP…"},
		{"Running SPARE (world frame)...", "正在运行 SPARE…"},
		{"Running SDF/DDF (world frame)...", "正在运行 SDF/DDF…"},
		{"Running TPS...", "正在运行 TPS…"},
		{"ProcessFlow DES", "工艺流 DES 仿真"},
	};
	for (const Pair& p : kExact)
	{
		if (text == QLatin1String(p.en))
		{
			return zh ? QString::fromUtf8(p.cn) : QString::fromLatin1(p.en);
		}
	}
	return {};
}

QString mapPrefixed(const QString& text, bool zh)
{
	struct Pref
	{
		const char* en;
		const char* cn;
	};
	static const Pref kPref[] = {
		{"Point cloud: ", "点云："},
		{"Import model: ", "导入模型："},
		{"URDF: ", "URDF："},
		{"Open project: ", "打开工程："},
		{"Pyramid L0..L2 (world frame)...", "金字塔配准 L0..L2…"},
	};
	for (const Pref& p : kPref)
	{
		const QLatin1String en(p.en);
		if (text.startsWith(en))
		{
			const QString rest = text.mid(en.size());
			return zh ? (QString::fromUtf8(p.cn) + rest) : text;
		}
	}
	return {};
}
} // namespace

namespace ProgressUiText
{
QString localize(const QString& text, bool useChinese)
{
	if (text.isEmpty())
	{
		return text;
	}
	if (!useChinese)
	{
		return text;
	}
	if (const QString exact = mapExact(text, true); !exact.isEmpty())
	{
		return exact;
	}
	if (const QString pref = mapPrefixed(text, true); !pref.isEmpty())
	{
		return pref;
	}
	// 带进度序号的切片文案：Slicing B-rep parts... (3/10)
	if (text.startsWith(QLatin1String("Slicing B-rep parts...")))
	{
		return QStringLiteral("正在切片 B-rep 零件…") + text.mid(QStringLiteral("Slicing B-rep parts...").size());
	}
	if (text.startsWith(QLatin1String("Meshing B-rep parts...")))
	{
		return QStringLiteral("正在网格化 B-rep 零件…") + text.mid(QStringLiteral("Meshing B-rep parts...").size());
	}
	return text;
}
} // namespace ProgressUiText
