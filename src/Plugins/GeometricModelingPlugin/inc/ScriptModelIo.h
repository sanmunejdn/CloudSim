#ifndef GEOMETRICMODELINGPLUGIN_SCRIPTMODELIO_H
#define GEOMETRICMODELINGPLUGIN_SCRIPTMODELIO_H

/// @file ScriptModelIo.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 脚本建模 JSON 判别与 unwrap（history / feature.compose）

#include <QByteArray>
#include <QString>

enum class ScriptModelJsonKind
{
	Invalid = 0,
	History,
	Compose
};

struct ScriptModelParseResult
{
	ScriptModelJsonKind kind = ScriptModelJsonKind::Invalid;
	/// history：可直接交给 setParametricBodyHistoryJson；compose：原样交给 executeActionPlan
	QByteArray payloadUtf8;
	QString error;
};

ScriptModelParseResult parseScriptModelJson(const QByteArray& utf8);

#endif
