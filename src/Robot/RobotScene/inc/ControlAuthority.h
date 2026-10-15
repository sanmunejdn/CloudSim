#ifndef ROBOTSCENE_CONTROLAUTHORITY_H
#define ROBOTSCENE_CONTROLAUTHORITY_H

/// @file ControlAuthority.h
/// @brief 每机器人实例的控制权威：Kernel 程序 vs 外置控制器

#include "robot_scene_global.h"

#include <QHash>

enum class ROBOT_SCENE_API ControlAuthorityKind
{
	Kernel = 0,
	External
};

class ROBOT_SCENE_API ControlAuthorityMap
{
public:
	void clear();
	void setAuthority(int robotInstanceIndex, ControlAuthorityKind kind);
	ControlAuthorityKind authority(int robotInstanceIndex) const;
	bool isExternal(int robotInstanceIndex) const;
	QList<int> externalInstances() const;

private:
	QHash<int, ControlAuthorityKind> m_map;
};

#endif // ROBOTSCENE_CONTROLAUTHORITY_H
