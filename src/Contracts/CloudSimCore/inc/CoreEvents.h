#ifndef CLOUDSIMCORE_COREEVENTS_H
#define CLOUDSIMCORE_COREEVENTS_H

/// @file CoreEvents.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 选中来源

#include "CoreTypes.h"

namespace cloudsim::core
{
/// 选中来源
enum class SelectionSource
{
	Tree,
	OsgPick,
	Programmatic
};

/// 选中变更
struct SelectionChangedEvent
{
	QString documentId;
	ObjectId primaryId;
	SelectionSource source = SelectionSource::Programmatic;
};

/// 位姿落盘
struct PoseCommittedEvent
{
	QString documentId;
	ObjectId objectId;
	PoseDto pose;
};

/// 后端对象注册
struct BackendObjectRegisteredEvent
{
	QString documentId;
	ObjectId objectId;
	QString className;
};

/// 后端对象移除
struct BackendObjectRemovedEvent
{
	QString documentId;
	ObjectId objectId;
};

/// 工程加载完成
struct ProjectLoadedEvent
{
	QString documentId;
	QString projectPath;
};

/// 机器人 FK 已应用
struct RobotKinematicsAppliedEvent
{
	QString documentId;
	ObjectId sceneRootBackendId;
	QVector<double> jointAnglesRad;
};

} // namespace cloudsim::core

#endif // CLOUDSIMCORE_COREEVENTS_H
