#ifndef CLOUDSIMUIASSETS_UIICONID_H
#define CLOUDSIMUIASSETS_UIICONID_H

/// @file UiIconId.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 语义化 UI 图标 ID，与 qrc basename 映射见 UiIcons.cpp

#include "uiassets_global.h"

/// 语义化 UI 图标 ID，与 qrc basename 映射见 UiIcons.cpp
enum class UiIconId
{
	NewDocument,
	OpenProject,
	SaveProject,
	OpenModel,
	OpenPointCloud,
	Exit,

	Undo,
	Redo,
	Delete,
	Clear,
	Add,
	Rename,
	Duplicate,

	Run,
	Stop,
	Export,
	Ptp,
	Line,
	TcpDragTeach,
	Wait,
	If,
	While,
	SetDo,
	SetAo,

	Apply,
	Reset,
	SaveTemplate,
	LoadTemplate,
	NewPathPlan,
	PickEdge,
	PickFace,
	Discretize,
	Refresh,
	FillRecipe,
	EmitProgram,

	ViewMode,
	ObjectSelect,
	PointPick,
	LinePick,
	FacePick,
	TransformWorld,
	TransformLocal,

	Send,
	Settings,
	RobotPlaceholder,
	Connect,
	Disconnect,
	Read,
	Write,
	ClearLog,
	SetActive,

	FocusCamera,
	Wireframe,
	Screenshot,

	Close,
	DockFloat,

	Count
};

#endif // CLOUDSIMUIASSETS_UIICONID_H
