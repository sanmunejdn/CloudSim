#ifndef CLOUDSIMHOST_MULTIVIEWTYPES_H
#define CLOUDSIMHOST_MULTIVIEWTYPES_H

/// @file MultiViewTypes.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 多视角分屏布局枚举（Core / Desktop 共用，无 OSG 依赖）

namespace cloudsim::host
{
/// 文档内分屏布局（最多四格）
enum class MultiViewLayout
{
	Single = 0,
	SplitH = 1,
	SplitV = 2,
	Quad = 3
};
} // namespace cloudsim::host

#endif // CLOUDSIMHOST_MULTIVIEWTYPES_H
