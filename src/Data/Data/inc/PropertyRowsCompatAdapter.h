#ifndef DATA_PROPERTYROWSCOMPATADAPTER_H
#define DATA_PROPERTYROWSCOMPATADAPTER_H

/// @file PropertyRowsCompatAdapter.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 旧 PropertyBag 与 BackendDataBase 位姿/颜色同步

#include "BackendDataBase.h"
#include "PropertyBag.h"

#include <string>

class BackendDataManager;
class FollowAttachmentComponent;

/// 旧 PropertyBag 与 BackendDataBase 位姿/颜色同步
namespace property_rows_compat
{
inline void syncTransformColorToBag(PropertyBag& bag, const BackendDataBase& data)
{
	if (!data.hasPoseProperty())
	{
		return;
	}
	const BackendVec3 p = data.pose();
	const BackendVec3 r = data.rotation();
	bag.set<double>("pose.x", p.x);
	bag.set<double>("pose.y", p.y);
	bag.set<double>("pose.z", p.z);
	bag.set<double>("rotation.x", r.x);
	bag.set<double>("rotation.y", r.y);
	bag.set<double>("rotation.z", r.z);
	// color 键仅写给真正有颜色属性的对象；Frame/CustomDevice 无颜色概念，
	// 写入 color.*=白 会污染 bag 并随工程持久化
	if (data.hasColorProperty())
	{
		const BackendColor c = data.color();
		bag.set<double>("color.r", static_cast<double>(c.r));
		bag.set<double>("color.g", static_cast<double>(c.g));
		bag.set<double>("color.b", static_cast<double>(c.b));
		bag.set<double>("color.a", static_cast<double>(c.a));
	}
	const std::string frame = (data.poseReferenceFrame() == BackendPoseReferenceFrame::Parent) ? "parent" : "world";
	bag.set<std::string>("pose.frame", frame);
}
} // namespace property_rows_compat

#endif // DATA_PROPERTYROWSCOMPATADAPTER_H
