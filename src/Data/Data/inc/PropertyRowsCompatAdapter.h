#pragma once

#include <string>

#include "BackendDataBase.h"
#include "PropertyBag.h"

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
	const BackendColor c = data.color();
	bag.set<double>("pose.x", p.x);
	bag.set<double>("pose.y", p.y);
	bag.set<double>("pose.z", p.z);
	bag.set<double>("rotation.x", r.x);
	bag.set<double>("rotation.y", r.y);
	bag.set<double>("rotation.z", r.z);
	bag.set<double>("color.r", static_cast<double>(c.r));
	bag.set<double>("color.g", static_cast<double>(c.g));
	bag.set<double>("color.b", static_cast<double>(c.b));
	bag.set<double>("color.a", static_cast<double>(c.a));
	const std::string frame = (data.poseReferenceFrame() == BackendPoseReferenceFrame::Parent) ? "parent" : "world";
	bag.set<std::string>("pose.frame", frame);
}
} // namespace property_rows_compat
