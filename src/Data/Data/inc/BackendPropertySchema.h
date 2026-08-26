#ifndef DATA_BACKENDPROPERTYSCHEMA_H
#define DATA_BACKENDPROPERTYSCHEMA_H

/// @file BackendPropertySchema.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 后端对象属性 schema 拼装与语义位标注

#include "../../PropertyCore/inc/PropertySchema.h"
#include "BackendDataBase.h"
#include "BackendTypeIdentity.h"

#ifdef DATA_BUILD_DLL
// Data.dll 内部：直接链 RunLogger
#include "RunLogger.h"
#else
// 宿主工程（BackendVisual 等）无 RunLogger include 目录：
// 此处仅保证可编译；未知 className 告警在宿主侧静默（Data.dll 内部仍全量告警）
namespace BackendPropertySchemaDetail
{
inline void defaultWarn(const std::string&) {}
} // namespace BackendPropertySchemaDetail
#endif

#include <mutex>
#include <unordered_set>

/// 后端对象属性 schema 拼装与语义位标注
namespace backend_property_schema
{
inline void tagPoseRotationColorSemantics(std::vector<property_core::PropertyDescriptor>& descriptors)
{
	using namespace property_core;
	for (PropertyDescriptor& d : descriptors)
	{
		if (d.key == "pose.frame")
		{
			d.semanticFlags = PropertySemanticFlags::None;
		}
		else if (d.key == "visible")
		{
			d.semanticFlags = PropertySemanticFlags::AffectsVisibility;
		}
		else if (d.key.size() >= 5U && d.key.compare(0, 5, "pose.") == 0)
		{
			d.semanticFlags = PropertySemanticFlags::AffectsBackendRootWorldXform;
		}
		else if (d.key.size() >= 10U && d.key.compare(0, 10, "rotation.") == 0)
		{
			d.semanticFlags = PropertySemanticFlags::AffectsBackendRootWorldXform;
		}
		else if (d.key.size() >= 6U && d.key.compare(0, 6, "color.") == 0)
		{
			d.semanticFlags = PropertySemanticFlags::AffectsColorOnly;
		}
	}
}

inline std::vector<property_core::PropertyDescriptor> commonTransformDisplayPack()
{
	using namespace property_core;
	return {{"pose.frame", "Pose frame (world|parent)", PropertyType::String, std::string("world")},
			{"pose.x", "Pose X", PropertyType::Double, 0.0},
			{"pose.y", "Pose Y", PropertyType::Double, 0.0},
			{"pose.z", "Pose Z", PropertyType::Double, 0.0},
			{"rotation.x", "Rotation X (deg)", PropertyType::Double, 0.0},
			{"rotation.y", "Rotation Y (deg)", PropertyType::Double, 0.0},
			{"rotation.z", "Rotation Z (deg)", PropertyType::Double, 0.0},
			{"color.r", "Color R", PropertyType::Double, 1.0},
			{"color.g", "Color G", PropertyType::Double, 1.0},
			{"color.b", "Color B", PropertyType::Double, 1.0},
			{"color.a", "Color A", PropertyType::Double, 1.0}};
}

inline const property_core::PropertySchema& pointCloudBackendSchema()
{
	using namespace property_core;
	static const PropertySchema schema = []()
	{
		PropertySchema s;
		s.objectTypeId = "backend.point_cloud";
		s.schemaVersion = 1;
		s.descriptors = commonTransformDisplayPack();
		tagPoseRotationColorSemantics(s.descriptors);
		s.descriptors.push_back({"visible", "Visible", PropertyType::Bool, true});
		for (PropertyDescriptor& d : s.descriptors)
		{
			if (d.key == "visible")
			{
				d.semanticFlags = PropertySemanticFlags::AffectsVisibility;
			}
		}
		return s;
	}();
	return schema;
}

inline const property_core::PropertySchema& meshBackendSchema()
{
	using namespace property_core;
	static const PropertySchema schema = []()
	{
		PropertySchema s;
		s.objectTypeId = "backend.mesh";
		s.schemaVersion = 1;
		s.descriptors = commonTransformDisplayPack();
		tagPoseRotationColorSemantics(s.descriptors);
		s.descriptors.push_back({"visible", "Visible", PropertyType::Bool, true});
		s.descriptors.push_back({"mesh.triangle_count", "Triangles", PropertyType::Int, 0, false});
		for (PropertyDescriptor& d : s.descriptors)
		{
			if (d.key == "visible")
			{
				d.semanticFlags = PropertySemanticFlags::AffectsVisibility;
			}
		}
		return s;
	}();
	return schema;
}

inline const property_core::PropertySchema& frameBackendSchema()
{
	using namespace property_core;
	static const PropertySchema schema = []()
	{
		PropertySchema s;
		s.objectTypeId = "backend.frame";
		s.schemaVersion = 1;
		s.descriptors = commonTransformDisplayPack();
		tagPoseRotationColorSemantics(s.descriptors);
		s.descriptors.push_back({"visible", "Visible", PropertyType::Bool, true});
		s.descriptors.push_back({"axisLengthMm", "Axis length (mm)", PropertyType::Double, 100.0});
		for (PropertyDescriptor& d : s.descriptors)
		{
			if (d.key == "visible")
			{
				d.semanticFlags = PropertySemanticFlags::AffectsVisibility;
			}
			else if (d.key == "axisLengthMm")
			{
				d.semanticFlags = PropertySemanticFlags::AffectsGeometry;
			}
		}
		return s;
	}();
	return schema;
}

inline const property_core::PropertySchema& customDeviceBackendSchema()
{
	using namespace property_core;
	static const PropertySchema schema = []()
	{
		PropertySchema s;
		s.objectTypeId = "backend.custom_device";
		s.schemaVersion = 1;
		s.descriptors = commonTransformDisplayPack();
		tagPoseRotationColorSemantics(s.descriptors);
		s.descriptors.push_back({"visible", "Visible", PropertyType::Bool, true});
		s.descriptors.push_back({"axisLengthMm", "Axis length (mm)", PropertyType::Double, 80.0});
		for (PropertyDescriptor& d : s.descriptors)
		{
			if (d.key == "visible")
			{
				d.semanticFlags = PropertySemanticFlags::AffectsVisibility;
			}
			else if (d.key == "axisLengthMm")
			{
				d.semanticFlags = PropertySemanticFlags::AffectsGeometry;
			}
		}
		return s;
	}();
	return schema;
}

inline const property_core::PropertySchema& brepBackendSchema()
{
	using namespace property_core;
	static const PropertySchema schema = []()
	{
		PropertySchema s;
		s.objectTypeId = "backend.brep";
		s.schemaVersion = 1;
		s.descriptors = commonTransformDisplayPack();
		tagPoseRotationColorSemantics(s.descriptors);
		s.descriptors.push_back({"visible", "Visible", PropertyType::Bool, true});
		for (PropertyDescriptor& d : s.descriptors)
		{
			if (d.key == "visible")
			{
				d.semanticFlags = PropertySemanticFlags::AffectsVisibility;
			}
		}
		return s;
	}();
	return schema;
}

inline const property_core::PropertySchema& followAttachmentBackendPropertySchema()
{
	using namespace property_core;
	static const PropertySchema schema = []()
	{
		PropertySchema s;
		s.objectTypeId = "backend.component.follow_attachment";
		s.schemaVersion = 1;
		PropertyDescriptor d;
		d.key = "follow.targetName";
		d.label = "Follow: target object name";
		d.type = PropertyType::String;
		d.editable = true;
		d.semanticFlags = PropertySemanticFlags::AffectsFollowConstraintGraph;
		s.descriptors.push_back(std::move(d));
		return s;
	}();
	return schema;
}

inline const property_core::PropertySchema& schemaForBackendClassName(const std::string& className)
{
	if (backend_type::isPointCloudClassName(className))
	{
		return pointCloudBackendSchema();
	}
	if (className == backend_type::kClassFrame)
	{
		return frameBackendSchema();
	}
	if (className == backend_type::kClassCustomDevice)
	{
		return customDeviceBackendSchema();
	}
	if (className == backend_type::kClassBrepModel || className == backend_type::kClassParametricBrep)
	{
		return brepBackendSchema();
	}
	if (!backend_type::isMeshClassName(className))
	{
		// 未知 className 落到 mesh schema 易掩盖拼写错误；告警一次即可
		static std::unordered_set<std::string> warned;
		static std::mutex warnedMutex;
		{
			std::lock_guard<std::mutex> lock(warnedMutex);
			if (warned.insert(className).second)
			{
#ifdef DATA_BUILD_DLL
			RunLogger::warn("[BackendPropertySchema] unknown className \"" + className +
							"\", fallback to mesh schema.");
#else
			BackendPropertySchemaDetail::defaultWarn(className);
#endif
			}
		}
	}
	return meshBackendSchema();
}

inline const property_core::PropertyDescriptor* findAnyBackendPropertyDescriptor(const std::string& key)
{
	if (const auto* descriptor = pointCloudBackendSchema().find(key))
	{
		return descriptor;
	}
	if (const auto* descriptor = meshBackendSchema().find(key))
	{
		return descriptor;
	}
	if (const auto* descriptor = frameBackendSchema().find(key))
	{
		return descriptor;
	}
	if (const auto* descriptor = customDeviceBackendSchema().find(key))
	{
		return descriptor;
	}
	if (const auto* descriptor = brepBackendSchema().find(key))
	{
		return descriptor;
	}
	return followAttachmentBackendPropertySchema().find(key);
}

} // namespace backend_property_schema

#endif // DATA_BACKENDPROPERTYSCHEMA_H
