#ifndef DATA_BACKENDPROPERTYSCHEMA_H
#define DATA_BACKENDPROPERTYSCHEMA_H

/// @file BackendPropertySchema.h
/// @brief 后端对象属性 schema 拼装与语义位标注

#include "../../PropertyCore/inc/PropertySchema.h"
#include "BackendDataBase.h"

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
		s.descriptors.push_back({"mesh.triangle_count", "Triangles", PropertyType::Int, 0, false});
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
	if (className == "PointCloudBackendData")
	{
		return pointCloudBackendSchema();
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
	return followAttachmentBackendPropertySchema().find(key);
}

} // namespace backend_property_schema

#endif // DATA_BACKENDPROPERTYSCHEMA_H
