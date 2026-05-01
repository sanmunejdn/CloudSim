#pragma once

#include "BackendDataBase.h"
#include "../../PropertyCore/inc/PropertySchema.h"

namespace backend_property_schema
{

inline std::vector<property_core::PropertyDescriptor> commonTransformDisplayPack()
{
	using namespace property_core;
	return {
		{ "pose.x", "Pose X", PropertyType::Double, 0.0 },
		{ "pose.y", "Pose Y", PropertyType::Double, 0.0 },
		{ "pose.z", "Pose Z", PropertyType::Double, 0.0 },
		{ "rotation.x", "Rotation X (deg)", PropertyType::Double, 0.0 },
		{ "rotation.y", "Rotation Y (deg)", PropertyType::Double, 0.0 },
		{ "rotation.z", "Rotation Z (deg)", PropertyType::Double, 0.0 },
		{ "color.r", "Color R", PropertyType::Double, 1.0 },
		{ "color.g", "Color G", PropertyType::Double, 1.0 },
		{ "color.b", "Color B", PropertyType::Double, 1.0 },
		{ "color.a", "Color A", PropertyType::Double, 1.0 }
	};
}

inline const property_core::PropertySchema& pointCloudBackendSchema()
{
	using namespace property_core;
	static const PropertySchema schema = []() {
		PropertySchema s;
		s.objectTypeId = "backend.point_cloud";
		s.schemaVersion = 1;
		s.descriptors = commonTransformDisplayPack();
		return s;
	}();
	return schema;
}

inline const property_core::PropertySchema& meshBackendSchema()
{
	using namespace property_core;
	static const PropertySchema schema = []() {
		PropertySchema s;
		s.objectTypeId = "backend.mesh";
		s.schemaVersion = 1;
		s.descriptors = commonTransformDisplayPack();
		s.descriptors.push_back({ "mesh.triangle_count", "Triangles", PropertyType::Int, 0, false });
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
	return meshBackendSchema().find(key);
}

} // namespace backend_property_schema
