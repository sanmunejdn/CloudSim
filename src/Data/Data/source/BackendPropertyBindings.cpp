/// @file BackendPropertyBindings.cpp
/// @brief 标准包 + Binding 收集 / schema 缓存

#include "BackendPropertyBinding.h"
#include "BackendPropertyBindingSelfTest.h"
#include "BackendExternalPropertySchemaRegistry.h"

#include "BackendDataBase.h"
#include "BackendPropertyRow.h"
#include "BackendRegistry.h"
#include "BackendRegistryBuiltins.h"
#include "BackendTypeIdentity.h"
#include "CustomDeviceBackendData.h"
#include "FrameBackendData.h"
#include "MeshBackendData.h"
#include "RunLogger.h"

#include "../../PropertyCore/inc/PropertyAttributeHelpers.h"
#include "../../PropertyCore/inc/PropertyTypes.h"

#include <cctype>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <unordered_set>

namespace
{
using property_core::PropertyDescriptor;
using property_core::PropertySemanticFlags;
using property_core::PropertyType;

PropertyDescriptor makeDesc(const char* key, const char* label, PropertyType type, bool editable,
							PropertySemanticFlags flags)
{
	PropertyDescriptor d;
	d.key = key;
	d.label = label;
	d.type = type;
	d.editable = editable;
	d.semanticFlags = flags;
	return d;
}

std::string formatPoseX(const BackendDataBase& d)
{
	return property_core::formatDoubleFixed3(d.pose().x);
}
std::string formatPoseY(const BackendDataBase& d)
{
	return property_core::formatDoubleFixed3(d.pose().y);
}
std::string formatPoseZ(const BackendDataBase& d)
{
	return property_core::formatDoubleFixed3(d.pose().z);
}
std::string formatRotX(const BackendDataBase& d)
{
	return property_core::formatDoubleFixed3(d.rotation().x);
}
std::string formatRotY(const BackendDataBase& d)
{
	return property_core::formatDoubleFixed3(d.rotation().y);
}
std::string formatRotZ(const BackendDataBase& d)
{
	return property_core::formatDoubleFixed3(d.rotation().z);
}
std::string formatColorR(const BackendDataBase& d)
{
	return property_core::formatDoubleFixed3(static_cast<double>(d.color().r));
}
std::string formatColorG(const BackendDataBase& d)
{
	return property_core::formatDoubleFixed3(static_cast<double>(d.color().g));
}
std::string formatColorB(const BackendDataBase& d)
{
	return property_core::formatDoubleFixed3(static_cast<double>(d.color().b));
}
std::string formatColorA(const BackendDataBase& d)
{
	return property_core::formatDoubleFixed3(static_cast<double>(d.color().a));
}
std::string formatPoseFrame(const BackendDataBase& d)
{
	return d.poseReferenceFrame() == BackendPoseReferenceFrame::Parent ? "parent" : "world";
}
std::string formatVisible(const BackendDataBase& d)
{
	return d.isVisible() ? "true" : "false";
}

bool applyPoseComponent(BackendDataBase& d, char axis, const std::string& value, std::string* err)
{
	double v = 0.0;
	if (!property_core::parseStrictDouble(value, v, err))
	{
		return false;
	}
	BackendVec3 p = d.pose();
	if (axis == 'x')
	{
		p.x = v;
	}
	else if (axis == 'y')
	{
		p.y = v;
	}
	else
	{
		p.z = v;
	}
	d.setPose(p);
	return true;
}

bool applyRotComponent(BackendDataBase& d, char axis, const std::string& value, std::string* err)
{
	double v = 0.0;
	if (!property_core::parseStrictDouble(value, v, err))
	{
		return false;
	}
	BackendVec3 r = d.rotation();
	if (axis == 'x')
	{
		r.x = v;
	}
	else if (axis == 'y')
	{
		r.y = v;
	}
	else
	{
		r.z = v;
	}
	d.setRotation(r);
	return true;
}

bool applyColorComponent(BackendDataBase& d, char channel, const std::string& value, std::string* err)
{
	double v = 0.0;
	if (!property_core::parseStrictDouble(value, v, err))
	{
		return false;
	}
	BackendColor c = d.color();
	const float f = static_cast<float>(v);
	if (channel == 'r')
	{
		c.r = f;
	}
	else if (channel == 'g')
	{
		c.g = f;
	}
	else if (channel == 'b')
	{
		c.b = f;
	}
	else
	{
		c.a = f;
	}
	d.setColor(c);
	return true;
}

bool applyPoseX(BackendDataBase& d, const std::string& value, std::string* err)
{
	return applyPoseComponent(d, 'x', value, err);
}
bool applyPoseY(BackendDataBase& d, const std::string& value, std::string* err)
{
	return applyPoseComponent(d, 'y', value, err);
}
bool applyPoseZ(BackendDataBase& d, const std::string& value, std::string* err)
{
	return applyPoseComponent(d, 'z', value, err);
}
bool applyRotX(BackendDataBase& d, const std::string& value, std::string* err)
{
	return applyRotComponent(d, 'x', value, err);
}
bool applyRotY(BackendDataBase& d, const std::string& value, std::string* err)
{
	return applyRotComponent(d, 'y', value, err);
}
bool applyRotZ(BackendDataBase& d, const std::string& value, std::string* err)
{
	return applyRotComponent(d, 'z', value, err);
}
bool applyColorR(BackendDataBase& d, const std::string& value, std::string* err)
{
	return applyColorComponent(d, 'r', value, err);
}
bool applyColorG(BackendDataBase& d, const std::string& value, std::string* err)
{
	return applyColorComponent(d, 'g', value, err);
}
bool applyColorB(BackendDataBase& d, const std::string& value, std::string* err)
{
	return applyColorComponent(d, 'b', value, err);
}
bool applyColorA(BackendDataBase& d, const std::string& value, std::string* err)
{
	return applyColorComponent(d, 'a', value, err);
}

bool applyPoseFrame(BackendDataBase& d, const std::string& value, std::string* err)
{
	std::string frame;
	frame.reserve(value.size());
	for (unsigned char ch : value)
	{
		if (ch == ' ' || ch == '\t')
		{
			continue;
		}
		frame.push_back(static_cast<char>(std::tolower(ch)));
	}
	if (frame == "world")
	{
		d.setPoseReferenceFrame(BackendPoseReferenceFrame::World);
		d.propertyBag().set<std::string>("pose.frame", "world");
		return true;
	}
	if (frame == "parent")
	{
		d.setPoseReferenceFrame(BackendPoseReferenceFrame::Parent);
		d.propertyBag().set<std::string>("pose.frame", "parent");
		return true;
	}
	if (err)
	{
		*err = "pose.frame only supports 'world' or 'parent'.";
	}
	return false;
}

bool applyVisible(BackendDataBase& d, const std::string& value, std::string* err)
{
	bool on = false;
	if (!property_core::parseStrictBool(value, on, err))
	{
		return false;
	}
	d.setVisible(on);
	return true;
}

std::string formatMeshTriangleCount(const BackendDataBase& d)
{
	return std::to_string(d.geometryElementCount());
}

bool applyReadOnly(BackendDataBase&, const std::string&, std::string* err)
{
	if (err)
	{
		*err = "Property is read-only.";
	}
	return false;
}

std::string formatAxisLengthMm(const BackendDataBase& d)
{
	if (const auto* frame = dynamic_cast<const FrameBackendData*>(&d))
	{
		return property_core::formatDoubleFixed3(static_cast<double>(frame->axisLengthMm()));
	}
	if (const auto* device = dynamic_cast<const CustomDeviceBackendData*>(&d))
	{
		return property_core::formatDoubleFixed3(static_cast<double>(device->axisLengthMm()));
	}
	return "0.000";
}

bool applyAxisLengthMm(BackendDataBase& d, const std::string& value, std::string* err)
{
	double v = 0.0;
	if (!property_core::parseStrictDouble(value, v, err))
	{
		return false;
	}
	if (v <= 0.0)
	{
		if (err)
		{
			*err = "axisLengthMm must be positive.";
		}
		return false;
	}
	if (auto* frame = dynamic_cast<FrameBackendData*>(&d))
	{
		frame->setAxisLengthMm(static_cast<float>(v));
		return true;
	}
	if (auto* device = dynamic_cast<CustomDeviceBackendData*>(&d))
	{
		device->setAxisLengthMm(static_cast<float>(v));
		return true;
	}
	if (err)
	{
		*err = "axisLengthMm not supported on this object.";
	}
	return false;
}

void appendPosePack(std::vector<BackendPropertyBinding>& out)
{
	const auto xform = PropertySemanticFlags::AffectsBackendRootWorldXform;
	out.push_back({makeDesc("pose.frame", "Pose frame (world|parent)", PropertyType::String, true,
							 PropertySemanticFlags::None),
				   formatPoseFrame, applyPoseFrame});
	out.push_back({makeDesc("pose.x", "Pose X", PropertyType::Double, true, xform), formatPoseX, applyPoseX});
	out.push_back({makeDesc("pose.y", "Pose Y", PropertyType::Double, true, xform), formatPoseY, applyPoseY});
	out.push_back({makeDesc("pose.z", "Pose Z", PropertyType::Double, true, xform), formatPoseZ, applyPoseZ});
}

void appendRotationPack(std::vector<BackendPropertyBinding>& out)
{
	const auto xform = PropertySemanticFlags::AffectsBackendRootWorldXform;
	out.push_back(
		{makeDesc("rotation.x", "Rotation X (deg)", PropertyType::Double, true, xform), formatRotX, applyRotX});
	out.push_back(
		{makeDesc("rotation.y", "Rotation Y (deg)", PropertyType::Double, true, xform), formatRotY, applyRotY});
	out.push_back(
		{makeDesc("rotation.z", "Rotation Z (deg)", PropertyType::Double, true, xform), formatRotZ, applyRotZ});
}

void appendColorPack(std::vector<BackendPropertyBinding>& out)
{
	const auto color = PropertySemanticFlags::AffectsColorOnly;
	out.push_back({makeDesc("color.r", "Color R", PropertyType::Double, true, color), formatColorR, applyColorR});
	out.push_back({makeDesc("color.g", "Color G", PropertyType::Double, true, color), formatColorG, applyColorG});
	out.push_back({makeDesc("color.b", "Color B", PropertyType::Double, true, color), formatColorB, applyColorB});
	out.push_back({makeDesc("color.a", "Color A", PropertyType::Double, true, color), formatColorA, applyColorA});
}

void appendVisiblePack(std::vector<BackendPropertyBinding>& out)
{
	out.push_back({makeDesc("visible", "Visible", PropertyType::Bool, true, PropertySemanticFlags::AffectsVisibility),
				   formatVisible, applyVisible});
}

property_core::PropertySchema buildSchemaFromBindings(const std::string& className,
													  const std::vector<BackendPropertyBinding>& bindings)
{
	property_core::PropertySchema s;
	s.objectTypeId = "backend." + className;
	s.schemaVersion = 1;
	s.descriptors.reserve(bindings.size());
	for (const BackendPropertyBinding& b : bindings)
	{
		s.descriptors.push_back(b.desc);
	}
	return s;
}

} // namespace

namespace backend_property_binding
{
namespace
{
std::mutex& schemaCacheMutex()
{
	static std::mutex mu;
	return mu;
}
std::unordered_map<std::string, property_core::PropertySchema>& schemaCache()
{
	static std::unordered_map<std::string, property_core::PropertySchema> cache;
	return cache;
}
std::unordered_set<std::string>& schemaWarned()
{
	static std::unordered_set<std::string> warned;
	return warned;
}
} // namespace

void invalidateSchemaCacheForClass(const std::string& className)
{
	std::lock_guard<std::mutex> lock(schemaCacheMutex());
	schemaCache().erase(className);
	schemaWarned().erase(className);
}

void invalidateAllSchemaCaches()
{
	std::lock_guard<std::mutex> lock(schemaCacheMutex());
	schemaCache().clear();
	schemaWarned().clear();
}

std::vector<BackendPropertyBinding> collectBindings(const BackendDataBase& data)
{
	std::vector<BackendPropertyBinding> out;
	out.reserve(16U);
	if (data.hasPoseProperty())
	{
		appendPosePack(out);
	}
	if (data.hasRotationProperty())
	{
		appendRotationPack(out);
	}
	if (data.hasColorProperty())
	{
		appendColorPack(out);
	}
	appendVisiblePack(out);
	for (const BackendPropertyBinding& extra : data.extraPropertyBindings())
	{
		out.push_back(extra);
	}
	return out;
}

const property_core::PropertySchema& schemaForClassName(const std::string& className)
{
	ensureBackendBuiltinsRegistered();
	{
		std::lock_guard<std::mutex> lock(schemaCacheMutex());
		const auto it = schemaCache().find(className);
		if (it != schemaCache().end())
		{
			return it->second;
		}
	}

	std::string resolved = className;
	std::shared_ptr<BackendDataBase> obj = BackendRegistry::instance().create(className);
	if (!obj && backend_type::isMeshClassName(className))
	{
		obj = BackendRegistry::instance().create(backend_type::kClassModel);
		resolved = backend_type::kClassModel;
	}

	if (!obj)
	{
		property_core::PropertySchema ext;
		if (backend_external_property_schema::tryGetSchema(className, ext))
		{
			std::lock_guard<std::mutex> lock(schemaCacheMutex());
			auto inserted = schemaCache().emplace(className, std::move(ext));
			return inserted.first->second;
		}
		std::lock_guard<std::mutex> lock(schemaCacheMutex());
		if (schemaWarned().insert(className).second)
		{
			RunLogger::warn("[BackendPropertyBinding] unknown className \"" + className +
							"\", empty schema.");
		}
		auto inserted = schemaCache().emplace(className, property_core::PropertySchema{});
		return inserted.first->second;
	}

	property_core::PropertySchema built = buildSchemaFromBindings(resolved, collectBindings(*obj));
	property_core::PropertySchema ext;
	if (backend_external_property_schema::tryGetSchema(className, ext))
	{
		for (const property_core::PropertyDescriptor& d : ext.descriptors)
		{
			bool exists = false;
			for (const property_core::PropertyDescriptor& have : built.descriptors)
			{
				if (have.key == d.key)
				{
					exists = true;
					break;
				}
			}
			if (!exists)
			{
				built.descriptors.push_back(d);
			}
		}
	}

	{
		std::lock_guard<std::mutex> lock(schemaCacheMutex());
		auto inserted = schemaCache().emplace(className, std::move(built));
		if (!inserted.second)
		{
			return inserted.first->second;
		}
		if (resolved != className)
		{
			schemaCache().emplace(resolved, inserted.first->second);
		}
	}

#ifndef NDEBUG
	static int selfTestState = 0;
	if (selfTestState == 0)
	{
		selfTestState = 1;
		if (!runBackendPropertyBindingSelfTest(nullptr))
		{
			RunLogger::error("BackendPropertyBindingSelfTest failed.");
		}
		selfTestState = 2;
	}
#endif

	std::lock_guard<std::mutex> lock(schemaCacheMutex());
	return schemaCache().find(className)->second;
}

void appendBindingRows(const BackendDataBase& data, nlohmann::json& rows)
{
	for (const BackendPropertyBinding& b : collectBindings(data))
	{
		if (!b.formatValue)
		{
			continue;
		}
		backend_property_json::appendRow(rows, b.desc.key, b.desc.label, b.desc.editable, b.formatValue(data));
	}
}

bool applyBindingKey(BackendDataBase& data, const std::string& key, const std::string& value, std::string* errMsg)
{
	const std::vector<BackendPropertyBinding> bindings = collectBindings(data);
	for (const BackendPropertyBinding& b : bindings)
	{
		if (b.desc.key != key)
		{
			continue;
		}
		if (!b.desc.editable || !b.applyValue)
		{
			if (errMsg)
			{
				*errMsg = "Property is read-only for this object type.";
			}
			return false;
		}
		return b.applyValue(data, value, errMsg);
	}
	return false;
}

} // namespace backend_property_binding

// Mesh / Frame / CustomDevice 的 extra 表（供派生类返回引用）
namespace backend_property_binding_extras
{
const std::vector<BackendPropertyBinding>& meshExtras()
{
	static const std::vector<BackendPropertyBinding> k = []
	{
		std::vector<BackendPropertyBinding> v;
		v.push_back({makeDesc("mesh.triangle_count", "Triangles", PropertyType::Int, false, PropertySemanticFlags::None),
					 formatMeshTriangleCount, applyReadOnly});
		return v;
	}();
	return k;
}

const std::vector<BackendPropertyBinding>& axisLengthExtras()
{
	static const std::vector<BackendPropertyBinding> k = []
	{
		std::vector<BackendPropertyBinding> v;
		v.push_back({makeDesc("axisLengthMm", "Axis length (mm)", PropertyType::Double, true,
							  PropertySemanticFlags::AffectsGeometry),
					 formatAxisLengthMm, applyAxisLengthMm});
		return v;
	}();
	return k;
}

const std::vector<BackendPropertyBinding>& emptyExtras()
{
	static const std::vector<BackendPropertyBinding> k;
	return k;
}
} // namespace backend_property_binding_extras
