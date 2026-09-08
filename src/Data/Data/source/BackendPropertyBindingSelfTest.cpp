/// @file BackendPropertyBindingSelfTest.cpp
/// @brief Binding / schema / visible / axisLength 回归

#include "BackendPropertyBindingSelfTest.h"

#include "BackendPropertyBinding.h"
#include "BackendPropertySchema.h"
#include "BackendPropertyVisualAspect.h"
#include "BackendRegistryBuiltins.h"
#include "BackendTypeIdentity.h"
#include "BrepBackendData.h"
#include "CustomDeviceBackendData.h"
#include "FrameBackendData.h"
#include "MeshBackendData.h"
#include "ParametricBrepBackendData.h"
#include "PointCloudBackendData.h"
#include "RunLogger.h"

#include <cmath>
#include <set>
#include <string>
#include <vector>

namespace
{
void fail(std::vector<std::string>* failures, const std::string& msg)
{
	if (failures)
	{
		failures->push_back(msg);
	}
	RunLogger::warn("[BackendPropertyBindingSelfTest] " + msg);
}

bool snapshotHasKey(const nlohmann::json& rows, const std::string& key)
{
	if (!rows.is_array())
	{
		return false;
	}
	for (const auto& r : rows)
	{
		if (r.is_object() && r.value("key", std::string()) == key)
		{
			return true;
		}
	}
	return false;
}

std::set<std::string> bindingKeys(const BackendDataBase& obj)
{
	std::set<std::string> keys;
	for (const BackendPropertyBinding& b : backend_property_binding::collectBindings(obj))
	{
		keys.insert(b.desc.key);
	}
	return keys;
}

std::set<std::string> schemaKeys(const std::string& className)
{
	std::set<std::string> keys;
	for (const property_core::PropertyDescriptor& d :
		 backend_property_schema::schemaForBackendClassName(className).descriptors)
	{
		keys.insert(d.key);
	}
	return keys;
}

bool snapshotContainsAllBindingKeys(const BackendDataBase& obj)
{
	const nlohmann::json rows = obj.snapshotPropertyRows();
	for (const std::string& key : bindingKeys(obj))
	{
		if (!snapshotHasKey(rows, key))
		{
			return false;
		}
	}
	return true;
}

} // namespace

bool runBackendPropertyBindingSelfTest(std::vector<std::string>* failures)
{
	ensureBackendBuiltinsRegistered();
	bool ok = true;
	const auto check = [&](bool cond, const std::string& msg)
	{
		if (!cond)
		{
			ok = false;
			fail(failures, msg);
		}
	};

	PointCloudBackendData pc;
	MeshBackendData mesh;
	BrepBackendData brep;
	ParametricBrepBackendData parametric;
	FrameBackendData frame;
	CustomDeviceBackendData device;

	check(bindingKeys(pc) == schemaKeys(backend_type::kClassPointCloud), "PointCloud binding keys != schema");
	check(bindingKeys(mesh) == schemaKeys(backend_type::kClassModel), "Mesh binding keys != schema");
	check(bindingKeys(brep) == schemaKeys(backend_type::kClassBrepModel), "Brep binding keys != schema");
	check(bindingKeys(parametric) == schemaKeys(backend_type::kClassParametricBrep),
		  "ParametricBrep binding keys != schema");
	check(bindingKeys(frame) == schemaKeys(backend_type::kClassFrame), "Frame binding keys != schema");
	check(bindingKeys(device) == schemaKeys(backend_type::kClassCustomDevice), "CustomDevice binding keys != schema");

	check(snapshotContainsAllBindingKeys(pc), "PointCloud snapshot missing binding key");
	check(snapshotContainsAllBindingKeys(mesh), "Mesh snapshot missing binding key");
	check(snapshotContainsAllBindingKeys(brep), "Brep snapshot missing binding key");
	check(snapshotContainsAllBindingKeys(parametric), "ParametricBrep snapshot missing binding key");
	check(snapshotContainsAllBindingKeys(frame), "Frame snapshot missing binding key");
	check(snapshotContainsAllBindingKeys(device), "CustomDevice snapshot missing binding key");

	check(schemaKeys(backend_type::kClassFrame).count("color.r") == 0, "Frame schema must not have color.r");
	check(schemaKeys(backend_type::kClassFrame).count("axisLengthMm") == 1, "Frame schema missing axisLengthMm");
	check(schemaKeys(backend_type::kClassFrame).count("visible") == 1, "Frame schema missing visible");
	check(schemaKeys(backend_type::kClassCustomDevice).count("axisLengthMm") == 1,
		  "CustomDevice schema missing axisLengthMm");
	check(schemaKeys(backend_type::kClassModel).count("mesh.triangle_count") == 1, "Mesh schema missing triangle_count");
	check(schemaKeys(backend_type::kClassPointCloud).count("color.r") == 1, "PointCloud schema missing color.r");
	check(schemaKeys(backend_type::kClassPointCloud).count("visible") == 1, "PointCloud schema missing visible");

	const nlohmann::json frameRows = frame.snapshotPropertyRows();
	check(snapshotHasKey(frameRows, "visible"), "Frame snapshot missing visible");
	check(snapshotHasKey(frameRows, "axisLengthMm"), "Frame snapshot missing axisLengthMm");
	check(!snapshotHasKey(frameRows, "color.r"), "Frame snapshot must not have color.r");

	check(pc.isVisible(), "default visible");
	check(pc.applyPropertyChange("visible", "false", nullptr), "apply visible=false");
	check(!pc.isVisible(), "visible after apply");

	const std::uint64_t rev0 = frame.geometryRevision();
	check(frame.applyPropertyChange("axisLengthMm", "50", nullptr), "apply axisLengthMm");
	check(std::abs(frame.axisLengthMm() - 50.0f) < 1e-3f, "axisLengthMm value");
	check(frame.geometryRevision() > rev0, "axisLengthMm bumps geometryRevision");

	std::string err;
	check(!frame.applyPropertyChange("no.such.key", "1", &err), "unknown key should fail");
	check(err.find("Unknown") != std::string::npos, "unknown key error message");

	check(backend_property_schema::visualAspectsForPropertyKey(backend_type::kClassFrame, "color.r") == 0u,
		  "Frame color.r aspects must be 0");
	check(backend_property_schema::visualAspectsForPropertyKey(backend_type::kClassFrame, "axisLengthMm") ==
			  backend_property_schema::kVisualAspectGeometry,
		  "axisLengthMm -> Geometry");
	check(backend_property_schema::visualAspectsForPropertyKey(backend_type::kClassModel, "pose.x") ==
			  backend_property_schema::kVisualAspectTransform,
		  "pose.x -> Transform");
	check(backend_property_schema::visualAspectsForPropertyKey(backend_type::kClassModel, "no.such") == 0u,
		  "unknown key aspects 0");

	if (ok)
	{
		RunLogger::info("[BackendPropertyBindingSelfTest] PASS");
	}
	return ok;
}
