/// @file PluginPropertyBindingSelfTest.cpp
/// @brief 插件 Binding 注册表 mock 回归（Debug）

#include "PluginPropertyBindingSelfTest.h"

#include "BackendExternalPropertySchemaRegistry.h"
#include "BackendPropertySchema.h"
#include "BackendPropertyVisualAspect.h"
#include "PluginBackendMeta.h"
#include "PluginPropertyBindingRegistry.h"
#include "RunLogger.h"

#include "../../Data/PropertyCore/inc/PropertyTypes.h"

namespace
{
constexpr const char* kMockClass = "PluginBindingSelfTestMock";

class MockPluginObject final : public IPluginBackendObject
{
public:
	std::string id() const override { return "mock-id"; }
	std::string name() const override { return "mock"; }
	std::string className() const override { return kMockClass; }
	std::string propertyRowsJson() const override { return "[]"; }
	bool applyPropertyChange(const std::string&, const std::string&) override { return false; }

	bool visibleFlag = true;
};

std::string formatVisible(const IPluginBackendObject* obj)
{
	const auto* m = dynamic_cast<const MockPluginObject*>(obj);
	return (m && m->visibleFlag) ? "true" : "false";
}

bool applyVisible(IPluginBackendObject* obj, const char* valueUtf8, std::string* err)
{
	(void)err;
	auto* m = dynamic_cast<MockPluginObject*>(obj);
	if (!m || !valueUtf8)
	{
		return false;
	}
	const std::string v(valueUtf8);
	m->visibleFlag = (v == "true" || v == "1");
	return true;
}

void fail(std::vector<std::string>* failures, const std::string& msg)
{
	if (failures)
	{
		failures->push_back(msg);
	}
	RunLogger::warn("[PluginPropertyBindingSelfTest] " + msg);
}
} // namespace

bool runPluginPropertyBindingSelfTest(std::vector<std::string>* failures)
{
	bool ok = true;
	const auto check = [&](bool cond, const std::string& msg)
	{
		if (!cond)
		{
			ok = false;
			fail(failures, msg);
		}
	};

	plugin_property_binding_registry::unregisterType(kMockClass);

	PluginPropertyBindingEntry e{};
	e.key = "plugin.mock.visible";
	e.label = "Mock visible";
	e.propertyType = static_cast<int>(property_core::PropertyType::Bool);
	e.editable = true;
	e.semanticFlags = static_cast<unsigned>(property_core::PropertySemanticFlags::AffectsVisibility);
	e.formatValue = formatVisible;
	e.applyValue = applyVisible;

	plugin_property_binding_registry::Entry entry;
	entry.supportsTransform = false;
	entry.supportsVisibility = true;
	entry.bindings.push_back(e);
	plugin_property_binding_registry::registerType(kMockClass, std::move(entry));

	const auto& schema = backend_property_schema::schemaForBackendClassName(kMockClass);
	check(schema.find("plugin.mock.visible") != nullptr, "schema missing plugin.mock.visible");
	check(backend_property_schema::visualAspectsForPropertyKey(kMockClass, "plugin.mock.visible") ==
			  backend_property_schema::kVisualAspectVisibility,
		  "aspect Visibility");

	MockPluginObject mock;
	std::string err;
	check(e.applyValue(&mock, "false", &err), "apply visible false");
	check(!mock.visibleFlag, "visible flag");
	check(formatVisible(&mock) == "false", "format visible");

	plugin_property_binding_registry::unregisterType(kMockClass);
	backend_external_property_schema::invalidateSchemaCache(kMockClass);

	if (ok)
	{
		RunLogger::info("[PluginPropertyBindingSelfTest] PASS");
	}
	return ok;
}
