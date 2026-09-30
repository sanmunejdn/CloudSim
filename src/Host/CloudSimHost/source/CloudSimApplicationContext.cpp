/// @file CloudSimApplicationContext.cpp
/// @brief 应用上下文组合根

#include "CloudSimBootstrap.h"
#include "CloudSimHost.h"
#include "EventHub.h"
#include "ICloudSimContext.h"
#include "IDocumentScope.h"
#include "IRenderView.h"
#include "NullCoreServices.h"
#include "BackendComponentCodecBuiltins.h"
#include "BackendComponentCodecRegistry.h"
#include "BackendRegistry.h"
#include "BackendRegistryBuiltins.h"
#include "BackendVisualRegistry.h"
#include "FeatureDiscretizerBridge.h"
#include "FeatureDiscretizerConfigRegistry.h"
#include "FeatureDiscretizerRegistry.h"
#include "TrajectoryOpBridge.h"
#include "TrajectoryOpConfigRegistry.h"
#include "TrajectoryOpRegistry.h"
#include "import/GeometryFileImporterRegistry.h"

#include <QWidget>
#include <memory>

namespace cloudsim::core
{
// 进程级组合根
class ApplicationContextImpl final : public ICloudSimContext
{
public:
	ApplicationContextImpl(std::unique_ptr<IRenderViewFactory> renderFactory, bool headlessDocuments)
		: m_renderFactory(std::move(renderFactory)), m_headlessDocuments(headlessDocuments)
	{
		registerHostRegistries();
	}

	void registerHostRegistries()
	{
		auto backendReg = std::make_shared<BackendRegistry>();
		BackendRegistry::setProcessInstance(backendReg.get());
		m_services.registerService(backendReg);

		auto codecReg = std::make_shared<BackendComponentCodecRegistry>();
		BackendComponentCodecRegistry::setProcessInstance(codecReg.get());
		m_services.registerService(codecReg);

		auto visualReg = std::make_shared<BackendVisualRegistry>();
		BackendVisualRegistry::setProcessInstance(visualReg.get());
		m_services.registerService(visualReg);
		BackendVisualRegistry::ensureBuiltinsRegistered();

		ensureBackendBuiltinsRegistered();
		initBackendComponentCodecs();

		auto trajectoryReg = std::make_shared<trajectory_algo::TrajectoryOpRegistry>();
		RobotInstruction::setTrajectoryOpRegistry(trajectoryReg.get());
		m_services.registerService(trajectoryReg);

		auto trajectoryCfg = std::make_shared<trajectory_algo::TrajectoryOpConfigRegistry>();
		trajectory_algo::TrajectoryOpConfigRegistry::setProcessInstance(trajectoryCfg.get());
		m_services.registerService(trajectoryCfg);
		trajectory_algo::ensureTrajectoryOpBuiltinsRegistered();

		auto featureReg = std::make_shared<geoalgo::FeatureDiscretizerRegistry>();
		geoalgo::setFeatureDiscretizerRegistry(featureReg.get());
		m_services.registerService(featureReg);

		auto featureCfg = std::make_shared<geoalgo::FeatureDiscretizerConfigRegistry>();
		geoalgo::FeatureDiscretizerConfigRegistry::setProcessInstance(featureCfg.get());
		m_services.registerService(featureCfg);
		geoalgo::ensureFeatureDiscretizersRegistered();

		m_services.registerService(std::make_shared<cloudsim::host::GeometryFileImporterRegistry>());
	}

	EventHub& events() override { return m_events; }
	IRenderViewFactory& renderFactory() override { return *m_renderFactory; }
	ServiceRegistry& services() override { return m_services; }
	const ServiceRegistry& services() const override { return m_services; }

	std::unique_ptr<IDocumentScope> createDocumentScope(QWidget* parent, const QString& documentId) override
	{
		if (m_headlessDocuments)
		{
			return cloudsim::host::createHeadlessDocumentHost(m_events, documentId);
		}
		return cloudsim::host::createDocumentHost(parent, m_events, documentId);
	}

	IDocumentScope* activeScope() const override { return m_activeScope; }
	void setActiveScope(IDocumentScope* scope) override { m_activeScope = scope; }

private:
	EventHub m_events;
	ServiceRegistry m_services;
	std::unique_ptr<IRenderViewFactory> m_renderFactory;
	IDocumentScope* m_activeScope = nullptr;
	bool m_headlessDocuments = false;
};

} // namespace cloudsim::core

namespace
{
std::unique_ptr<cloudsim::core::ICloudSimContext> g_applicationContext; // main 注入单例

} // namespace

void cloudsimSetApplicationContext(std::unique_ptr<cloudsim::core::ICloudSimContext> context)
{
	g_applicationContext = std::move(context);
}

cloudsim::core::ICloudSimContext* cloudsimApplicationContext()
{
	return g_applicationContext.get();
}

std::unique_ptr<cloudsim::core::ICloudSimContext> cloudsimCreateApplicationContext()
{
	return std::make_unique<cloudsim::core::ApplicationContextImpl>(cloudsim::host::createHostRenderViewFactory(),
																	false);
}

std::unique_ptr<cloudsim::core::ICloudSimContext> cloudsimCreateHeadlessApplicationContext()
{
	return std::make_unique<cloudsim::core::ApplicationContextImpl>(cloudsim::core::makeNullRenderViewFactory(), true);
}
