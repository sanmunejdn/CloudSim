#include "CloudSimBootstrap.h"

#include "CloudSimHost.h"
#include "EventHub.h"
#include "ICloudSimContext.h"
#include "IDocumentScope.h"
#include "IRenderView.h"

#include <QWidget>

#include <memory>

namespace cloudsim::core {

class ApplicationContextImpl final : public ICloudSimContext
{
public:
	ApplicationContextImpl(std::unique_ptr<IRenderViewFactory> renderFactory)
		: m_renderFactory(std::move(renderFactory))
	{
	}

	EventHub& events() override { return m_events; }
	IRenderViewFactory& renderFactory() override { return *m_renderFactory; }

	std::unique_ptr<IDocumentScope> createDocumentScope(QWidget* parent, const QString& documentId) override
	{
		return cloudsim::host::createDocumentHost(parent, m_events, documentId);
	}

	IDocumentScope* activeScope() const override { return m_activeScope; }
	void setActiveScope(IDocumentScope* scope) override { m_activeScope = scope; }

private:
	EventHub m_events;
	std::unique_ptr<IRenderViewFactory> m_renderFactory;
	IDocumentScope* m_activeScope = nullptr;
};

} // namespace cloudsim::core

namespace {

std::unique_ptr<cloudsim::core::ICloudSimContext> g_applicationContext;

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
	return std::make_unique<cloudsim::core::ApplicationContextImpl>(cloudsim::host::createHostRenderViewFactory());
}
