#pragma once

#include "CoreTypes.h"
#include "cloudsim_core_global.h"

#include <functional>
#include <memory>

class QWidget;

namespace cloudsim::core {

using PickHandler = std::function<void(const ObjectId& backendId)>;

/// OSG-free render viewport for one document.
class CLOUDSIM_CORE_EXPORT IRenderView
{
public:
	virtual ~IRenderView() = default;

	virtual QWidget* widget() = 0;
	virtual const QWidget* widget() const = 0;

	virtual void setWorldMatrix(const ObjectId& id, const Mat4& columnMajor) = 0;
	virtual bool getWorldMatrix(const ObjectId& id, Mat4& outColumnMajor) const = 0;

	virtual void setVisible(const ObjectId& id, bool visible) = 0;
	virtual void removeVisual(const ObjectId& id) = 0;
	virtual bool hasVisualBranch(const ObjectId& id) const = 0;

	virtual bool tryGetModelCenterMm(const ObjectId& id, double& outCx, double& outCy, double& outCz) const = 0;

	virtual void setPickHandler(PickHandler handler) = 0;
	virtual void clearPickHandler() = 0;

	virtual void requestRedraw() = 0;
};

class CLOUDSIM_CORE_EXPORT IRenderViewFactory
{
public:
	virtual ~IRenderViewFactory() = default;
	virtual std::unique_ptr<IRenderView> createView(QWidget* parent) = 0;
};

} // namespace cloudsim::core
