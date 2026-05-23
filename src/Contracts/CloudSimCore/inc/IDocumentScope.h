#pragma once

#include "cloudsim_core_global.h"

namespace cloudsim::core {

class IDataService;
class IRobotService;
class IRenderView;

/// One document tab: data + robot + render services.
class CLOUDSIM_CORE_EXPORT IDocumentScope
{
public:
	virtual ~IDocumentScope();

	virtual QString documentId() const = 0;
	virtual IDataService& data() = 0;
	virtual IRobotService& robot() = 0;
	virtual IRenderView& render() = 0;
};

} // namespace cloudsim::core
