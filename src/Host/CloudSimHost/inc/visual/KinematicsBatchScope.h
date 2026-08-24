#ifndef CLOUDSIMHOST_KINEMATICSBATCHSCOPE_H
#define CLOUDSIMHOST_KINEMATICSBATCHSCOPE_H

/// @file KinematicsBatchScope.h
/// @brief FK/关节批量写 worldMatrix，批末一次 flushTransform

#include "cloudsim_host_global.h"

namespace cloudsim::host
{
class DocumentHost;

/// RAII：批内累积 Transform 脏标记，析构时父序 flush
class CLOUDSIM_HOST_EXPORT KinematicsBatchScope
{
public:
	explicit KinematicsBatchScope(DocumentHost& host);
	~KinematicsBatchScope();

	KinematicsBatchScope(const KinematicsBatchScope&) = delete;
	KinematicsBatchScope& operator=(const KinematicsBatchScope&) = delete;

private:
	DocumentHost& m_host;
	bool m_active = false;
};

} // namespace cloudsim::host

#endif // CLOUDSIMHOST_KINEMATICSBATCHSCOPE_H
