/// @file IWorkpieceWorldMatrixProvider.cpp
/// @brief BackendDataWorldMatrixProvider 实现

#include "IWorkpieceWorldMatrixProvider.h"

#include "BackendDataBase.h"
#include "BackendSpatial.h"

bool BackendDataWorldMatrixProvider::getWorldMatrix(const std::string& backendId, BackendMat4& out) const
{
	const auto data = m_query.getData(backendId);
	if (!data)
	{
		return false;
	}
	out = objectWorldMatrix(*data);
	return true;
}
