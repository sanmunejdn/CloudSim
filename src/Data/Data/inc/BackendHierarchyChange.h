#pragma once

#include "data_global.h"

#include <functional>
#include <string>

/// Structural changes to the backend object graph (not property/pose edits).
enum class BackendHierarchyChangeKind
{
	DataRegistered,
	DataUnregistered,
	EdgeAttached,
	EdgeDetached,
	AllCleared
};

/// \c parentId / \c childId interpretation depends on \ref kind:
/// - DataRegistered: \c childId is the new backend object id; \c parentId empty.
/// - DataUnregistered: \c childId is the removed backend object id; \c parentId empty.
/// - EdgeAttached / EdgeDetached: parent/child edge endpoints.
/// - AllCleared: both ids empty.
struct DATA_EXPORT BackendHierarchyChangeEvent
{
	BackendHierarchyChangeKind kind = BackendHierarchyChangeKind::AllCleared;
	std::string parentId;
	std::string childId;
};

using BackendHierarchyObserver = std::function<void(const BackendHierarchyChangeEvent&)>;
