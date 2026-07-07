#pragma once

#include "mesh_trajectory_sdk_global.h"

#include "MeshTrajectoryTypes.h"

#include <MeshTrajectory.h>

#include <string>
#include <vector>

class MESH_TRAJECTORY_SDK_EXPORT MeshTrajectorySession
{
public:
	MeshTrajectorySession() = default;

	bool beginMesh(
		const std::string& backendIdUtf8,
		const std::vector<float>& triangleSoup);

	const std::string& backendIdUtf8() const { return m_backendId; }
	const std::vector<float>& triangleSoup() const { return m_triangleSoup; }
	const std::vector<int>& selectedTriangleIndices() const { return m_selectedTriangles; }

	geoalgo::MeshTrajectorySpec& spec() { return m_spec; }
	const geoalgo::MeshTrajectorySpec& spec() const { return m_spec; }

	void setSpec(const geoalgo::MeshTrajectorySpec& spec) { m_spec = spec; }

	bool applyTriangleSelection(
		const std::vector<int>& triangleIndices,
		MeshTrajectorySelectionMode mode);

	bool clearSelection();
	bool invertSelection();

	bool undo();
	bool redo();
	bool canUndo() const { return !m_undoStack.empty(); }
	bool canRedo() const { return !m_redoStack.empty(); }

	MeshTrajectorySessionSummary summary() const;

	bool generateRawPath(geoalgo::RawPath& outPath, std::string* errMsg = nullptr) const;

	std::string specJsonUtf8() const;

private:
	void pushUndo();
	void syncRegionToSpec();

	std::string m_backendId;
	std::vector<float> m_triangleSoup;
	std::vector<int> m_selectedTriangles;
	geoalgo::MeshTrajectorySpec m_spec;

	std::vector<std::vector<int>> m_undoStack;
	std::vector<std::vector<int>> m_redoStack;
};
