#include "MeshTrajectorySession.h"

#include <algorithm>
#include <unordered_set>

namespace
{

void sortUnique(std::vector<int>& indices)
{
	std::sort(indices.begin(), indices.end());
	indices.erase(std::unique(indices.begin(), indices.end()), indices.end());
}

} // namespace

bool MeshTrajectorySession::beginMesh(
	const std::string& backendIdUtf8,
	const std::vector<float>& triangleSoup)
{
	if (triangleSoup.size() < 9U)
	{
		return false;
	}
	m_backendId = backendIdUtf8;
	m_triangleSoup = triangleSoup;
	m_selectedTriangles.clear();
	m_undoStack.clear();
	m_redoStack.clear();
	m_spec = geoalgo::MeshTrajectorySpec{};
	m_spec.workpiece.backendIdUtf8 = backendIdUtf8;
	m_spec.trajectoryId = "mesh_traj";
	syncRegionToSpec();
	return true;
}

bool MeshTrajectorySession::applyTriangleSelection(
	const std::vector<int>& triangleIndices,
	MeshTrajectorySelectionMode mode)
{
	if (m_triangleSoup.empty())
	{
		return false;
	}
	const int triCount = static_cast<int>(m_triangleSoup.size() / 9U);
	pushUndo();
	std::vector<int> next = m_selectedTriangles;
	if (mode == MeshTrajectorySelectionMode::Replace)
	{
		next.clear();
	}
	std::unordered_set<int> set(next.begin(), next.end());
	for (int ti : triangleIndices)
	{
		if (ti < 0 || ti >= triCount)
		{
			continue;
		}
		if (mode == MeshTrajectorySelectionMode::Toggle)
		{
			if (set.count(ti) != 0U)
			{
				set.erase(ti);
			}
			else
			{
				set.insert(ti);
			}
		}
		else if (mode == MeshTrajectorySelectionMode::Subtract)
		{
			set.erase(ti);
		}
		else
		{
			set.insert(ti);
		}
	}
	next.assign(set.begin(), set.end());
	sortUnique(next);
	m_selectedTriangles = std::move(next);
	syncRegionToSpec();
	return true;
}

bool MeshTrajectorySession::clearSelection()
{
	if (m_selectedTriangles.empty())
	{
		return true;
	}
	pushUndo();
	m_selectedTriangles.clear();
	syncRegionToSpec();
	return true;
}

bool MeshTrajectorySession::invertSelection()
{
	if (m_triangleSoup.empty())
	{
		return false;
	}
	pushUndo();
	const int triCount = static_cast<int>(m_triangleSoup.size() / 9U);
	std::unordered_set<int> set(m_selectedTriangles.begin(), m_selectedTriangles.end());
	std::vector<int> next;
	next.reserve(static_cast<std::size_t>(triCount));
	for (int ti = 0; ti < triCount; ++ti)
	{
		if (set.count(ti) == 0U)
		{
			next.push_back(ti);
		}
	}
	m_selectedTriangles = std::move(next);
	syncRegionToSpec();
	return true;
}

bool MeshTrajectorySession::undo()
{
	if (m_undoStack.empty())
	{
		return false;
	}
	m_redoStack.push_back(m_selectedTriangles);
	m_selectedTriangles = std::move(m_undoStack.back());
	m_undoStack.pop_back();
	syncRegionToSpec();
	return true;
}

bool MeshTrajectorySession::redo()
{
	if (m_redoStack.empty())
	{
		return false;
	}
	m_undoStack.push_back(m_selectedTriangles);
	m_selectedTriangles = std::move(m_redoStack.back());
	m_redoStack.pop_back();
	syncRegionToSpec();
	return true;
}

MeshTrajectorySessionSummary MeshTrajectorySession::summary() const
{
	MeshTrajectorySessionSummary s;
	s.backendIdUtf8 = m_backendId;
	s.triangleCount = static_cast<int>(m_triangleSoup.size() / 9U);
	s.selectedTriangleCount = static_cast<int>(m_selectedTriangles.size());
	return s;
}

bool MeshTrajectorySession::generateRawPath(geoalgo::RawPath& outPath, std::string* errMsg) const
{
	geoalgo::MeshTrajectorySpec spec = m_spec;
	spec.workpiece.backendIdUtf8 = m_backendId;
	spec.region.triangleIndices = m_selectedTriangles;
	return geoalgo::generateMeshTrajectory(spec, m_triangleSoup, outPath, errMsg);
}

std::string MeshTrajectorySession::specJsonUtf8() const
{
	std::string json;
	geoalgo::MeshTrajectorySpec specCopy = m_spec;
	specCopy.region.triangleIndices = m_selectedTriangles;
	(void)geoalgo::meshTrajectorySpecToJson(specCopy, json);
	return json;
}

void MeshTrajectorySession::pushUndo()
{
	m_undoStack.push_back(m_selectedTriangles);
	m_redoStack.clear();
}

void MeshTrajectorySession::syncRegionToSpec()
{
	m_spec.region.triangleIndices = m_selectedTriangles;
}
