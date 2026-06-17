#include "LabelingSession.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <random>
#include <sstream>

namespace
{

bool writeAsciiPly(const std::string& path, const std::vector<float>& xyz)
{
	const std::size_t n = xyz.size() / 3U;
	std::ofstream f(path);
	if (!f)
	{
		return false;
	}
	f << "ply\nformat ascii 1.0\nelement vertex " << n << "\n";
	f << "property float x\nproperty float y\nproperty float z\nend_header\n";
	for (std::size_t i = 0; i < n; ++i)
	{
		const std::size_t b = i * 3U;
		f << xyz[b] << ' ' << xyz[b + 1U] << ' ' << xyz[b + 2U] << '\n';
	}
	return true;
}

bool writeNpyInt64(const std::string& path, const std::vector<int>& labels)
{
	std::ofstream f(path, std::ios::binary);
	if (!f)
	{
		return false;
	}
	static const char kNpyMagic[6] = { '\x93', 'N', 'U', 'M', 'P', 'Y' };
	f.write(kNpyMagic, 6);
	const char version[2] = { '\x01', '\x00' };
	f.write(version, 2);
	const std::string header = "{'descr': '<i8', 'fortran_order': False, 'shape': (" + std::to_string(labels.size()) + ",), }";
	const std::size_t hlen = header.size() + 1U;
	const std::size_t padlen = (16U - ((8U + hlen) % 16U)) % 16U;
	std::string padded = header;
	padded.append(padlen, ' ');
	padded.push_back('\n');
	const std::uint16_t hl = static_cast<std::uint16_t>(hlen + padlen);
	f.write(reinterpret_cast<const char*>(&hl), 2);
	f.write(padded.data(), static_cast<std::streamsize>(padded.size()));
	for (int v : labels)
	{
		const std::int64_t iv = static_cast<std::int64_t>(v);
		f.write(reinterpret_cast<const char*>(&iv), 8);
	}
	return true;
}

} // namespace

bool LabelingSession::beginPointCloud(const std::vector<float>& xyz, const LabelingSessionConfig& config)
{
	if (xyz.size() < 3U || (xyz.size() % 3U) != 0U)
	{
		return false;
	}
	m_kind = LabelingGeometryKind::PointCloud;
	m_config = config;
	m_xyz = xyz;
	m_triangleSoup.clear();
	m_pointLabels.assign(xyz.size() / 3U, config.unlabeledClassId);
	m_triangleLabels.clear();
	m_activeClassId = config.classes.empty() ? 1 : config.classes.front().classId;
	m_undoStack.clear();
	m_redoStack.clear();
	return true;
}

bool LabelingSession::beginTriangleMesh(const std::vector<float>& triangleSoup, const LabelingSessionConfig& config)
{
	if (triangleSoup.size() < 9U || (triangleSoup.size() % 9U) != 0U)
	{
		return false;
	}
	m_kind = LabelingGeometryKind::TriangleMesh;
	m_config = config;
	m_triangleSoup = triangleSoup;
	m_xyz.clear();
	m_triangleLabels.assign(triangleSoup.size() / 9U, config.unlabeledClassId);
	m_pointLabels.clear();
	m_activeClassId = config.classes.empty() ? 1 : config.classes.front().classId;
	m_undoStack.clear();
	m_redoStack.clear();
	return true;
}

void LabelingSession::updateSessionConfig(const LabelingSessionConfig& config)
{
	m_config = config;
	if (findClass(m_activeClassId) == nullptr)
	{
		m_activeClassId = config.classes.empty() ? 1 : config.classes.front().classId;
	}
}

std::size_t LabelingSession::totalElements() const
{
	return m_kind == LabelingGeometryKind::PointCloud ? m_pointLabels.size() : m_triangleLabels.size();
}

std::size_t LabelingSession::labeledCount() const
{
	const auto& labels = m_kind == LabelingGeometryKind::PointCloud ? m_pointLabels : m_triangleLabels;
	std::size_t c = 0U;
	for (int v : labels)
	{
		if (v != m_config.unlabeledClassId)
		{
			++c;
		}
	}
	return c;
}

std::map<int, std::size_t> LabelingSession::classHistogram() const
{
	std::map<int, std::size_t> hist;
	const auto& labels = m_kind == LabelingGeometryKind::PointCloud ? m_pointLabels : m_triangleLabels;
	for (int v : labels)
	{
		++hist[v];
	}
	return hist;
}

const LabelingClassDef* LabelingSession::findClass(int classId) const
{
	for (const LabelingClassDef& c : m_config.classes)
	{
		if (c.classId == classId)
		{
			return &c;
		}
	}
	return nullptr;
}

int LabelingSession::resolveClassId(int classId, bool erase) const
{
	if (erase)
	{
		return m_config.unlabeledClassId;
	}
	return classId;
}

void LabelingSession::pushUndo(const LabelingUndoPatch& patch)
{
	m_undoStack.push_back(patch);
	m_redoStack.clear();
}

bool LabelingSession::applyPointLabels(const std::vector<std::size_t>& indices, int classId, bool erase)
{
	if (m_kind != LabelingGeometryKind::PointCloud)
	{
		return false;
	}
	const int target = resolveClassId(classId, erase);
	LabelingUndoPatch patch;
	for (std::size_t idx : indices)
	{
		if (idx >= m_pointLabels.size())
		{
			continue;
		}
		if (m_pointLabels[idx] == target)
		{
			continue;
		}
		patch.indices.push_back(idx);
		patch.previousLabels.push_back(m_pointLabels[idx]);
		m_pointLabels[idx] = target;
	}
	if (!patch.indices.empty())
	{
		pushUndo(patch);
	}
	return true;
}

bool LabelingSession::applyTriangleLabels(const std::vector<int>& triIndices, int classId, bool erase)
{
	if (m_kind != LabelingGeometryKind::TriangleMesh)
	{
		return false;
	}
	const int target = resolveClassId(classId, erase);
	LabelingUndoPatch patch;
	for (int tri : triIndices)
	{
		if (tri < 0 || static_cast<std::size_t>(tri) >= m_triangleLabels.size())
		{
			continue;
		}
		const std::size_t idx = static_cast<std::size_t>(tri);
		if (m_triangleLabels[idx] == target)
		{
			continue;
		}
		patch.indices.push_back(idx);
		patch.previousLabels.push_back(m_triangleLabels[idx]);
		m_triangleLabels[idx] = target;
	}
	if (!patch.indices.empty())
	{
		pushUndo(patch);
	}
	return true;
}

bool LabelingSession::importPointLabels(const std::vector<int>& labels, int numClasses)
{
	(void)numClasses;
	if (m_kind == LabelingGeometryKind::PointCloud)
	{
		if (labels.size() != m_pointLabels.size())
		{
			return false;
		}
		m_pointLabels = labels;
		m_undoStack.clear();
		m_redoStack.clear();
		return true;
	}
	// 网格：采样点标签多数投票到三角
	if (labels.empty())
	{
		return false;
	}
	std::vector<float> sampledXyz;
	std::vector<int> sampledLabels;
	if (!sampleMeshLabelsToPointCloud(m_triangleSoup, m_triangleLabels, static_cast<int>(labels.size()), sampledXyz, sampledLabels))
	{
		(void)sampledXyz;
	}
	if (labels.size() != sampledLabels.size())
	{
		// 直接按三角数匹配时回退：均匀分桶
		const std::size_t triCount = m_triangleLabels.size();
		if (labels.size() != triCount)
		{
			return false;
		}
		for (std::size_t i = 0; i < triCount; ++i)
		{
			m_triangleLabels[i] = labels[i];
		}
	}
	else
	{
		std::vector<int> voteCounts(m_triangleLabels.size(), 0);
		std::vector<std::map<int, int>> votes(m_triangleLabels.size());
		const std::size_t triCount = m_triangleLabels.size();
		std::mt19937 rng(42);
		std::uniform_int_distribution<int> triDist(0, static_cast<int>(triCount) - 1);
		for (std::size_t i = 0; i < labels.size(); ++i)
		{
			const int tri = triDist(rng) % static_cast<int>(triCount);
			++votes[static_cast<std::size_t>(tri)][labels[i]];
		}
		for (std::size_t t = 0; t < triCount; ++t)
		{
			int bestLabel = m_config.unlabeledClassId;
			int bestCount = 0;
			for (const auto& kv : votes[t])
			{
				if (kv.second > bestCount)
				{
					bestCount = kv.second;
					bestLabel = kv.first;
				}
			}
			if (bestCount > 0)
			{
				m_triangleLabels[t] = bestLabel;
			}
		}
	}
	m_undoStack.clear();
	m_redoStack.clear();
	return true;
}

bool LabelingSession::undo()
{
	if (m_undoStack.empty())
	{
		return false;
	}
	LabelingUndoPatch patch = m_undoStack.back();
	m_undoStack.pop_back();
	LabelingUndoPatch redoPatch;
	redoPatch.indices = patch.indices;
	auto& labels = m_kind == LabelingGeometryKind::PointCloud ? m_pointLabels : m_triangleLabels;
	for (std::size_t i = 0; i < patch.indices.size(); ++i)
	{
		const std::size_t idx = patch.indices[i];
		redoPatch.previousLabels.push_back(labels[idx]);
		labels[idx] = patch.previousLabels[i];
	}
	m_redoStack.push_back(redoPatch);
	return true;
}

bool LabelingSession::redo()
{
	if (m_redoStack.empty())
	{
		return false;
	}
	LabelingUndoPatch patch = m_redoStack.back();
	m_redoStack.pop_back();
	LabelingUndoPatch undoPatch;
	undoPatch.indices = patch.indices;
	auto& labels = m_kind == LabelingGeometryKind::PointCloud ? m_pointLabels : m_triangleLabels;
	for (std::size_t i = 0; i < patch.indices.size(); ++i)
	{
		const std::size_t idx = patch.indices[i];
		undoPatch.previousLabels.push_back(labels[idx]);
		labels[idx] = patch.previousLabels[i];
	}
	m_undoStack.push_back(undoPatch);
	return true;
}

void LabelingSession::buildPointCloudRgba(std::vector<float>& outRgba) const
{
	const std::size_t n = m_pointLabels.size();
	outRgba.resize(n * 4U);
	for (std::size_t i = 0; i < n; ++i)
	{
		const LabelingClassDef* cls = findClass(m_pointLabels[i]);
		float r = 0.5f;
		float g = 0.5f;
		float b = 0.5f;
		if (cls)
		{
			r = cls->colorRgb[0];
			g = cls->colorRgb[1];
			b = cls->colorRgb[2];
		}
		else if (m_pointLabels[i] == m_config.unlabeledClassId)
		{
			r = g = b = 0.7f;
		}
		const std::size_t b4 = i * 4U;
		outRgba[b4] = r;
		outRgba[b4 + 1U] = g;
		outRgba[b4 + 2U] = b;
		outRgba[b4 + 3U] = 1.f;
	}
}

void LabelingSession::buildMeshVertexRgb(std::vector<float>& outRgb) const
{
	const std::size_t vertCount = m_triangleSoup.size();
	outRgb.resize(vertCount);
	for (std::size_t tri = 0; tri < m_triangleLabels.size(); ++tri)
	{
		const LabelingClassDef* cls = findClass(m_triangleLabels[tri]);
		float r = 0.7f;
		float g = 0.7f;
		float b = 0.7f;
		if (cls)
		{
			r = cls->colorRgb[0];
			g = cls->colorRgb[1];
			b = cls->colorRgb[2];
		}
		for (int v = 0; v < 3; ++v)
		{
			const std::size_t b3 = tri * 9U + static_cast<std::size_t>(v) * 3U;
			outRgb[b3] = r;
			outRgb[b3 + 1U] = g;
			outRgb[b3 + 2U] = b;
		}
	}
}

bool LabelingSession::sampleMeshLabelsToPointCloud(
	const std::vector<float>& soup,
	const std::vector<int>& triLabels,
	int sampleCount,
	std::vector<float>& outXyz,
	std::vector<int>& outLabels)
{
	const std::size_t triCount = soup.size() / 9U;
	if (triCount == 0U || triLabels.size() != triCount || sampleCount <= 0)
	{
		return false;
	}
	outXyz.clear();
	outLabels.clear();
	outXyz.reserve(static_cast<std::size_t>(sampleCount) * 3U);
	outLabels.reserve(static_cast<std::size_t>(sampleCount));
	std::mt19937 rng(123);
	std::uniform_int_distribution<int> triDist(0, static_cast<int>(triCount) - 1);
	std::uniform_real_distribution<float> u01(0.f, 1.f);
	for (int i = 0; i < sampleCount; ++i)
	{
		const int tri = triDist(rng);
		float u = u01(rng);
		float v = u01(rng);
		if (u + v > 1.f)
		{
			u = 1.f - u;
			v = 1.f - v;
		}
		const float w = 1.f - u - v;
		const std::size_t b = static_cast<std::size_t>(tri) * 9U;
		const float x = soup[b] * u + soup[b + 3U] * v + soup[b + 6U] * w;
		const float y = soup[b + 1U] * u + soup[b + 4U] * v + soup[b + 7U] * w;
		const float z = soup[b + 2U] * u + soup[b + 5U] * v + soup[b + 8U] * w;
		outXyz.push_back(x);
		outXyz.push_back(y);
		outXyz.push_back(z);
		outLabels.push_back(triLabels[static_cast<std::size_t>(tri)]);
	}
	return true;
}

bool LabelingSession::exportPointNetDataset(
	const std::string& outputDirUtf8,
	const LabelingDatasetExportOptions& options,
	LabelingDatasetExportResult& outResult,
	std::string* errMsg) const
{
	namespace fs = std::filesystem;
	outResult = {};
	try
	{
		const fs::path root(outputDirUtf8);
		const fs::path dataDir = root / "data";
		fs::create_directories(dataDir);

		std::vector<float> exportXyz;
		std::vector<int> exportLabels;
		const int nc = options.numClasses > 0 ? options.numClasses : static_cast<int>(m_config.classes.size());

		if (m_kind == LabelingGeometryKind::PointCloud)
		{
			exportXyz = m_xyz;
			exportLabels = m_pointLabels;
		}
		else
		{
			if (!sampleMeshLabelsToPointCloud(
					m_triangleSoup, m_triangleLabels, options.meshSampleCount, exportXyz, exportLabels))
			{
				if (errMsg)
				{
					*errMsg = "mesh sample failed";
				}
				return false;
			}
		}

		const std::string baseName = options.sampleNameUtf8.empty() ? "sample_0001" : options.sampleNameUtf8;
		const std::string plyName = baseName + ".ply";
		const std::string labelName = baseName + "_labels.npy";
		const fs::path plyPath = dataDir / plyName;
		const fs::path labelPath = dataDir / labelName;

		if (!writeAsciiPly(plyPath.string(), exportXyz))
		{
			if (errMsg)
			{
				*errMsg = "write ply failed";
			}
			return false;
		}
		if (!writeNpyInt64(labelPath.string(), exportLabels))
		{
			if (errMsg)
			{
				*errMsg = "write npy failed";
			}
			return false;
		}

		const fs::path jsonlPath = root / "dataset.jsonl";
		std::ofstream jsonl(jsonlPath, std::ios::app);
		if (!jsonl)
		{
			if (errMsg)
			{
				*errMsg = "write dataset.jsonl failed";
			}
			return false;
		}
		jsonl << "{\"instruction\":\"分割网格部件\",\"input\":\"" << plyName << "\",\"output\":\"{\\\"label_file\\\": \\\""
			  << labelName << "\\\", \\\"num_classes\\\": " << nc << "}\"}\n";

		outResult.ok = true;
		outResult.plyRelativePath = (std::string("data/") + plyName);
		outResult.labelRelativePath = (std::string("data/") + labelName);
		outResult.datasetJsonlPath = jsonlPath.string();
		return true;
	}
	catch (const std::exception& ex)
	{
		if (errMsg)
		{
			*errMsg = ex.what();
		}
		return false;
	}
}
