#include "RegistrationSdf.h"

#include "Downsample.h"
#include "Preprocess.h"
#include "RegistrationRigid.h"
#include "Transform.h"
#include "sdf/DistanceField.h"
#include "sdf/SdfDeformSolver.h"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <map>
#include <sstream>
#include <utility>

namespace pclalgo
{
namespace
{

bool ensureNormals(std::vector<float>& xyz, std::vector<float>& normals, std::string* errMsg)
{
	if (normals.size() == xyz.size() && !normals.empty())
	{
		return true;
	}
	if (!estimateNormalsPca(xyz, normals, 12U, errMsg))
	{
		return false;
	}
	return orientNormalsMst(xyz, normals, 12U, nullptr, errMsg);
}

/**
 * 由三角面面积加权求顶点法线。
 * orientNormalsMst 会重排/裁剪点数组，绝不能用在焊接顶点数组上（会打乱 cornerToVert 索引）
 */
void computeVertexNormalsFromSoup(const std::vector<float>& soup, const std::vector<int>& cornerToVert,
								  const std::size_t vertCount, std::vector<float>& normalsOut)
{
	normalsOut.assign(vertCount * 3U, 0.0f);
	const std::size_t triCount = cornerToVert.size() / 3U;
	for (std::size_t t = 0; t < triCount; ++t)
	{
		const float* p = soup.data() + t * 9U;
		const Eigen::Vector3d a(p[0], p[1], p[2]);
		const Eigen::Vector3d b(p[3], p[4], p[5]);
		const Eigen::Vector3d c(p[6], p[7], p[8]);
		// 未归一化的叉积自带面积权重
		const Eigen::Vector3d n = (b - a).cross(c - a);
		for (int k = 0; k < 3; ++k)
		{
			const int vi = cornerToVert[t * 3U + static_cast<std::size_t>(k)];
			if (vi < 0 || static_cast<std::size_t>(vi) >= vertCount)
			{
				continue;
			}
			const std::size_t b3 = static_cast<std::size_t>(vi) * 3U;
			normalsOut[b3] += static_cast<float>(n.x());
			normalsOut[b3 + 1U] += static_cast<float>(n.y());
			normalsOut[b3 + 2U] += static_cast<float>(n.z());
		}
	}
	for (std::size_t i = 0; i < vertCount; ++i)
	{
		const std::size_t b = i * 3U;
		Eigen::Vector3d n(normalsOut[b], normalsOut[b + 1U], normalsOut[b + 2U]);
		const double len = n.norm();
		if (len > 1e-12)
		{
			n /= len;
		}
		else
		{
			n = Eigen::Vector3d(0.0, 0.0, 1.0);
		}
		normalsOut[b] = static_cast<float>(n.x());
		normalsOut[b + 1U] = static_cast<float>(n.y());
		normalsOut[b + 2U] = static_cast<float>(n.z());
	}
}

bool prepareCloud(std::vector<float>& xyz, std::vector<float>& normals, const SdfRegisterParams& params,
				  std::string* errMsg)
{
	if (params.voxelPrefilterMm > 0.0)
	{
		std::vector<float>* nPtr = normals.empty() ? nullptr : &normals;
		if (!downsampleVoxelGrid(xyz, params.voxelPrefilterMm, 1U, nPtr))
		{
			if (errMsg)
			{
				*errMsg = "SDF: voxel prefilter failed";
			}
			return false;
		}
	}
	return ensureNormals(xyz, normals, errMsg);
}

bool maybeRigidPreAlign(std::vector<float>& srcXyz, std::vector<float>& srcNormals, const std::vector<float>& tgtXyz,
						const std::vector<float>& tgtNormals, const SdfRegisterParams& params, std::string* errMsg)
{
	if (!params.rigidPreAlign)
	{
		return true;
	}
	Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
	double rmse = 0.0;
	if (!rigidRegisterPointToPlaneIcp(srcXyz, srcNormals, tgtXyz, tgtNormals, T, &rmse,
									  params.rigidPreAlignMaxIterations, 0.01, params.rigidPreAlignMaxPairDistanceMm,
									  params.rigidPreAlignMaxPoints, errMsg))
	{
		return false;
	}
	transformXyzInPlace(srcXyz, T);
	// 旋转法线
	const Eigen::Matrix3d R = T.linear();
	for (std::size_t i = 0; i + 2U < srcNormals.size(); i += 3U)
	{
		Eigen::Vector3d n(srcNormals[i], srcNormals[i + 1U], srcNormals[i + 2U]);
		n = R * n;
		const double len = n.norm();
		if (len > 1e-12)
		{
			n /= len;
		}
		srcNormals[i] = static_cast<float>(n.x());
		srcNormals[i + 1U] = static_cast<float>(n.y());
		srcNormals[i + 2U] = static_cast<float>(n.z());
	}
	return true;
}

double bboxDiag(const std::vector<float>& xyz)
{
	if (xyz.size() < 3U)
	{
		return 1.0;
	}
	Eigen::AlignedBox3d box;
	for (std::size_t i = 0; i + 2U < xyz.size(); i += 3U)
	{
		box.extend(Eigen::Vector3d(xyz[i], xyz[i + 1U], xyz[i + 2U]));
	}
	return std::max(1e-6, box.diagonal().norm());
}

void scaleCloud(std::vector<float>& xyz, double s)
{
	for (float& v : xyz)
	{
		v = static_cast<float>(static_cast<double>(v) * s);
	}
}

Eigen::Vector3d cloudCentroid(const std::vector<float>& xyz)
{
	Eigen::Vector3d c = Eigen::Vector3d::Zero();
	const std::size_t n = xyz.size() / 3U;
	if (n == 0U)
	{
		return c;
	}
	for (std::size_t i = 0; i < n; ++i)
	{
		c += Eigen::Vector3d(xyz[i * 3U], xyz[i * 3U + 1U], xyz[i * 3U + 2U]);
	}
	return c / static_cast<double>(n);
}

void translateCloud(std::vector<float>& xyz, const Eigen::Vector3d& t)
{
	for (std::size_t i = 0; i + 2U < xyz.size(); i += 3U)
	{
		xyz[i] = static_cast<float>(static_cast<double>(xyz[i]) + t.x());
		xyz[i + 1U] = static_cast<float>(static_cast<double>(xyz[i + 1U]) + t.y());
		xyz[i + 2U] = static_cast<float>(static_cast<double>(xyz[i + 2U]) + t.z());
	}
}

bool soupToUniqueVertices(const std::vector<float>& soup, std::vector<float>& xyz, std::vector<int>& cornerToVert,
						  std::string* errMsg)
{
	if (soup.size() % 9U != 0U || soup.empty())
	{
		if (errMsg)
		{
			*errMsg = "SDF: invalid soup";
		}
		return false;
	}
	xyz.clear();
	cornerToVert.assign(soup.size() / 3U, -1);
	// 简单量化焊点
	struct Key
	{
		int x, y, z;
		bool operator<(const Key& o) const
		{
			if (x != o.x)
				return x < o.x;
			if (y != o.y)
				return y < o.y;
			return z < o.z;
		}
	};
	std::map<Key, int> map;
	// 与 SPARE 对齐：绝对 1e-4 下限，避免过细量化导致共点不焊、三角撕裂
	double diag = 1.0;
	{
		Eigen::AlignedBox3d box;
		for (std::size_t c = 0; c < soup.size() / 3U; ++c)
		{
			box.extend(Eigen::Vector3d(soup[c * 3U], soup[c * 3U + 1U], soup[c * 3U + 2U]));
		}
		diag = std::max(1e-3, box.diagonal().norm());
	}
	const double q = std::max(1e-4, diag * 1e-6);
	int next = 0;
	for (std::size_t c = 0; c < soup.size() / 3U; ++c)
	{
		const double px = soup[c * 3U];
		const double py = soup[c * 3U + 1U];
		const double pz = soup[c * 3U + 2U];
		Key k{static_cast<int>(std::llround(px / q)), static_cast<int>(std::llround(py / q)),
			  static_cast<int>(std::llround(pz / q))};
		auto it = map.find(k);
		if (it == map.end())
		{
			map.emplace(k, next);
			cornerToVert[c] = next;
			xyz.push_back(static_cast<float>(px));
			xyz.push_back(static_cast<float>(py));
			xyz.push_back(static_cast<float>(pz));
			++next;
		}
		else
		{
			cornerToVert[c] = it->second;
		}
	}
	return true;
}

void writeSoupFromVerts(const std::vector<float>& xyz, const std::vector<int>& cornerToVert,
						std::vector<float>& soupOut)
{
	soupOut.resize(cornerToVert.size() * 3U);
	for (std::size_t c = 0; c < cornerToVert.size(); ++c)
	{
		const int vi = cornerToVert[c];
		soupOut[c * 3U] = xyz[static_cast<std::size_t>(vi) * 3U];
		soupOut[c * 3U + 1U] = xyz[static_cast<std::size_t>(vi) * 3U + 1U];
		soupOut[c * 3U + 2U] = xyz[static_cast<std::size_t>(vi) * 3U + 2U];
	}
}

void buildMeshEdges(const std::vector<int>& cornerToVert, std::vector<std::pair<int, int>>& edgesOut)
{
	edgesOut.clear();
	const std::size_t triCount = cornerToVert.size() / 3U;
	edgesOut.reserve(triCount * 3U);
	for (std::size_t t = 0; t < triCount; ++t)
	{
		const int a = cornerToVert[t * 3U];
		const int b = cornerToVert[t * 3U + 1U];
		const int c = cornerToVert[t * 3U + 2U];
		if (a < 0 || b < 0 || c < 0)
		{
			continue;
		}
		auto add = [&](int u, int v) {
			if (u == v)
			{
				return;
			}
			if (u > v)
			{
				std::swap(u, v);
			}
			edgesOut.emplace_back(u, v);
		};
		add(a, b);
		add(b, c);
		add(c, a);
	}
	std::sort(edgesOut.begin(), edgesOut.end());
	edgesOut.erase(std::unique(edgesOut.begin(), edgesOut.end()), edgesOut.end());
}

bool runCore(std::vector<float>& srcXyz, std::vector<float>& srcNormals, std::vector<float> tgtXyz,
			 std::vector<float> tgtNormals, const SdfRegisterParams& params, SdfRegisterResult* stats,
			 std::string* errMsg, const std::vector<std::pair<int, int>>* meshEdges = nullptr)
{
	std::string stageTrace;
	auto traceBbox = [&stageTrace](const char* tag, const std::vector<float>& v) {
		if (v.size() < 3U)
		{
			return;
		}
		Eigen::Vector3d mn(v[0], v[1], v[2]);
		Eigen::Vector3d mx = mn;
		for (std::size_t i = 0; i + 2U < v.size(); i += 3U)
		{
			const Eigen::Vector3d p(v[i], v[i + 1U], v[i + 2U]);
			mn = mn.cwiseMin(p);
			mx = mx.cwiseMax(p);
		}
		std::ostringstream oss;
		oss << "[SDF-trace] " << tag << " bbox min=(" << mn.x() << "," << mn.y() << "," << mn.z() << ") max=(" << mx.x()
			<< "," << mx.y() << "," << mx.z() << ")\n";
		stageTrace += oss.str();
	};
	traceBbox("input", srcXyz);

	if (!prepareCloud(srcXyz, srcNormals, params, errMsg))
	{
		return false;
	}
	if (!prepareCloud(tgtXyz, tgtNormals, params, errMsg))
	{
		return false;
	}
	traceBbox("afterPrepare", srcXyz);

	// 先把源质心对齐到目标，避免未预对齐时 NN 对应到轴向远端
	const Eigen::Vector3d srcC0 = cloudCentroid(srcXyz);
	const Eigen::Vector3d tgtC0 = cloudCentroid(tgtXyz);
	const Eigen::Vector3d centroidShift = tgtC0 - srcC0;
	translateCloud(srcXyz, centroidShift);
	traceBbox("afterCentroidShift", srcXyz);

	if (!maybeRigidPreAlign(srcXyz, srcNormals, tgtXyz, tgtNormals, params, errMsg))
	{
		return false;
	}
	traceBbox("afterRigid", srcXyz);

	double scale = 1.0;
	Eigen::Vector3d pivot = Eigen::Vector3d::Zero();
	if (params.normalizeScale)
	{
		pivot = cloudCentroid(tgtXyz);
		translateCloud(srcXyz, -pivot);
		translateCloud(tgtXyz, -pivot);
		scale = 1.0 / bboxDiag(tgtXyz);
		scaleCloud(srcXyz, scale);
		scaleCloud(tgtXyz, scale);
	}
	traceBbox("afterNormalize", srcXyz);

	sdf::DistanceField field;
	SdfRegisterParams p = params;
	if (params.normalizeScale && params.fieldVoxelMm > 0.0)
	{
		p.fieldVoxelMm = params.fieldVoxelMm * scale;
	}
	if (!field.buildFromPointCloud(tgtXyz, tgtNormals, p.fieldVoxelMm, errMsg))
	{
		return false;
	}
	if (!sdf::runSdfDeform(srcXyz, srcNormals, field, p, stats, errMsg, meshEdges))
	{
		return false;
	}
	traceBbox("afterDeform(norm)", srcXyz);
	if (params.normalizeScale)
	{
		scaleCloud(srcXyz, 1.0 / scale);
		translateCloud(srcXyz, pivot);
	}
	traceBbox("final", srcXyz);
	if (stats)
	{
		stats->debugSummary = stageTrace + stats->debugSummary;
		stats->meshScale = scale;
		if (params.normalizeScale)
		{
			stats->meanErrorMm /= scale;
			stats->fieldVoxelMmUsed /= scale;
			stats->nodeDispMeanMm /= scale;
			stats->nodeDispMaxMm /= scale;
			stats->vertDispMeanMm /= scale;
			stats->vertDispMaxMm /= scale;
		}
		{
			std::ostringstream nod;
			nod << "[SDF-debug] nodeDisp mean=" << stats->nodeDispMeanMm << " max=" << stats->nodeDispMaxMm << " mm\n";
			nod << "[SDF-debug] vertFineDisp mean=" << stats->vertDispMeanMm << " max=" << stats->vertDispMaxMm
				<< " mm\n";
			stats->debugSummary += nod.str();
		}
		{
			const double k = params.normalizeScale ? (1.0 / scale) : 1.0;
			std::ostringstream ed;
			ed << "[SDF-debug] errInit normal=" << stats->errInitNormal * k
			   << " tangential=" << stats->errInitTangential * k << " mm | errFinal mean=" << stats->errFinalMean * k
			   << " normal=" << stats->errFinalNormal * k << " tangential=" << stats->errFinalTangential * k << " mm\n";
			if (stats->errFinalTangential > stats->errFinalNormal)
			{
				ed << "[SDF-WARN] 残差以切向为主 → 当前 SDF/点-面数据项天然看不见轴向滑移，"
					  "请改 DDF 模式或提高切向权重\n";
			}
			else if (stats->errInitNormal > 1e-9 && stats->errFinalNormal > stats->errInitNormal * 0.8)
			{
				ed << "[SDF-WARN] 法向残差几乎没降 → 节点分辨率不足/优化未收敛，"
					  "建议减小采样半径(sampleRadiusRatio)或增加迭代\n";
			}
			stats->debugSummary += ed.str();
		}
	}
	return true;
}

} // namespace

bool sdfRegisterPointClouds(const std::vector<float>& sourceXyz, const std::vector<float>& sourceNormals,
							const std::vector<float>& targetXyz, const std::vector<float>& targetNormals,
							std::vector<float>& sourceXyzDeformedOut, std::vector<float>& sourceNormalsDeformedOut,
							const SdfRegisterParams& params, SdfRegisterResult* stats, std::string* errMsg)
{
	sourceXyzDeformedOut = sourceXyz;
	sourceNormalsDeformedOut = sourceNormals;
	return runCore(sourceXyzDeformedOut, sourceNormalsDeformedOut, targetXyz, targetNormals, params, stats, errMsg,
				   nullptr);
}

bool sdfRegisterMeshSoupToTarget(const std::vector<float>& sourceSoup, const std::vector<float>& targetXyz,
								 const std::vector<float>& targetNormals, std::vector<float>& sourceSoupDeformedOut,
								 const SdfRegisterParams& params, SdfRegisterResult* stats, std::string* errMsg)
{
	std::vector<float> xyz;
	std::vector<int> cornerToVert;
	if (!soupToUniqueVertices(sourceSoup, xyz, cornerToVert, errMsg))
	{
		return false;
	}
	// 焊点正确性直证：首个「焊接边 ≫ 原始边」的三角，打印角点原始坐标 vs 焊接顶点坐标
	std::string weldProof;
	{
		const std::size_t tc = sourceSoup.size() / 9U;
		for (std::size_t t = 0; t < tc; ++t)
		{
			const float* p = sourceSoup.data() + t * 9U;
			const double rawE01 = std::sqrt(std::pow(p[3] - p[0], 2) + std::pow(p[4] - p[1], 2) + std::pow(p[5] - p[2], 2));
			const double rawE12 = std::sqrt(std::pow(p[6] - p[3], 2) + std::pow(p[7] - p[4], 2) + std::pow(p[8] - p[5], 2));
			const double rawE20 = std::sqrt(std::pow(p[0] - p[6], 2) + std::pow(p[1] - p[7], 2) + std::pow(p[2] - p[8], 2));
			const double rawMax = std::max({rawE01, rawE12, rawE20});
			const int ia = cornerToVert[t * 3U];
			const int ib = cornerToVert[t * 3U + 1U];
			const int ic = cornerToVert[t * 3U + 2U];
			if (ia < 0 || ib < 0 || ic < 0)
			{
				continue;
			}
			const Eigen::Vector3d a(xyz[static_cast<std::size_t>(ia) * 3U], xyz[static_cast<std::size_t>(ia) * 3U + 1U],
									xyz[static_cast<std::size_t>(ia) * 3U + 2U]);
			const Eigen::Vector3d b(xyz[static_cast<std::size_t>(ib) * 3U], xyz[static_cast<std::size_t>(ib) * 3U + 1U],
									xyz[static_cast<std::size_t>(ib) * 3U + 2U]);
			const Eigen::Vector3d c(xyz[static_cast<std::size_t>(ic) * 3U], xyz[static_cast<std::size_t>(ic) * 3U + 1U],
									xyz[static_cast<std::size_t>(ic) * 3U + 2U]);
			const double weldMax = std::max({(b - a).norm(), (c - b).norm(), (a - c).norm()});
			if (weldMax > std::max(1.0, rawMax * 10.0))
			{
				std::ostringstream oss;
				oss << "[SDF-WARN] 焊点错位 tri#" << t << " rawMax=" << rawMax << " weldMax=" << weldMax
					<< " verts=" << ia << "," << ib << "," << ic << " rawCorners: (" << p[0] << "," << p[1] << ","
					<< p[2] << ") (" << p[3] << "," << p[4] << "," << p[5] << ") (" << p[6] << "," << p[7] << ","
					<< p[8] << ")\n";
				weldProof = oss.str();
				break;
			}
		}
	}
	std::vector<float> normals;
	// 网格路径：法线由面片面积加权得到，保证 xyz 索引不变（orientNormalsMst 会重排点）
	computeVertexNormalsFromSoup(sourceSoup, cornerToVert, xyz.size() / 3U, normals);
	std::vector<std::pair<int, int>> meshEdges;
	buildMeshEdges(cornerToVert, meshEdges);

	// 长边审计直接基于原始 soup（与 Host auditSoupLongTris 完全一致），不经过焊点映射
	int longRest8 = 0;
	int longRest25 = 0;
	int longRest50 = 0;
	double maxEdgeRatio = 0.0;
	double maxEdgeMm = 0.0;
	{
		double minX = sourceSoup[0], minY = sourceSoup[1], minZ = sourceSoup[2];
		double maxX = minX, maxY = minY, maxZ = minZ;
		for (std::size_t i = 0; i + 2U < sourceSoup.size(); i += 3U)
		{
			minX = std::min(minX, static_cast<double>(sourceSoup[i]));
			minY = std::min(minY, static_cast<double>(sourceSoup[i + 1U]));
			minZ = std::min(minZ, static_cast<double>(sourceSoup[i + 2U]));
			maxX = std::max(maxX, static_cast<double>(sourceSoup[i]));
			maxY = std::max(maxY, static_cast<double>(sourceSoup[i + 1U]));
			maxZ = std::max(maxZ, static_cast<double>(sourceSoup[i + 2U]));
		}
		const double dx = maxX - minX, dy = maxY - minY, dz = maxZ - minZ;
		const double soupDiag = std::max(1e-6, std::sqrt(dx * dx + dy * dy + dz * dz));
		const std::size_t triCount = sourceSoup.size() / 9U;
		for (std::size_t t = 0; t < triCount; ++t)
		{
			const float* p = sourceSoup.data() + t * 9U;
			const double e01 = std::sqrt(std::pow(p[3] - p[0], 2) + std::pow(p[4] - p[1], 2) + std::pow(p[5] - p[2], 2));
			const double e12 = std::sqrt(std::pow(p[6] - p[3], 2) + std::pow(p[7] - p[4], 2) + std::pow(p[8] - p[5], 2));
			const double e20 = std::sqrt(std::pow(p[0] - p[6], 2) + std::pow(p[1] - p[7], 2) + std::pow(p[2] - p[8], 2));
			const double maxE = std::max({e01, e12, e20});
			const double ratio = maxE / soupDiag;
			maxEdgeRatio = std::max(maxEdgeRatio, ratio);
			maxEdgeMm = std::max(maxEdgeMm, maxE);
			if (ratio > 0.08)
			{
				++longRest8;
			}
			if (ratio > 0.25)
			{
				++longRest25;
			}
			if (ratio > 0.50)
			{
				++longRest50;
			}
		}
	}
	const std::size_t triCount = cornerToVert.size() / 3U;

	SdfRegisterParams meshParams = params;
	meshParams.voxelPrefilterMm = 0.0;
	if (!runCore(xyz, normals, targetXyz, targetNormals, meshParams, stats, errMsg, &meshEdges))
	{
		return false;
	}
	if (stats)
	{
		stats->soupCornerCount = static_cast<int>(cornerToVert.size());
		stats->uniqueVertexCount = static_cast<int>(xyz.size() / 3U);
		stats->sourceTriangleCount = static_cast<int>(triCount);
		std::ostringstream weld;
		weld << "[SDF-debug] weld soupCorners=" << stats->soupCornerCount
			 << " uniqueVerts=" << stats->uniqueVertexCount << " tris=" << stats->sourceTriangleCount
			 << " weldRatio="
			 << (stats->soupCornerCount > 0
					 ? (100.0 * stats->uniqueVertexCount / static_cast<double>(stats->soupCornerCount))
					 : 0.0)
			 << "%\n";
		weld << "[SDF-debug] sourceLongTris >8%diag=" << longRest8 << " >25%=" << longRest25 << " >50%=" << longRest50
			 << " /" << triCount << " maxEdge/diag=" << maxEdgeRatio << " maxEdgeMm=" << maxEdgeMm << "\n";
		if (stats->soupCornerCount > 0 && stats->uniqueVertexCount * 10 > stats->soupCornerCount * 9)
		{
			weld << "[SDF-WARN] 焊点率极低(角点几乎未合并) → soup 未共享顶点，易出现裂缝式长三角\n";
		}
		if (longRest25 > 0)
		{
			weld << "[SDF-WARN] 源在变形前已有跨尺度长边(>25%diag=" << longRest25
				 << ") → 蜘蛛网来自源网格拓扑/离散，非本次非刚性撕边；请对源开边显示核对\n";
		}
		else if (longRest8 * 5 > static_cast<int>(triCount) && triCount > 0)
		{
			weld << "[SDF-INFO] 源有较多>8%diag 三角（可能是细长件粗网格），若无>25% 则未必是蜘蛛网\n";
		}
		stats->debugSummary = weld.str() + weldProof + stats->debugSummary;
	}
	writeSoupFromVerts(xyz, cornerToVert, sourceSoupDeformedOut);
	// 输出撕裂直证：首个「输出边 ≫ 原始边」三角，打印原始角点 / 焊接索引 / 最终顶点坐标
	if (stats)
	{
		const std::size_t tc = sourceSoup.size() / 9U;
		for (std::size_t t = 0; t < tc; ++t)
		{
			const float* p = sourceSoup.data() + t * 9U;
			const double rawE01 = std::sqrt(std::pow(p[3] - p[0], 2) + std::pow(p[4] - p[1], 2) + std::pow(p[5] - p[2], 2));
			const double rawE12 = std::sqrt(std::pow(p[6] - p[3], 2) + std::pow(p[7] - p[4], 2) + std::pow(p[8] - p[5], 2));
			const double rawE20 = std::sqrt(std::pow(p[0] - p[6], 2) + std::pow(p[1] - p[7], 2) + std::pow(p[2] - p[8], 2));
			const double rawMax = std::max({rawE01, rawE12, rawE20});
			const float* q = sourceSoupDeformedOut.data() + t * 9U;
			const double outE01 =
				std::sqrt(std::pow(q[3] - q[0], 2) + std::pow(q[4] - q[1], 2) + std::pow(q[5] - q[2], 2));
			const double outE12 =
				std::sqrt(std::pow(q[6] - q[3], 2) + std::pow(q[7] - q[4], 2) + std::pow(q[8] - q[5], 2));
			const double outE20 =
				std::sqrt(std::pow(q[0] - q[6], 2) + std::pow(q[1] - q[7], 2) + std::pow(q[2] - q[8], 2));
			const double outMax = std::max({outE01, outE12, outE20});
			if (outMax > std::max(5.0, rawMax * 10.0))
			{
				const int ia = cornerToVert[t * 3U];
				const int ib = cornerToVert[t * 3U + 1U];
				const int ic = cornerToVert[t * 3U + 2U];
				std::ostringstream oss;
				oss << "[SDF-WARN] 输出撕裂 tri#" << t << " rawMax=" << rawMax << " outMax=" << outMax
					<< " verts=" << ia << "," << ib << "," << ic << "\n[SDF-WARN] rawCorners: (" << p[0] << "," << p[1]
					<< "," << p[2] << ") (" << p[3] << "," << p[4] << "," << p[5] << ") (" << p[6] << "," << p[7]
					<< "," << p[8] << ")\n[SDF-WARN] outCorners: (" << q[0] << "," << q[1] << "," << q[2] << ") ("
					<< q[3] << "," << q[4] << "," << q[5] << ") (" << q[6] << "," << q[7] << "," << q[8] << ")\n";
				stats->debugSummary += oss.str();
				break;
			}
		}
	}
	return true;
}

bool sdfRegisterMeshSoupToMeshSoup(const std::vector<float>& sourceSoup, const std::vector<float>& targetSoup,
								   std::vector<float>& sourceSoupDeformedOut, const SdfRegisterParams& params,
								   SdfRegisterResult* stats, std::string* errMsg)
{
	std::vector<float> tgtXyz;
	std::vector<float> tgtNormals;
	std::vector<int> corners;
	if (!soupToUniqueVertices(targetSoup, tgtXyz, corners, errMsg))
	{
		return false;
	}
	// 目标侧同理：面片法线，避免点集被重排导致场对应错乱
	computeVertexNormalsFromSoup(targetSoup, corners, tgtXyz.size() / 3U, tgtNormals);
	return sdfRegisterMeshSoupToTarget(sourceSoup, tgtXyz, tgtNormals, sourceSoupDeformedOut, params, stats, errMsg);
}

} // namespace pclalgo
