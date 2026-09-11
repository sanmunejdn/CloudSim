/// @file RegistrationGlobalPcl.cpp
/// @brief PCL FPFH + SampleConsensusPrerejective 全局粗配

#include "RegistrationGlobalPcl.h"

#include "Measure.h"
#include "PointCloudBuffer.h"
#include "RegistrationRigid.h"
#include "Transform.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

#ifdef CLOUDSIM_HAS_PCL
#include "pcl/PclCloudConvert.h"

#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/search/kdtree.h>
#endif

namespace pclalgo
{
namespace
{
double bboxDiagMm(const std::vector<float>& xyz)
{
	const Eigen::AlignedBox3d box = computeBoundingBox(xyz);
	if (box.isEmpty())
	{
		return 0.0;
	}
	return box.diagonal().norm();
}

void resolvePclParams(const std::vector<float>& srcXyz, const std::vector<float>& tgtXyz, PclGlobalAlignParams& p)
{
	const double diag = (std::max)(bboxDiagMm(srcXyz), bboxDiagMm(tgtXyz));
	if (p.featureVoxelMm <= 0.0)
	{
		p.featureVoxelMm = (std::max)(diag * 0.015, 1.0);
	}
	if (p.normalRadiusMm <= 0.0)
	{
		p.normalRadiusMm = p.featureVoxelMm * 2.5;
	}
	if (p.fpfhRadiusMm <= 0.0)
	{
		p.fpfhRadiusMm = p.featureVoxelMm * 5.0;
	}
	// 过宽会吃进错转角；过窄+高 inlierFraction 会永不收敛
	if (p.inlierDistanceMm <= 0.0)
	{
		p.inlierDistanceMm = (std::max)(diag * 0.025, 2.0);
	}
}

#ifdef CLOUDSIM_HAS_PCL
using pcl_bridge::CloudNT;
using pcl_bridge::CloudNTPtr;
using pcl_bridge::PointNT;
using FeatureT = pcl::FPFHSignature33;
using FeatureCloud = pcl::PointCloud<FeatureT>;

void clearNormals(CloudNT& cloud)
{
	for (PointNT& p : cloud.points)
	{
		p.normal_x = 0.f;
		p.normal_y = 0.f;
		p.normal_z = 0.f;
	}
}

bool downsampleVoxel(CloudNTPtr& cloud, const float leafMm)
{
	if (!cloud || cloud->empty() || leafMm <= 0.f)
	{
		return false;
	}
	pcl::VoxelGrid<PointNT> vg;
	vg.setInputCloud(cloud);
	vg.setLeafSize(leafMm, leafMm, leafMm);
	CloudNTPtr out(new CloudNT);
	vg.filter(*out);
	if (out->size() < 20U)
	{
		return false;
	}
	cloud.swap(out);
	return true;
}

bool reestimateNormals(CloudNTPtr& cloud, const float radiusMm, std::string* errMsg)
{
	if (!cloud || cloud->empty())
	{
		if (errMsg)
		{
			*errMsg = "empty cloud for normal estimation";
		}
		return false;
	}
	// 视点取自身质心，避免源/目标独立 MST 翻向导致 FPFH 对不上
	Eigen::Vector4f centroid = Eigen::Vector4f::Zero();
	pcl::compute3DCentroid(*cloud, centroid);

	pcl::NormalEstimationOMP<PointNT, PointNT> ne;
	ne.setInputCloud(cloud);
	pcl::search::KdTree<PointNT>::Ptr tree(new pcl::search::KdTree<PointNT>);
	ne.setSearchMethod(tree);
	ne.setRadiusSearch(radiusMm);
	ne.setViewPoint(centroid[0], centroid[1], centroid[2]);
	CloudNTPtr out(new CloudNT);
	ne.compute(*out);
	if (out->size() != cloud->size())
	{
		if (errMsg)
		{
			*errMsg = "PCL normal estimation size mismatch";
		}
		return false;
	}
	for (std::size_t i = 0; i < cloud->size(); ++i)
	{
		cloud->points[i].normal_x = out->points[i].normal_x;
		cloud->points[i].normal_y = out->points[i].normal_y;
		cloud->points[i].normal_z = out->points[i].normal_z;
	}
	return true;
}

bool computeFpfh(const CloudNTPtr& cloud, const float radiusMm, FeatureCloud::Ptr& outFeat, std::string* errMsg)
{
	pcl::FPFHEstimationOMP<PointNT, PointNT, FeatureT> est;
	est.setInputCloud(cloud);
	est.setInputNormals(cloud);
	pcl::search::KdTree<PointNT>::Ptr tree(new pcl::search::KdTree<PointNT>);
	est.setSearchMethod(tree);
	est.setRadiusSearch(radiusMm);
	outFeat.reset(new FeatureCloud);
	est.compute(*outFeat);
	if (!outFeat || outFeat->size() != cloud->size())
	{
		if (errMsg)
		{
			*errMsg = "PCL FPFH computation failed";
		}
		return false;
	}
	return true;
}

void cloudToXyzNormals(const CloudNT& cloud, std::vector<float>& xyz, std::vector<float>& normals)
{
	xyz.resize(cloud.size() * 3U);
	normals.resize(cloud.size() * 3U);
	for (std::size_t i = 0; i < cloud.size(); ++i)
	{
		const PointNT& p = cloud.points[i];
		const std::size_t b = i * 3U;
		xyz[b] = p.x;
		xyz[b + 1U] = p.y;
		xyz[b + 2U] = p.z;
		normals[b] = p.normal_x;
		normals[b + 1U] = p.normal_y;
		normals[b + 2U] = p.normal_z;
	}
}

bool transformLooksRigid(const Eigen::Isometry3d& t)
{
	const double det = t.linear().determinant();
	return det > 0.5 && std::abs(det - 1.0) < 0.15;
}

float evaluateFitnessMm(const CloudNT& src, const CloudNT& tgt, const Eigen::Isometry3d& t, const float maxDistMm,
						std::size_t& outInliers)
{
	outInliers = 0;
	if (src.empty() || tgt.empty())
	{
		return std::numeric_limits<float>::max();
	}
	pcl::search::KdTree<PointNT> tree;
	CloudNTPtr tgtPtr(new CloudNT(tgt));
	tree.setInputCloud(tgtPtr);
	const float maxDistSq = maxDistMm * maxDistMm;
	double sumSq = 0.0;
	for (const PointNT& p : src.points)
	{
		const Eigen::Vector3d pw = t * Eigen::Vector3d(p.x, p.y, p.z);
		PointNT q;
		q.x = static_cast<float>(pw.x());
		q.y = static_cast<float>(pw.y());
		q.z = static_cast<float>(pw.z());
		pcl::Indices nnIdx(1);
		std::vector<float> nnDist(1);
		if (tree.nearestKSearch(q, 1, nnIdx, nnDist) < 1)
		{
			continue;
		}
		if (nnDist[0] <= maxDistSq)
		{
			++outInliers;
			sumSq += static_cast<double>(nnDist[0]);
		}
	}
	if (outInliers == 0)
	{
		return std::numeric_limits<float>::max();
	}
	return static_cast<float>(std::sqrt(sumSq / static_cast<double>(outInliers)));
}

/// 全体源点 NN 均值（含离群）；比“仅内点 fitness”更能揭开错转角
float evaluateMeanNnMm(const CloudNT& src, const CloudNT& tgt, const Eigen::Isometry3d& t)
{
	if (src.empty() || tgt.empty())
	{
		return std::numeric_limits<float>::max();
	}
	pcl::search::KdTree<PointNT> tree;
	CloudNTPtr tgtPtr(new CloudNT(tgt));
	tree.setInputCloud(tgtPtr);
	double sum = 0.0;
	std::size_t counted = 0;
	for (const PointNT& p : src.points)
	{
		const Eigen::Vector3d pw = t * Eigen::Vector3d(p.x, p.y, p.z);
		PointNT q;
		q.x = static_cast<float>(pw.x());
		q.y = static_cast<float>(pw.y());
		q.z = static_cast<float>(pw.z());
		pcl::Indices nnIdx(1);
		std::vector<float> nnDist(1);
		if (tree.nearestKSearch(q, 1, nnIdx, nnDist) < 1)
		{
			continue;
		}
		sum += std::sqrt(static_cast<double>(nnDist[0]));
		++counted;
	}
	if (counted == 0)
	{
		return std::numeric_limits<float>::max();
	}
	return static_cast<float>(sum / static_cast<double>(counted));
}

float inlierRatioUnder(const CloudNT& src, const CloudNT& tgt, const Eigen::Isometry3d& t, const float maxDistMm)
{
	std::size_t inliers = 0;
	(void)evaluateFitnessMm(src, tgt, t, maxDistMm, inliers);
	return src.empty() ? 0.f : static_cast<float>(inliers) / static_cast<float>(src.size());
}

bool refineWithIcp(const CloudNT& src, const CloudNT& tgt, Eigen::Isometry3d& ioTransform,
				   const PclGlobalAlignParams& params)
{
	if (!params.refineWithIcp || src.empty() || tgt.empty())
	{
		return true;
	}
	std::vector<float> srcXyz;
	std::vector<float> srcN;
	std::vector<float> tgtXyz;
	std::vector<float> tgtN;
	cloudToXyzNormals(src, srcXyz, srcN);
	cloudToXyzNormals(tgt, tgtXyz, tgtN);
	transformXyzInPlace(srcXyz, ioTransform);
	const Eigen::Matrix3d rot = ioTransform.linear();
	for (std::size_t i = 0; i < pointCountFromXyz(srcN); ++i)
	{
		const std::size_t b = i * 3U;
		Eigen::Vector3d n(srcN[b], srcN[b + 1U], srcN[b + 2U]);
		n = rot * n;
		const double len = n.norm();
		if (len > 1e-12)
		{
			n /= len;
		}
		srcN[b] = static_cast<float>(n.x());
		srcN[b + 1U] = static_cast<float>(n.y());
		srcN[b + 2U] = static_cast<float>(n.z());
	}
	Eigen::Isometry3d icpStep = Eigen::Isometry3d::Identity();
	double rmse = 0.0;
	if (!rigidRegisterPointToPlaneIcp(srcXyz, srcN, tgtXyz, tgtN, icpStep, &rmse, 25, 0.005,
									  params.inlierDistanceMm, params.maxFeaturePoints, nullptr,
									  params.icpMaxNormalAngleDeg))
	{
		return false;
	}
	ioTransform = icpStep * ioTransform;
	return true;
}

struct SacCandidate
{
	Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
	float fitnessMm = std::numeric_limits<float>::max();
	float meanNnMm = std::numeric_limits<float>::max();
	float inlierRatio = 0.f;
	float reverseInlierRatio = 0.f;
	bool ok = false;
};

bool runSacOnce(const CloudNTPtr& src, const FeatureCloud::Ptr& srcFeat, const CloudNTPtr& tgt,
				const FeatureCloud::Ptr& tgtFeat, const PclGlobalAlignParams& params, SacCandidate& out,
				std::string* errMsg)
{
	out = SacCandidate{};
	pcl::SampleConsensusPrerejective<PointNT, PointNT, FeatureT> align;
	align.setInputSource(src);
	align.setSourceFeatures(srcFeat);
	align.setInputTarget(tgt);
	align.setTargetFeatures(tgtFeat);
	align.setMaximumIterations(params.maxIterations);
	align.setNumberOfSamples(params.numberOfSamples);
	align.setCorrespondenceRandomness(params.correspondenceRandomness);
	align.setSimilarityThreshold(static_cast<float>(params.similarityThreshold));
	align.setMaxCorrespondenceDistance(static_cast<float>(params.inlierDistanceMm));
	align.setInlierFraction(params.inlierFraction);

	CloudNT aligned;
	align.align(aligned);
	if (!align.hasConverged())
	{
		if (errMsg)
		{
			*errMsg = "PCL SampleConsensusPrerejective did not converge";
		}
		return false;
	}

	const Eigen::Matrix4f tf = align.getFinalTransformation();
	out.transform.matrix() = tf.cast<double>();
	if (!transformLooksRigid(out.transform))
	{
		if (errMsg)
		{
			*errMsg = "PCL SAC transform not a proper rotation";
		}
		return false;
	}

	(void)refineWithIcp(*src, *tgt, out.transform, params);

	const float dist = static_cast<float>(params.inlierDistanceMm);
	std::size_t inliers = 0;
	out.fitnessMm = evaluateFitnessMm(*src, *tgt, out.transform, dist, inliers);
	out.inlierRatio = src->empty() ? 0.f : static_cast<float>(inliers) / static_cast<float>(src->size());
	out.meanNnMm = evaluateMeanNnMm(*src, *tgt, out.transform);
	out.reverseInlierRatio = inlierRatioUnder(*tgt, *src, out.transform.inverse(), dist);

	const float maxMeanNn = dist * static_cast<float>((std::max)(params.maxAcceptMeanNnFactor, 0.5));
	const float maxFitness = dist * 0.55f;
	if (out.inlierRatio < params.minAcceptInlierRatio ||
		out.reverseInlierRatio < params.minAcceptReverseInlierRatio || out.fitnessMm > maxFitness ||
		out.meanNnMm > maxMeanNn)
	{
		if (errMsg)
		{
			std::ostringstream oss;
			oss << "PCL SAC rejected: inlier=" << out.inlierRatio << " revInlier=" << out.reverseInlierRatio
				<< " fitnessMm=" << out.fitnessMm << " meanNnMm=" << out.meanNnMm;
			*errMsg = oss.str();
		}
		return false;
	}
	out.ok = true;
	return true;
}
#endif

} // namespace

PclGlobalAlignParams pclParamsFromRigidRansac(const RigidRegisterRansacParams& src)
{
	PclGlobalAlignParams p;
	p.featureVoxelMm = src.featureVoxelMm;
	p.inlierDistanceMm = src.inlierDistanceMm;
	p.maxIterations = (std::max)(src.maxIterations, 1000);
	p.refineWithIcp = src.refineWithIcp;
	p.maxFeaturePoints = src.maxFeaturePoints;
	p.icpMaxNormalAngleDeg = src.maxNormalAngleDeg > 0.0 ? src.maxNormalAngleDeg : 45.0;
	return p;
}

bool rigidRegisterFeatureRansacPcl(const std::vector<float>& sourceXyz, const std::vector<float>& sourceNormalsNxNyNz,
								   const std::vector<float>& targetXyz, const std::vector<float>& targetNormalsNxNyNz,
								   Eigen::Isometry3d& sourceToTarget, double* inlierRatio, PclGlobalAlignParams params,
								   std::string* errMsg)
{
	sourceToTarget = Eigen::Isometry3d::Identity();
	if (inlierRatio)
	{
		*inlierRatio = 0.0;
	}

#ifndef CLOUDSIM_HAS_PCL
	(void)sourceXyz;
	(void)sourceNormalsNxNyNz;
	(void)targetXyz;
	(void)targetNormalsNxNyNz;
	(void)params;
	if (errMsg)
	{
		*errMsg = "CLOUDSIM_HAS_PCL not enabled";
	}
	return false;
#else
	(void)sourceNormalsNxNyNz;
	(void)targetNormalsNxNyNz;
	if (!validXyzLength(sourceXyz) || !validXyzLength(targetXyz) || pointCountFromXyz(sourceXyz) < 20U ||
		pointCountFromXyz(targetXyz) < 20U)
	{
		if (errMsg)
		{
			*errMsg = "too few points for PCL feature registration";
		}
		return false;
	}

	resolvePclParams(sourceXyz, targetXyz, params);
	srand(params.randomSeed);

	// 入参法线不可靠（体素会平均法线）；统一用质心视点重估
	CloudNTPtr src = pcl_bridge::makeCloudFromXyzNormals(sourceXyz, {});
	CloudNTPtr tgt = pcl_bridge::makeCloudFromXyzNormals(targetXyz, {});
	const float leaf = static_cast<float>(params.featureVoxelMm);
	if (!downsampleVoxel(src, leaf) || !downsampleVoxel(tgt, leaf))
	{
		src = pcl_bridge::makeCloudFromXyzNormals(sourceXyz, {});
		tgt = pcl_bridge::makeCloudFromXyzNormals(targetXyz, {});
	}
	clearNormals(*src);
	clearNormals(*tgt);

	auto maybeRandomThin = [&](CloudNTPtr& c) {
		if (c->size() <= params.maxFeaturePoints)
		{
			return;
		}
		CloudNTPtr thin(new CloudNT);
		thin->points.reserve(params.maxFeaturePoints);
		const double step = static_cast<double>(c->size()) / static_cast<double>(params.maxFeaturePoints);
		for (std::size_t k = 0; k < params.maxFeaturePoints; ++k)
		{
			const std::size_t i = (std::min)(c->size() - 1U, static_cast<std::size_t>(k * step));
			thin->points.push_back(c->points[i]);
		}
		thin->width = static_cast<std::uint32_t>(thin->points.size());
		thin->height = 1;
		thin->is_dense = false;
		c.swap(thin);
	};
	maybeRandomThin(src);
	maybeRandomThin(tgt);

	if (!reestimateNormals(src, static_cast<float>(params.normalRadiusMm), errMsg) ||
		!reestimateNormals(tgt, static_cast<float>(params.normalRadiusMm), errMsg))
	{
		return false;
	}

	FeatureCloud::Ptr srcFeat;
	FeatureCloud::Ptr tgtFeat;
	if (!computeFpfh(src, static_cast<float>(params.fpfhRadiusMm), srcFeat, errMsg) ||
		!computeFpfh(tgt, static_cast<float>(params.fpfhRadiusMm), tgtFeat, errMsg))
	{
		return false;
	}

	SacCandidate best;
	std::string forwardErr;
	if (!runSacOnce(src, srcFeat, tgt, tgtFeat, params, best, &forwardErr))
	{
		if (!params.tryReverse)
		{
			if (errMsg)
			{
				*errMsg = forwardErr;
			}
			return false;
		}
	}

	if (params.tryReverse)
	{
		SacCandidate reverseCand;
		std::string reverseErr;
		if (runSacOnce(tgt, tgtFeat, src, srcFeat, params, reverseCand, &reverseErr))
		{
			reverseCand.transform = reverseCand.transform.inverse();
			const float dist = static_cast<float>(params.inlierDistanceMm);
			std::size_t inliers = 0;
			reverseCand.fitnessMm = evaluateFitnessMm(*src, *tgt, reverseCand.transform, dist, inliers);
			reverseCand.inlierRatio =
				src->empty() ? 0.f : static_cast<float>(inliers) / static_cast<float>(src->size());
			reverseCand.meanNnMm = evaluateMeanNnMm(*src, *tgt, reverseCand.transform);
			reverseCand.reverseInlierRatio = inlierRatioUnder(*tgt, *src, reverseCand.transform.inverse(), dist);
			const float maxMeanNn = dist * static_cast<float>((std::max)(params.maxAcceptMeanNnFactor, 0.5));
			const float maxFitness = dist * 0.55f;
			const bool reverseOk = reverseCand.inlierRatio >= params.minAcceptInlierRatio &&
								   reverseCand.reverseInlierRatio >= params.minAcceptReverseInlierRatio &&
								   reverseCand.fitnessMm <= maxFitness && reverseCand.meanNnMm <= maxMeanNn;
			if (reverseOk &&
				(!best.ok || reverseCand.meanNnMm < best.meanNnMm ||
				 (std::abs(reverseCand.meanNnMm - best.meanNnMm) < 1e-3f &&
				  reverseCand.inlierRatio > best.inlierRatio)))
			{
				best = reverseCand;
				best.ok = true;
			}
		}
		else if (!best.ok)
		{
			if (errMsg)
			{
				*errMsg = forwardErr.empty() ? reverseErr : forwardErr;
			}
			return false;
		}
	}

	if (!best.ok)
	{
		if (errMsg)
		{
			*errMsg = forwardErr.empty() ? "PCL SAC produced no acceptable pose" : forwardErr;
		}
		return false;
	}

	sourceToTarget = best.transform;
	if (inlierRatio)
	{
		*inlierRatio = static_cast<double>(best.inlierRatio);
	}
	return true;
#endif
}

} // namespace pclalgo
