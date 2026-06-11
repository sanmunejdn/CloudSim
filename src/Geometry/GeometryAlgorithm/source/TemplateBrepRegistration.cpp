#include "detail/OccIncludes.h"

#include "TemplateBrepRegistration.h"
#include "ShapeHandle.h"

#include <BRepBuilderAPI_Transform.hxx>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

namespace geoalgo
{
namespace
{

using Vec3 = Eigen::Vector3d;

Vec3 pointAt(const std::vector<float>& xyz, const std::size_t i)
{
	const std::size_t b = i * 3U;
	return Vec3(xyz[b], xyz[b + 1U], xyz[b + 2U]);
}

Vec3 normalAt(const std::vector<float>& nrm, const std::size_t i)
{
	const std::size_t b = i * 3U;
	Vec3 n(nrm[b], nrm[b + 1U], nrm[b + 2U]);
	if (n.norm() > 1e-12)
	{
		n.normalize();
	}
	return n;
}

std::vector<std::size_t> subsampleIndices(const std::size_t count, const std::size_t maxPoints)
{
	std::vector<std::size_t> indices(count);
	for (std::size_t i = 0; i < count; ++i)
	{
		indices[i] = i;
	}
	if (count <= maxPoints)
	{
		return indices;
	}
	std::shuffle(indices.begin(), indices.end(), std::mt19937{std::random_device{}()});
	indices.resize(maxPoints);
	return indices;
}

gp_Trsf isometryToGpTrsf(const Eigen::Isometry3d& transform)
{
	gp_Trsf trsf;
	trsf.SetValues(
		transform(0, 0),
		transform(0, 1),
		transform(0, 2),
		transform(0, 3),
		transform(1, 0),
		transform(1, 1),
		transform(1, 2),
		transform(1, 3),
		transform(2, 0),
		transform(2, 1),
		transform(2, 2),
		transform(2, 3));
	return trsf;
}

bool nativeShapeFromHandle(const ShapeHandle& handle, TopoDS_Shape& outShape)
{
	return ShapeHandleAccess::nativeShape(handle, &outShape);
}

TopoDS_Shape transformNativeShape(const TopoDS_Shape& shape, const Eigen::Isometry3d& transform)
{
	const gp_Trsf trsf = isometryToGpTrsf(transform);
	BRepBuilderAPI_Transform builder(shape, trsf, true);
	return builder.Shape();
}

bool solvePointToPlaneStep(
	const std::vector<Vec3>& srcPts,
	const std::vector<Vec3>& tgtPts,
	const std::vector<Vec3>& tgtNormals,
	Eigen::Isometry3d& outStep)
{
	if (srcPts.size() < 3U || srcPts.size() != tgtPts.size() || srcPts.size() != tgtNormals.size())
	{
		return false;
	}

	Eigen::MatrixXd a(static_cast<int>(srcPts.size()), 6);
	Eigen::VectorXd b(static_cast<int>(srcPts.size()));
	for (std::size_t i = 0; i < srcPts.size(); ++i)
	{
		const Vec3& p = srcPts[i];
		const Vec3& q = tgtPts[i];
		Vec3 n = tgtNormals[i];
		if (n.norm() < 1e-9)
		{
			n = (q - p).normalized();
		}
		else
		{
			n.normalize();
		}
		const Vec3 cross = p.cross(n);
		a(static_cast<int>(i), 0) = cross.x();
		a(static_cast<int>(i), 1) = cross.y();
		a(static_cast<int>(i), 2) = cross.z();
		a(static_cast<int>(i), 3) = n.x();
		a(static_cast<int>(i), 4) = n.y();
		a(static_cast<int>(i), 5) = n.z();
		b(static_cast<int>(i)) = n.dot(q - p);
	}

	const Eigen::VectorXd x = a.colPivHouseholderQr().solve(b);
	if (!x.allFinite())
	{
		return false;
	}

	const Vec3 omega(x(0), x(1), x(2));
	const Vec3 trans(x(3), x(4), x(5));
	outStep = Eigen::Isometry3d::Identity();
	if (omega.norm() > 1e-12)
	{
		outStep.linear() = Eigen::AngleAxisd(omega.norm(), omega.normalized()).toRotationMatrix();
	}
	outStep.translation() = trans;
	return true;
}

bool faceNormalAtPointOnShape(
	const TopoDS_Shape& shape,
	const gp_Pnt& nearPoint,
	Vec3& outNormal)
{
	BRepBuilderAPI_MakeVertex vertexMaker(nearPoint);
	if (!vertexMaker.IsDone())
	{
		return false;
	}
	BRepExtrema_DistShapeShape dist(vertexMaker.Vertex(), shape);
	if (!dist.IsDone() || dist.NbSolution() < 1)
	{
		return false;
	}
	const TopoDS_Shape support = dist.SupportOnShape2(1);
	if (support.IsNull() || support.ShapeType() != TopAbs_FACE)
	{
		return false;
	}
	const TopoDS_Face face = TopoDS::Face(support);
	BRepAdaptor_Surface surf(face, true);
	Standard_Real u = 0.0;
	Standard_Real v = 0.0;
	dist.ParOnFaceS2(1, u, v);
	gp_Pnt p;
	gp_Vec d1u;
	gp_Vec d1v;
	surf.D1(u, v, p, d1u, d1v);
	gp_Vec n = d1u.Crossed(d1v);
	if (face.Orientation() == TopAbs_REVERSED)
	{
		n.Reverse();
	}
	if (n.Magnitude() < 1e-9)
	{
		return false;
	}
	outNormal = Vec3(n.X(), n.Y(), n.Z()).normalized();
	return true;
}

bool projectScanPointToShape(
	const gp_Pnt& scanPt,
	const TopoDS_Shape& trialShape,
	const double maxPairMm,
	const double minNormalDot,
	const Vec3& scanNormal,
	const bool useNormalGate,
	Vec3& outShapePoint,
	Vec3& outShapeNormal,
	double& outDistMm)
{
	BRepBuilderAPI_MakeVertex vertexMaker(scanPt);
	if (!vertexMaker.IsDone())
	{
		return false;
	}
	BRepExtrema_DistShapeShape dist(vertexMaker.Vertex(), trialShape);
	if (!dist.IsDone() || dist.NbSolution() < 1)
	{
		return false;
	}
	outDistMm = dist.Value();
	if (maxPairMm > 0.0 && outDistMm > maxPairMm)
	{
		return false;
	}
	const gp_Pnt closest = dist.PointOnShape2(1);
	outShapePoint = Vec3(closest.X(), closest.Y(), closest.Z());
	if (!faceNormalAtPointOnShape(trialShape, closest, outShapeNormal))
	{
		outShapeNormal = Vec3::Zero();
	}
	if (useNormalGate && minNormalDot > -0.999 && scanNormal.norm() > 1e-9 && outShapeNormal.norm() > 1e-9)
	{
		if (scanNormal.normalized().dot(outShapeNormal) < minNormalDot)
		{
			return false;
		}
	}
	return true;
}

double measureScanToNativeShapeMaxDistance(
	const std::vector<float>& scanXyz,
	const TopoDS_Shape& shape,
	double& outAvgDist)
{
	const std::size_t n = scanXyz.size() / 3U;
	if (n == 0U || shape.IsNull())
	{
		outAvgDist = 0.0;
		return 0.0;
	}

	double maxDist = 0.0;
	double sumDist = 0.0;
	const std::size_t maxSamples = 512U;
	const std::size_t step = std::max<std::size_t>(1U, n / maxSamples);
	std::size_t sampleCount = 0U;

	for (std::size_t i = 0; i < n; i += step)
	{
		const std::size_t b = i * 3U;
		const gp_Pnt pt(scanXyz[b], scanXyz[b + 1U], scanXyz[b + 2U]);
		BRepBuilderAPI_MakeVertex vertexMaker(pt);
		if (!vertexMaker.IsDone())
		{
			continue;
		}
		BRepExtrema_DistShapeShape dist(vertexMaker.Vertex(), shape);
		if (!dist.IsDone() || dist.NbSolution() < 1)
		{
			continue;
		}
		const double d = dist.Value();
		maxDist = std::max(maxDist, d);
		sumDist += d;
		++sampleCount;
	}

	outAvgDist = (sampleCount > 0U) ? (sumDist / static_cast<double>(sampleCount)) : 0.0;
	return maxDist;
}

} // namespace

bool applyIsometryToShapeHandle(
	const ShapeHandle& shape,
	const Eigen::Isometry3d& transform,
	ShapeHandle& outShape,
	std::string* errMsg)
{
	TopoDS_Shape native;
	if (!nativeShapeFromHandle(shape, native))
	{
		if (errMsg)
		{
			*errMsg = "shape access failed";
		}
		return false;
	}
	const TopoDS_Shape transformed = transformNativeShape(native, transform);
	outShape = ShapeHandleAccess::fromNativeShape(&transformed);
	return !outShape.isNull();
}

double measureScanToShapeMaxDistanceMm(
	const std::vector<float>& scanXyz,
	const ShapeHandle& shape,
	double& outAvgDistMm)
{
	TopoDS_Shape native;
	if (!nativeShapeFromHandle(shape, native))
	{
		outAvgDistMm = 0.0;
		return 0.0;
	}
	return measureScanToNativeShapeMaxDistance(scanXyz, native, outAvgDistMm);
}

bool rigidRegisterTemplateToScanPointToPlane(
	const std::vector<float>& scanXyz,
	const std::vector<float>& scanNormals,
	const ShapeHandle& originalTemplateShape,
	ShapeHandle& outAlignedTemplateShape,
	Eigen::Isometry3d& outTemplateToScan,
	double& outRmseMm,
	const TemplateBrepRegistrationParams& params,
	std::string* errMsg)
{
	outTemplateToScan = Eigen::Isometry3d::Identity();
	outRmseMm = 0.0;
	outAlignedTemplateShape = ShapeHandle{};

	if (scanXyz.size() < 9U || (scanXyz.size() % 3U) != 0U)
	{
		if (errMsg)
		{
			*errMsg = "invalid scan xyz buffer";
		}
		return false;
	}

	TopoDS_Shape templateNative;
	if (!nativeShapeFromHandle(originalTemplateShape, templateNative))
	{
		if (errMsg)
		{
			*errMsg = "template shape access failed";
		}
		return false;
	}

	const std::size_t nScan = scanXyz.size() / 3U;
	const std::vector<std::size_t> scanIdx = subsampleIndices(nScan, params.icpMaxPoints);
	const bool useScanNormals = scanNormals.size() == scanXyz.size() && !scanNormals.empty();
	const double minNormalDot = params.normalGateDeg > 0.0
		? std::cos(params.normalGateDeg * 3.14159265358979323846 / 180.0)
		: -1.0;
	const bool useNormalGate = params.normalGateDeg > 0.0 && useScanNormals;

	double maxPairMm = params.maxPairMm;
	if (maxPairMm <= 0.0)
	{
		const ShapeHandle::BoundsMm bounds = originalTemplateShape.boundingBoxMm();
		const double dx = bounds.maxX - bounds.minX;
		const double dy = bounds.maxY - bounds.minY;
		const double dz = bounds.maxZ - bounds.minZ;
		maxPairMm = std::sqrt(dx * dx + dy * dy + dz * dz) * 0.05;
		if (maxPairMm <= 0.0)
		{
			maxPairMm = 1.0;
		}
	}

	Eigen::Isometry3d cumulative = Eigen::Isometry3d::Identity();
	TopoDS_Shape trialShape = templateNative;
	for (int iter = 0; iter < params.maxIterations; ++iter)
	{
		std::vector<Vec3> pairedShapePts;
		std::vector<Vec3> pairedScanPts;
		std::vector<Vec3> pairedNormals;
		pairedShapePts.reserve(scanIdx.size());
		pairedScanPts.reserve(scanIdx.size());
		pairedNormals.reserve(scanIdx.size());

		for (const std::size_t i : scanIdx)
		{
			const std::size_t b = i * 3U;
			const gp_Pnt scanPt(scanXyz[b], scanXyz[b + 1U], scanXyz[b + 2U]);
			const Vec3 scanNormal = useScanNormals ? normalAt(scanNormals, i) : Vec3::Zero();

			Vec3 shapePt;
			Vec3 shapeNormal;
			double distMm = 0.0;
			if (!projectScanPointToShape(
					scanPt,
					trialShape,
					maxPairMm,
					minNormalDot,
					scanNormal,
					useNormalGate,
					shapePt,
					shapeNormal,
					distMm))
			{
				continue;
			}
			if (shapeNormal.norm() < 1e-9)
			{
				shapeNormal = (pointAt(scanXyz, i) - shapePt).normalized();
			}
			pairedShapePts.push_back(shapePt);
			pairedScanPts.push_back(pointAt(scanXyz, i));
			pairedNormals.push_back(shapeNormal);
		}

		if (pairedShapePts.size() < 3U)
		{
			if (errMsg)
			{
				*errMsg = "too few B-rep projection correspondences";
			}
			return false;
		}

		Eigen::Isometry3d step = Eigen::Isometry3d::Identity();
		if (!solvePointToPlaneStep(pairedShapePts, pairedScanPts, pairedNormals, step))
		{
			if (errMsg)
			{
				*errMsg = "point-to-plane step failed";
			}
			return false;
		}

		const double delta = step.translation().norm();
		cumulative = step * cumulative;
		trialShape = transformNativeShape(trialShape, step);
		if (delta < params.convergenceTransMm)
		{
			break;
		}
	}

	outTemplateToScan = cumulative;
	outAlignedTemplateShape = ShapeHandleAccess::fromNativeShape(&trialShape);

	double sumSq = 0.0;
	std::size_t pairs = 0U;
	for (const std::size_t i : scanIdx)
	{
		const std::size_t b = i * 3U;
		const gp_Pnt scanPt(scanXyz[b], scanXyz[b + 1U], scanXyz[b + 2U]);
		const Vec3 scanNormal = useScanNormals ? normalAt(scanNormals, i) : Vec3::Zero();
		Vec3 shapePt;
		Vec3 shapeNormal;
		double distMm = 0.0;
		if (!projectScanPointToShape(
				scanPt,
				trialShape,
				maxPairMm,
				minNormalDot,
				scanNormal,
				useNormalGate,
				shapePt,
				shapeNormal,
				distMm))
		{
			continue;
		}
		if (shapeNormal.norm() > 1e-9)
		{
			const double d = std::abs(shapeNormal.dot(pointAt(scanXyz, i) - shapePt));
			sumSq += d * d;
		}
		else
		{
			sumSq += distMm * distMm;
		}
		++pairs;
	}
	outRmseMm = pairs > 0U ? std::sqrt(sumSq / static_cast<double>(pairs)) : 0.0;
	return true;
}

} // namespace geoalgo
