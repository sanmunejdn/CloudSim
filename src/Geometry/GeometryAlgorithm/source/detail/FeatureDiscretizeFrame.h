#ifndef GEOMETRYALGORITHM_FEATUREDISCRETIZEFRAME_H
#define GEOMETRYALGORITHM_FEATUREDISCRETIZEFRAME_H

/// @file FeatureDiscretizeFrame.h
/// @brief FeatureDiscretizeFrame 接口

#include "FeatureListDocument.h"

#include <cstddef>
#include <vector>

#include <TopoDS_Face.hxx>

namespace geoalgo
{
namespace detail
{
enum class FaceNormalConvention
{
	SurfaceOutward,
	LineReverseFace,
};

struct PolylineFrameContext
{
	FaceNormalConvention normalConvention = FaceNormalConvention::LineReverseFace;
	const std::vector<TopoDS_Face>* faces = nullptr;
	bool pathClosed = false;
};

bool bestFaceNormalAtPoint(const std::vector<TopoDS_Face>& faces, const Point3d& pt, Vec3d& outNormal);

Vec3d chordTangentAt(const std::vector<Point3d>& pts, std::size_t index, bool pathClosed,
					 const std::vector<std::size_t>* segmentEndExclusive = nullptr);

Vec3d storedFaceNormal(const std::vector<TopoDS_Face>& faces, const Point3d& pt, FaceNormalConvention convention,
					   bool* found = nullptr);

void assignPathChordTangents(RawPath& path, bool pathClosed, bool outputTangent,
							 const std::vector<std::size_t>* segmentEndExclusive = nullptr,
							 bool preserveExisting = false);

void appendPolylineToRawPath(const Polyline3d& poly, RawPath& out, const DiscretizeParams& disc, bool computeFrame,
							 const PolylineFrameContext& frameCtx = {});

std::vector<TopoDS_Face> collectContextFaces(const TopoDS_Shape& shape, const std::vector<int>& edgeIndices,
											 const std::vector<int>& faceIndices, std::string* errMsg = nullptr);

} // namespace detail
} // namespace geoalgo

#endif // GEOMETRYALGORITHM_FEATUREDISCRETIZEFRAME_H
