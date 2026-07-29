/// @file SketchDraft.cpp



#include "SketchDraft.h"



#include "ShapeQuery.h"

#include "detail/OccIncludes.h"



#include <BRepOffsetAPI_DraftAngle.hxx>

#include <TopoDS.hxx>

#include <TopoDS_Face.hxx>

#include <cmath>



#ifndef M_PI

#define M_PI 3.14159265358979323846

#endif



namespace geoalgo

{

bool draftFacesToHandle(const ShapeHandle& base, const std::vector<int>& faceIndices, double angleDeg,

						double neutralNx, double neutralNy, double neutralNz, double ox, double oy, double oz,

						ShapeHandle& outShape, std::string* errMsg)

{

	if (std::abs(angleDeg) < 1e-6)

	{

		if (errMsg)

			*errMsg = "draft angle too small";

		return false;

	}

	TopoDS_Shape shape;

	if (!ShapeHandleAccess::nativeShape(base, &shape) || shape.IsNull())

	{

		if (errMsg)

			*errMsg = "invalid base solid";

		return false;

	}

	try

	{

		const double angleRad = angleDeg * M_PI / 180.0;

		gp_Dir pullDir(neutralNx, neutralNy, neutralNz);

		const gp_Pln neutral(gp_Pnt(ox, oy, oz), pullDir);



		BRepOffsetAPI_DraftAngle draft(shape);

		int added = 0;

		for (int idx : faceIndices)

		{

			TopoDS_Face face;

			if (!shapeFaceAtIndex(shape, idx, face, nullptr))

				continue;

			draft.Add(face, pullDir, angleRad, neutral, Standard_True);

			if (draft.AddDone())

				++added;

			else

				draft.Remove(face);

		}

		if (added == 0)

		{

			if (errMsg)

				*errMsg = "no valid faces for draft";

			return false;

		}

		draft.Build();

		if (!draft.IsDone())

		{

			if (errMsg)

				*errMsg = "draft failed";

			return false;

		}

		const TopoDS_Shape result = draft.Shape();

		outShape = ShapeHandleAccess::fromNativeShape(&result);

		return !outShape.isNull();

	}

	catch (...)

	{

		if (errMsg)

			*errMsg = "draft exception";

		return false;

	}

}



} // namespace geoalgo


