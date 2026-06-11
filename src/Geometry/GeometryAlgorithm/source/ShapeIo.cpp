#include "detail/OccIncludes.h"

#include "ShapeIo.h"

#include <BRepTools.hxx>

namespace geoalgo
{

bool readStepShape(const std::string& pathLocal, TopoDS_Shape& outShape, std::string* errMsg)
{
	STEPControl_Reader reader;
	const IFSelect_ReturnStatus status = reader.ReadFile(pathLocal.c_str());
	if (status != IFSelect_RetDone)
	{
		if (errMsg)
		{
			*errMsg = "OCCT STEP read failed";
		}
		return false;
	}
	if (!reader.TransferRoots())
	{
		if (errMsg)
		{
			*errMsg = "OCCT STEP transfer failed";
		}
		return false;
	}
	outShape = reader.OneShape();
	if (outShape.IsNull())
	{
		if (errMsg)
		{
			*errMsg = "OCCT STEP produced empty shape";
		}
		return false;
	}
	return true;
}

bool readStepIntoHandle(const std::string& pathLocal, ShapeHandle& outShape, std::string* errMsg)
{
	TopoDS_Shape shape;
	if (!readStepShape(pathLocal, shape, errMsg))
	{
		return false;
	}
	outShape = ShapeHandleAccess::fromNativeShape(&shape);
	return !outShape.isNull();
}

bool readBrepFile(const std::string& pathLocal, ShapeHandle& outShape, std::string* errMsg)
{
	outShape = ShapeHandle{};
	TopoDS_Shape shape;
	BRep_Builder builder;
	const Standard_Boolean ok = BRepTools::Read(shape, pathLocal.c_str(), builder);
	if (!ok || shape.IsNull())
	{
		if (errMsg)
		{
			*errMsg = "OCCT BREP read failed";
		}
		return false;
	}
	outShape = ShapeHandleAccess::fromNativeShape(&shape);
	return true;
}

bool writeBrepFile(const std::string& pathLocal, const ShapeHandle& shape, std::string* errMsg)
{
	if (shape.isNull())
	{
		if (errMsg)
		{
			*errMsg = "null shape";
		}
		return false;
	}
	TopoDS_Shape native;
	if (!ShapeHandleAccess::nativeShape(shape, &native))
	{
		if (errMsg)
		{
			*errMsg = "shape access failed";
		}
		return false;
	}
	const Standard_Boolean ok = BRepTools::Write(native, pathLocal.c_str());
	if (!ok)
	{
		if (errMsg)
		{
			*errMsg = "OCCT BREP write failed";
		}
		return false;
	}
	return true;
}

bool writeStepFile(const std::string& pathLocal, const ShapeHandle& shape, std::string* errMsg)
{
	if (shape.isNull())
	{
		if (errMsg)
		{
			*errMsg = "null shape";
		}
		return false;
	}
	TopoDS_Shape native;
	if (!ShapeHandleAccess::nativeShape(shape, &native))
	{
		if (errMsg)
		{
			*errMsg = "shape access failed";
		}
		return false;
	}
	STEPControl_Writer writer;
	const IFSelect_ReturnStatus transferStatus = writer.Transfer(native, STEPControl_AsIs);
	if (transferStatus != IFSelect_RetDone)
	{
		if (errMsg)
		{
			*errMsg = "OCCT STEP transfer failed";
		}
		return false;
	}
	const IFSelect_ReturnStatus writeStatus = writer.Write(pathLocal.c_str());
	if (writeStatus != IFSelect_RetDone)
	{
		if (errMsg)
		{
			*errMsg = "OCCT STEP write failed";
		}
		return false;
	}
	return true;
}

} // namespace geoalgo
