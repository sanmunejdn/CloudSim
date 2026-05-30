#include "detail/OccIncludes.h"

#include "ShapeIo.h"

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

} // namespace geoalgo
