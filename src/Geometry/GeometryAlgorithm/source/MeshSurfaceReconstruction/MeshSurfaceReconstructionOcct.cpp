#include "MeshSurfaceReconstructionInternal.h"



#include "detail/OccIncludes.h"



namespace geoalgo

{

namespace meshrecon

{



bool tryRebuildBsplineSurface(

	const Handle(Geom_BSplineSurface)& src,

	const TColgp_Array2OfPnt& poles,

	Handle(Geom_BSplineSurface)& outSurface)

{

	outSurface = src;

	if (src.IsNull())

	{

		return false;

	}

	if (poles.ColLength() != src->NbUPoles() || poles.RowLength() != src->NbVPoles())

	{

		return false;

	}

	try

	{

		outSurface = new Geom_BSplineSurface(

			poles,

			src->UKnots(),

			src->VKnots(),

			src->UMultiplicities(),

			src->VMultiplicities(),

			src->UDegree(),

			src->VDegree());

		return !outSurface.IsNull();

	}

	catch (...)

	{

		outSurface = src;

		return false;

	}

}



} // namespace meshrecon

} // namespace geoalgo

