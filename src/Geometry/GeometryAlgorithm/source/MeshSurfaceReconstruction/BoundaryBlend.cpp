#include "MeshSurfaceReconstructionInternal.h"



#include "detail/OccIncludes.h"



#include <TColgp_Array2OfPnt.hxx>



#include <cmath>



namespace geoalgo

{

namespace meshrecon

{



namespace

{



// 论文 3.3.5：7 次边界混合权（简化固定控制点）

double boundaryBlendWeight(const double t)

{

	const double s = std::max(0.0, std::min(1.0, t));

	return s * s * s * (s * (s * 6.0 - 15.0) + 10.0);

}



} // namespace



bool applyBoundaryC2Blend(

	std::vector<QuadPatch>& patches,

	const MeshSurfaceReconstructParams& params,

	bool& outBlendOk,

	std::string* errMsg)

{

	(void)params;

	outBlendOk = true;



	for (std::size_t i = 0; i < patches.size(); ++i)

	{

		for (const int nb : patches[i].neighborPatchIds)

		{

			if (nb < 0 || static_cast<std::size_t>(nb) >= patches.size() || static_cast<std::size_t>(nb) <= i)

			{

				continue;

			}

			QuadPatch& a = patches[i];

			QuadPatch& b = patches[static_cast<std::size_t>(nb)];

			if (a.surface.IsNull() || b.surface.IsNull())

			{

				continue;

			}

			TColgp_Array2OfPnt pa = a.surface->Poles();

			TColgp_Array2OfPnt pb = b.surface->Poles();

			const int uMax = std::min(pa.UpperRow() - pa.LowerRow() + 1, pb.UpperRow() - pb.LowerRow() + 1);

			const int vMax = std::min(pa.UpperCol() - pa.LowerCol() + 1, pb.UpperCol() - pb.LowerCol() + 1);

			if (uMax < 2 || vMax < 1)

			{

				continue;

			}

			for (int k = 1; k <= std::min(3, uMax); ++k)

			{

				const double w = boundaryBlendWeight(static_cast<double>(k) / 3.0);

				for (int j = 1; j <= vMax; ++j)

				{

					const int vIdx = pa.LowerCol() + j - 1;

					const gp_Pnt pA = pa.Value(pa.UpperRow() - k + 1, vIdx);

					const gp_Pnt pB = pb.Value(pb.LowerRow() + k - 1, vIdx);

					const gp_Pnt blend(

						pA.X() * w + pB.X() * (1.0 - w),

						pA.Y() * w + pB.Y() * (1.0 - w),

						pA.Z() * w + pB.Z() * (1.0 - w));

					pa.SetValue(pa.UpperRow() - k + 1, vIdx, blend);

					pb.SetValue(pb.LowerRow() + k - 1, vIdx, blend);

				}

			}

			if (!tryRebuildBsplineSurface(a.surface, pa, a.surface)

				|| !tryRebuildBsplineSurface(b.surface, pb, b.surface))

			{

				outBlendOk = false;

			}

		}

	}



	(void)errMsg;

	return true;

}



} // namespace meshrecon

} // namespace geoalgo

