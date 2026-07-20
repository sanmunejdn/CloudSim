/// @file ImBatchBridge.cpp
/// @brief ImBatchBridge 实现

#include "InstantMeshesCore.h"

#if defined(INSTANT_MESHES_HAS_LIB)

#include "batch.h"

#include <fstream>
#include <iostream>
#include <sstream>
#include <streambuf>

// field.cpp Optimizer 依赖 main.cpp 中的全局线程数
int nprocs = -1;

namespace instant_meshes
{
namespace
{
class NullBuffer : public std::streambuf
{
protected:
	int overflow(const int c) override { return c; }
};

class ScopedImSilence
{
public:
	ScopedImSilence()
	{
		prevCout_ = std::cout.rdbuf(&null_);
		prevCerr_ = std::cerr.rdbuf(&null_);
	}
	~ScopedImSilence()
	{
		std::cout.rdbuf(prevCout_);
		std::cerr.rdbuf(prevCerr_);
	}

private:
	NullBuffer null_;
	std::streambuf* prevCout_ = nullptr;
	std::streambuf* prevCerr_ = nullptr;
};

} // namespace

bool remeshViaInProcessBatch(const std::string& inObj, const std::string& outObj, const Params& params,
							 std::string* errMsg)
{
	try
	{
		ScopedImSilence silence;
		const int vertexCount = params.targetVertexCount > 0 ? params.targetVertexCount : -1;
		batch_process(inObj, outObj, 4, 4, -1.f, -1, vertexCount, params.creaseAngleDeg, true, false, 0, 0,
					  params.pureQuad, params.deterministic);
	}
	catch (const std::exception& ex)
	{
		if (errMsg)
		{
			*errMsg = std::string("instant meshes batch_process exception: ") + ex.what();
		}
		return false;
	}
	catch (...)
	{
		if (errMsg)
		{
			*errMsg = "instant meshes batch_process unknown exception";
		}
		return false;
	}
	return true;
}

} // namespace instant_meshes

#endif // INSTANT_MESHES_HAS_LIB
