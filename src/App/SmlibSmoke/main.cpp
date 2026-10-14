#include <SmApiGeneral.h>
#include <SmApiPrimitives.h>
#include <SmApiQueries.h>
#include <SmBrep.h>
#include <SmVector3d.h>

#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#ifndef CLOUDSIM_HAS_SMLIB
#error SmlibSmoke requires CLOUDSIM_HAS_SMLIB (bin/SDK/smlib/install)
#endif

namespace {

void Check(SmApiStatus status, const char* op)
{
    if (status != SM_SUCCESS) {
        throw std::runtime_error(std::string(op) + " status=" + std::to_string(static_cast<int>(status)));
    }
}

} // namespace

int main()
{
    try {
        // 进程内先建默认 context，避免后续 API 懒初始化竞态
        SmApiCreateContext();

        SmBrep* raw = nullptr;
        Check(SmApiCreateBox(SmVector3d(0, 0, 0), 10.0, 10.0, 10.0, raw), "SmApiCreateBox");
        std::unique_ptr<SmBrep> box(raw);

        SmBoolean manifold = FALSE;
        Check(SmApiBrepIsManifoldSolid(box.get(), manifold), "SmApiBrepIsManifoldSolid");
        if (manifold != TRUE) {
            throw std::runtime_error("box is not manifold");
        }

        double volume = 0.0;
        Check(SmApiBrepComputeVolume(box.get(), 1.0e-7, volume), "SmApiBrepComputeVolume");
        if (!std::isfinite(volume) || std::abs(volume - 1000.0) > 1.0e-3) {
            throw std::runtime_error("unexpected volume");
        }

        SmPoint3d mn, mx;
        Check(SmApiBrepBoundingBox(box.get(), FALSE, mn, mx), "SmApiBrepBoundingBox");

        std::cout << "SmlibSmoke OK volume=" << volume
                  << " bounds=(" << mn[0] << "," << mn[1] << "," << mn[2] << ")-("
                  << mx[0] << "," << mx[1] << "," << mx[2] << ")\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "SmlibSmoke failed: " << e.what() << '\n';
        return 1;
    }
}
