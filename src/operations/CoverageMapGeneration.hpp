#pragma once

#include <maps/grid/MLSMap.hpp>

// TODO make configurable?
// TODO MLSMap is currently ignored

namespace maps {

// the simplest ML-Map (only vertical intervals):
typedef grid::MLSMapBase CoverageMap3d;

namespace operations {

class CoverageTracker {
    CoverageMap3d coverage;
    const grid::MLSMapKalman* mls;

    bool frameChanged(const grid::MLSMapKalman& mls_) const;


public:
    CoverageTracker() : mls(nullptr)
    {

    }

    void updateMLS(const grid::MLSMapKalman& mls_);

    template <typename CellT, typename GridT>
    void setFrame(const grid::GridMap<CellT, GridT>& map)
    {
        coverage.resize(map.getNumCells());
        coverage.setResolution(map.getResolution());
        coverage.getLocalFrame() = map.getLocalFrame();
    }

    void addCoverage(const double &radius, const base::AngleSegment& range /* ignored */, const base::Pose& pose_in_map);
    const CoverageMap3d& getCoverage() const { return coverage; }
};

}  // namespace operations
}  // namespace maps
