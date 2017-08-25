#include <maps/grid/MLSMap.hpp>

// TODO make configurable
// TODO move source to .cpp file

namespace maps {

// the simplest ML-Map (only vertical intervals):
typedef grid::MLSMap<grid::MLSConfig::BASE> CoverageMap3d;

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

    void addCoverage(const double &radius, const base::AngleSegment& range /* ignored */, const base::Pose& pose_in_map);
    const CoverageMap3d& getCoverage() const { return coverage; }
};

}  // namespace operations
}  // namespace maps
