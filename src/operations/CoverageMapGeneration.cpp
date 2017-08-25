#include "CoverageMapGeneration.hpp"

namespace maps {

namespace operations {

bool CoverageTracker::frameChanged(const grid::MLSMapKalman& mls_) const
{
    return ! (coverage.getLocalFrame().isApprox(mls_.getLocalFrame())
            && coverage.getResolution().isApprox(mls_.getResolution())
            && coverage.getNumCells() == mls_.getNumCells());
}


void CoverageTracker::updateMLS(const grid::MLSMapKalman& mls_) {
    mls = &mls_;
    if(frameChanged(*mls))
    {
        std::cout << "Frame changed. Resetting coverage. New Frame\n" << mls->getLocalFrame().matrix().topRows<3>() << "\nResolution: " << mls->getResolution().transpose() << ", NumCells: " << mls->getNumCells() << "\n";
        grid::MLSConfig cfg = mls->getConfig();
        cfg.updateModel = grid::MLSConfig::BASE;

        // Just reset the coverage map for now
        // TODO FIXME properly move the current contents of the coverage map
        coverage = CoverageMap3d(mls->getNumCells(), mls->getResolution(), cfg);
        coverage.getLocalFrame() = mls->getLocalFrame();
    }
}

void CoverageTracker::addCoverage(const double &radius, const base::AngleSegment& range /* ignored */, const base::Pose& pose_in_map)
{
    if(frameChanged(*mls) )
    {
        std::cerr << "Local Frames and resolution of coverage map and MLS map must be the same.\n";
        updateMLS(*mls); // HACK should not be necessary
//            throw std::runtime_error( "Local Frames and resolution of coverage map and MLS map must be the same.");
    }

    const Eigen::Array2d res = mls->getResolution();
    const Eigen::Affine3d pose_in_grid = mls->getLocalFrame() * pose_in_map.toTransform();
    const Eigen::Array2d pos2d = pose_in_grid.translation().head<2>().array();
    const double z = pose_in_grid.translation().z();

//    const auto config = mls.getConfig();

    const Eigen::Array2i minIdx = ((pos2d - radius) / res).cast<int>().max(0);
    const Eigen::Array2i maxIdx = ((pos2d + radius) / res).cast<int>().min(mls->getNumCells().array().cast<int>());

    for(int x=minIdx.x(); x < maxIdx.x(); ++x)
    {
        for(int y=minIdx.y(); y< maxIdx.y(); ++y)
        {
            Eigen::Array2i idx(x,y);

            const double z_diff_2 = radius*radius - (idx.cast<double>() * res - pos2d).matrix().squaredNorm();
            if(z_diff_2 <= 0) continue;

            const double z_diff = std::sqrt(z_diff_2);
            coverage.mergePatch(grid::Index(idx.matrix()), CoverageMap3d::Patch(z+z_diff, 2*z_diff));
        }
    }
}



}  // namespace operations
}  // namespace maps
