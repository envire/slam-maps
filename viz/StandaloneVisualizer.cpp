

#include <vizkit3d/Vizkit3DWidget.hpp>
#include <vizkit3d/QtThreadedWidget.hpp>
#include "MLSMapVisualization.hpp"
#include <vizkit3d/GridVisualization.hpp>
#include <vizkit3d/OccupancyGridMapVisualization.hpp>
#include "StandaloneVisualizer.hpp"


namespace maps { namespace grid
{

class StandaloneVisualizer::Impl
{
    friend class StandaloneVisualizer;
    QtThreadedWidget<vizkit3d::Vizkit3DWidget> app;
    vizkit3d::MLSMapVisualization *mls_viz;
    vizkit3d::OccupancyGridMapVisualization *occ_viz;

    Impl()
    {
        app.start();

        //create vizkit3d plugin
        mls_viz = new vizkit3d::MLSMapVisualization();

        //create vizkit3d widget
        vizkit3d::Vizkit3DWidget *widget = app.getWidget();
        // grid plugin
        vizkit3d::GridVisualization *grid_viz = new vizkit3d::GridVisualization();
        widget->addPlugin(grid_viz);
        // add plugin
        widget->addPlugin(mls_viz);

        occ_viz = new vizkit3d::OccupancyGridMapVisualization();
        widget->addPlugin(occ_viz);
    }
};


StandaloneVisualizer::StandaloneVisualizer()
: impl(new Impl())
{

}

StandaloneVisualizer::~StandaloneVisualizer()
{
    // TODO Auto-generated destructor stub
}

void StandaloneVisualizer::updateData(const MLSMapKalman& mls)
{
    impl->mls_viz->updateData(mls);
}
void StandaloneVisualizer::updateData(const MLSMapSloped& mls)
{
    impl->mls_viz->updateMLSSloped(mls);
}
void StandaloneVisualizer::updateData(const MLSMapPrecalculated& mls)
{
    impl->mls_viz->updateMLSPrecalculated(mls);
}

void StandaloneVisualizer::updateData(const OccupancyGridMap& grid)
{
    impl->occ_viz->updateData(grid);
}

bool StandaloneVisualizer::wait(int usecs)
{
    return impl->app.isRunning() && (!usleep(usecs));
}

} /* namespace grid */
} /* namespace maps */
