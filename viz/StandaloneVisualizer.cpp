

#include <vizkit3d/Vizkit3DWidget.hpp>
#include <vizkit3d/QtThreadedWidget.hpp>
#include "MLSGridVisualization.hpp"
#include <vizkit3d/GridVisualization.hpp>
#include "StandaloneVisualizer.hpp"



namespace maps
{

class StandaloneVisualizer::Impl
{
    friend class StandaloneVisualizer;
    QtThreadedWidget<vizkit3d::Vizkit3DWidget> app;
    vizkit3d::MLSGridVisualization *mls_viz;

    Impl()
    {
        app.start();

        //create vizkit3d plugin
        mls_viz = new vizkit3d::MLSGridVisualization();

        //create vizkit3d widget
        vizkit3d::Vizkit3DWidget *widget = app.getWidget();
        // grid plugin
        vizkit3d::GridVisualization *grid_viz = new vizkit3d::GridVisualization();
        widget->addPlugin(grid_viz);
        // add plugin
        widget->addPlugin(mls_viz);
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

void StandaloneVisualizer::updateData(const MLSGrid& mls)
{
    impl->mls_viz->updateData(mls);
}

bool StandaloneVisualizer::wait(int usecs)
{
    return impl->app.isRunning() && (!usleep(usecs));
}

} /* namespace maps */
