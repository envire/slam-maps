//
// Copyright (c) 2015-2017, Deutsches Forschungszentrum für Künstliche Intelligenz GmbH.
// Copyright (c) 2015-2017, University of Bremen
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//

#include <vizkit3d/Vizkit3DWidget.hpp>
#include <vizkit3d/QtThreadedWidget.hpp>
#include "MLSMapVisualization.hpp"
#include <vizkit3d/GridVisualization.hpp>
#include <vizkit3d/OccupancyGridMapVisualization.hpp>
#include <vizkit3d/TraversabilityGridVisualization.hpp>
#include <vizkit3d/TraversabilityMap3dVisualization.hpp>
#include "StandaloneVisualizer.hpp"


namespace maps { namespace grid
{

class StandaloneVisualizer::Impl
{
    friend class StandaloneVisualizer;
    QtThreadedWidget<vizkit3d::Vizkit3DWidget> app;
    vizkit3d::MLSMapVisualization *mls_viz;
    vizkit3d::OccupancyGridMapVisualization *occ_viz;
    vizkit3d::TraversabilityGridVisualization *trav_viz;
    vizkit3d::TraversabilityMap3dVisualization *trav_3d_viz;

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

        trav_viz = new vizkit3d::TraversabilityGridVisualization();
        widget->addPlugin(trav_viz);

        trav_3d_viz = new vizkit3d::TraversabilityMap3dVisualization();
        widget->addPlugin(trav_3d_viz);
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

void StandaloneVisualizer::updateData(const MLSMapBase& mls)
{
    impl->mls_viz->updateMLSBase(mls);
}

void StandaloneVisualizer::updateData(const OccupancyGridMap& grid)
{
    impl->occ_viz->updateData(grid);
}

void StandaloneVisualizer::updateData(const TraversabilityGrid& travGrid)
{
    impl->trav_viz->updateData(travGrid);
}

void StandaloneVisualizer::updateData(const maps::grid::TraversabilityMap3d< maps::grid::TraversabilityNodeBase* >& travGrid)
{
    impl->trav_3d_viz->updateData(travGrid);
}

bool StandaloneVisualizer::wait(int usecs)
{
    return impl->app.isRunning() && (!usleep(usecs));
}

} /* namespace grid */
} /* namespace maps */
