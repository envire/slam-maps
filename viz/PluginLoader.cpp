#include <vizkit3d/Vizkit3DPlugin.hpp>
#include "ContourMapVisualization.hpp"
#include "ElevationMapVisualization.hpp"
#include "GridMapVisualization.hpp"
#include "MLSMapVisualization.hpp"
#include "TraversabilityMap3dVisualization.hpp"
#include "OccupancyGridMapVisualization.hpp"

namespace vizkit3d {
    class QtPluginVizkitMaps : public vizkit3d::VizkitPluginFactory
    {
    public:

        QtPluginVizkitMaps() {}

        /**
        * Returns a list of all available visualization plugins.
        * @return list of plugin names
        */
        virtual QStringList* getAvailablePlugins() const
        {
            QStringList *pluginNames = new QStringList();
            pluginNames->push_back("ContourMapVisualization");
            pluginNames->push_back("ElevationMapVisualization");
            pluginNames->push_back("GridMapVisualization");
            pluginNames->push_back("MLSMapVisualization");
            pluginNames->push_back("TraversabilityMap3dVisualization");
            pluginNames->push_back("OccupancyGridMapVisualization");
            return pluginNames;
        }

        virtual QObject* createPlugin(QString const& pluginName)
        {
            vizkit3d::VizPluginBase* plugin = 0;
            if (pluginName == "ContourMapVisualization")
            {
                plugin = new vizkit3d::ContourMapVisualization();
            }
            else if (pluginName == "ElevationMapVisualization")
            {
                plugin = new vizkit3d::ElevationMapVisualization();
            }
            else if (pluginName == "GridMapVisualization")
            {
                plugin = new vizkit3d::GridMapVisualization();
            }
            else if (pluginName == "MLSMapVisualization")
            {
                plugin = new vizkit3d::MLSMapVisualization();
            }
            else if (pluginName == "TraversabilityMap3dVisualization")
            {
                plugin = new vizkit3d::TraversabilityMap3dVisualization();
            }
            else if (pluginName == "OccupancyGridMapVisualization")
            {
                plugin = new OccupancyGridMapVisualization();
            }

            if (plugin)
            {
                    return plugin;
            }
            return NULL;
        };
    };
    Q_EXPORT_PLUGIN2(QtPluginVizkitMaps, QtPluginVizkitMaps)
}