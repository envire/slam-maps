#ifndef PLUGINLOADER_HPP
#define PLUGINLOADER_HPP

#include <vizkit3d/Vizkit3DPlugin.hpp>

namespace vizkit3d {
    class QtPluginVizkitMaps : public vizkit3d::VizkitPluginFactory
    {
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "de.dfki.rock.QtPluginVizkitMaps")
    public:
        QtPluginVizkitMaps();
        virtual QStringList* getAvailablePlugins() const;
        virtual QObject* createPlugin(QString const& pluginName);
    };
}

#endif /*PLUGINLOADER_HPP*/
