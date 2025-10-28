#ifndef PLUGINLOADER_HPP
#define PLUGINLOADER_HPP

#include <vizkit3d/Vizkit3DPlugin.hpp>

namespace vizkit3d {
    class QtPluginVizkitMaps : public vizkit3d::VizkitPluginFactory
    {
      Q_OBJECT
#if QT_VERSION >= 0x050000
      Q_PLUGIN_METADATA(IID "rock.vizkit3d.VizkitPluginFactory")
#endif
    public:
        QtPluginVizkitMaps();
        virtual QStringList* getAvailablePlugins() const;
        virtual QObject* createPlugin(QString const& pluginName);
    };
}

#endif /*PLUGINLOADER_HPP*/
