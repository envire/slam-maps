Vizkit::UiLoader.register_3d_plugin('ElevationMapVisualization', 'maps', 'ElevationMapVisualization')
Vizkit::UiLoader.register_3d_plugin_for('ElevationMapVisualization', "/envire/maps/ElevationMap", :updateData )

Vizkit::UiLoader.register_3d_plugin('MLSGridVisualization', 'maps', 'MLSGridVisualization')
Vizkit::UiLoader.register_3d_plugin_for('MLSGridVisualization', "/envire/maps/MLSGrid", :updateData )
