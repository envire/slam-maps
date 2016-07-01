Vizkit::UiLoader.register_3d_plugin('ElevationMapVisualization', 'maps', 'ElevationMapVisualization')
Vizkit::UiLoader.register_3d_plugin_for('ElevationMapVisualization', "/maps/ElevationMap", :updateData )

Vizkit::UiLoader.register_3d_plugin('MLSMapVisualization', 'maps', 'MLSMapVisualization')
Vizkit::UiLoader.register_3d_plugin_for('MLSMapVisualization', "/maps/grid/MLSMap</maps/grid/MLSConfig/KALMAN>", :updateData )
Vizkit::UiLoader.register_3d_plugin_for('MLSMapVisualization', "/maps/grid/MLSMap</maps/grid/MLSConfig/SLOPE>", :updateData )

Vizkit::UiLoader.register_3d_plugin('ContourMapVisualization', "maps", 'ContourMapVisualization' )
Vizkit::UiLoader.register_3d_plugin_for('ContourMapVisualization', "/maps/geometric/GeometricMap</maps/geometric/LineSegment3d>", :updateData )

