Vizkit::UiLoader.register_3d_plugin('ElevationMapVisualization', 'maps', 'ElevationMapVisualization')
Vizkit::UiLoader.register_3d_plugin_for('ElevationMapVisualization', "/maps/grid/ElevationMap", :updateData )
Vizkit::UiLoader.register_3d_plugin_for('ElevationMapVisualization', "/envire/core/SpatioTemporal</maps/grid/ElevationMap>") do |plugin,sample,_|
    plugin.setVisualizationFrame(sample.frame_id)
    plugin.updateData(sample.data)
end

Vizkit::UiLoader.register_3d_plugin('GridMapVisualization', 'maps', 'GridMapVisualization')
Vizkit::UiLoader.register_3d_plugin_for('GridMapVisualization', "/maps/grid/GridMapD", :updateGridMapD )
Vizkit::UiLoader.register_3d_plugin_for('GridMapVisualization', "/envire/core/SpatioTemporal</maps/grid/GridMapD>") do |plugin,sample,_|
    plugin.setVisualizationFrame(sample.frame_id)
    plugin.updateGridMapD(sample.data)
end
Vizkit::UiLoader.register_3d_plugin_for('GridMapVisualization', "/maps/grid/GridMapF", :updateGridMapF )
Vizkit::UiLoader.register_3d_plugin_for('GridMapVisualization', "/envire/core/SpatioTemporal</maps/grid/GridMapF>") do |plugin,sample,_|
    plugin.setVisualizationFrame(sample.frame_id)
    plugin.updateGridMapF(sample.data)
end
Vizkit::UiLoader.register_3d_plugin_for('GridMapVisualization', "/maps/grid/GridMapI", :updateGridMapI )
Vizkit::UiLoader.register_3d_plugin_for('GridMapVisualization', "/envire/core/SpatioTemporal</maps/grid/GridMapI>") do |plugin,sample,_|
    plugin.setVisualizationFrame(sample.frame_id)
    plugin.updateGridMapI(sample.data)
end
Vizkit::UiLoader.register_3d_plugin_for('GridMapVisualization', "/maps/grid/GridMapC", :updateGridMapC )
Vizkit::UiLoader.register_3d_plugin_for('GridMapVisualization', "/envire/core/SpatioTemporal</maps/grid/GridMapC>") do |plugin,sample,_|
    plugin.setVisualizationFrame(sample.frame_id)
    plugin.updateGridMapC(sample.data)
end

Vizkit::UiLoader.register_3d_plugin('MLSMapVisualization', 'maps', 'MLSMapVisualization')
Vizkit::UiLoader.register_3d_plugin_for('MLSMapVisualization', "/maps/grid/MLSMapKalman", :updateMLSKalman )
Vizkit::UiLoader.register_3d_plugin_for('MLSMapVisualization', "/envire/core/SpatioTemporal</maps/grid/MLSMapKalman>") do |plugin,sample,_|
    plugin.setVisualizationFrame(sample.frame_id)
    plugin.updateMLSKalman(sample.data)
end
Vizkit::UiLoader.register_3d_plugin_for('MLSMapVisualization', "/maps/grid/MLSMapSloped", :updateMLSSloped )
Vizkit::UiLoader.register_3d_plugin_for('MLSMapVisualization', "/envire/core/SpatioTemporal</maps/grid/MLSMapSloped>") do |plugin,sample,_|
    plugin.setVisualizationFrame(sample.frame_id)
    plugin.updateMLSSloped(sample.data)
end
Vizkit::UiLoader.register_3d_plugin_for('MLSMapVisualization', "/maps/grid/MLSMapPrecalculated", :updateMLSPrecalculated )
Vizkit::UiLoader.register_3d_plugin_for('MLSMapVisualization', "/envire/core/SpatioTemporal</maps/grid/MLSMapPrecalculated>") do |plugin,sample,_|
    plugin.setVisualizationFrame(sample.frame_id)
    plugin.updateMLSPrecalculated(sample.data)
end

Vizkit::UiLoader.register_3d_plugin('ContourMapVisualization', "maps", 'ContourMapVisualization' )
Vizkit::UiLoader.register_3d_plugin_for('ContourMapVisualization', "/maps/geometric/ContourMap", :updateData )
Vizkit::UiLoader.register_3d_plugin_for('ContourMapVisualization', "/envire/core/SpatioTemporal</maps/geometric/ContourMap>") do |plugin,sample,_|
    plugin.setVisualizationFrame(sample.frame_id)
    plugin.updateData(sample.data)
end

Vizkit::UiLoader.register_3d_plugin('TraversabilityMap3dVisualization', "maps", 'TraversabilityMap3dVisualization' )
Vizkit::UiLoader.register_3d_plugin_for('TraversabilityMap3dVisualization', "/maps/grid/TraversabilityBaseMap3d", :updateData )
Vizkit::UiLoader.register_3d_plugin_for('TraversabilityMap3dVisualization', "/envire/core/SpatioTemporal</maps/grid/TraversabilityBaseMap3d>") do |plugin,sample,_|
    plugin.setVisualizationFrame(sample.frame_id)
    plugin.updateTrMap(sample.data)
end
