Vizkit::UiLoader.register_3d_plugin('ElevationMapVisualization', 'maps', 'ElevationMapVisualization')
Vizkit::UiLoader.register_3d_plugin_for('ElevationMapVisualization', "/maps/grid/ElevationMap", :updateData )
Vizkit::UiLoader.register_3d_plugin_for('ElevationMapVisualization', "/envire/core/SpatioTemporal</maps/grid/ElevationMap>") do |plugin,sample,_|
    if plugin.getVisualizationFrames.include? sample.frame_id
        plugin.setVisualizationFrame(sample.frame_id)
    end
    plugin.updateData(sample.data)
end

Vizkit::UiLoader.register_3d_plugin('GridMapVisualization', 'maps', 'GridMapVisualization')
Vizkit::UiLoader.register_3d_plugin_for('GridMapVisualization', "/maps/grid/GridMapD", :updateGridMapD )
Vizkit::UiLoader.register_3d_plugin_for('GridMapVisualization', "/envire/core/SpatioTemporal</maps/grid/GridMapD>") do |plugin,sample,_|
    if plugin.getVisualizationFrames.include? sample.frame_id
        plugin.setVisualizationFrame(sample.frame_id)
    end
    plugin.updateGridMapD(sample.data)
end
Vizkit::UiLoader.register_3d_plugin_for('GridMapVisualization', "/maps/grid/GridMapF", :updateGridMapF )
Vizkit::UiLoader.register_3d_plugin_for('GridMapVisualization', "/envire/core/SpatioTemporal</maps/grid/GridMapF>") do |plugin,sample,_|
    if plugin.getVisualizationFrames.include? sample.frame_id
        plugin.setVisualizationFrame(sample.frame_id)
    end
    plugin.updateGridMapF(sample.data)
end
Vizkit::UiLoader.register_3d_plugin_for('GridMapVisualization', "/maps/grid/GridMapI", :updateGridMapI )
Vizkit::UiLoader.register_3d_plugin_for('GridMapVisualization', "/envire/core/SpatioTemporal</maps/grid/GridMapI>") do |plugin,sample,_|
    if plugin.getVisualizationFrames.include? sample.frame_id
        plugin.setVisualizationFrame(sample.frame_id)
    end
    plugin.updateGridMapI(sample.data)
end
Vizkit::UiLoader.register_3d_plugin_for('GridMapVisualization', "/maps/grid/GridMapC", :updateGridMapC )
Vizkit::UiLoader.register_3d_plugin_for('GridMapVisualization', "/envire/core/SpatioTemporal</maps/grid/GridMapC>") do |plugin,sample,_|
    if plugin.getVisualizationFrames.include? sample.frame_id
        plugin.setVisualizationFrame(sample.frame_id)
    end
    plugin.updateGridMapC(sample.data)
end

Vizkit::UiLoader.register_3d_plugin('MLSMapVisualization', 'maps', 'MLSMapVisualization')
Vizkit::UiLoader.register_3d_plugin_for('MLSMapVisualization', "/maps/grid/MLSMapKalman", :updateData )
Vizkit::UiLoader.register_3d_plugin_for('MLSMapVisualization', "/envire/core/SpatioTemporal</maps/grid/MLSMapKalman>") do |plugin,sample,_|
    if plugin.getVisualizationFrames.include? sample.frame_id
        plugin.setVisualizationFrame(sample.frame_id)
    end
    plugin.updateData(sample.data)
end
Vizkit::UiLoader.register_3d_plugin_for('MLSMapVisualization', "/maps/grid/MLSMapSloped", :updateMLSSloped )
Vizkit::UiLoader.register_3d_plugin_for('MLSMapVisualization', "/envire/core/SpatioTemporal</maps/grid/MLSMapSloped>") do |plugin,sample,_|
    if plugin.getVisualizationFrames.include? sample.frame_id
        plugin.setVisualizationFrame(sample.frame_id)
    end
    plugin.updateMLSSloped(sample.data)
end
Vizkit::UiLoader.register_3d_plugin_for('MLSMapVisualization', "/maps/grid/MLSMapPrecalculated", :updateMLSPrecalculated )
Vizkit::UiLoader.register_3d_plugin_for('MLSMapVisualization', "/envire/core/SpatioTemporal</maps/grid/MLSMapPrecalculated>") do |plugin,sample,_|
    if plugin.getVisualizationFrames.include? sample.frame_id
        plugin.setVisualizationFrame(sample.frame_id)
    end
    plugin.updateMLSPrecalculated(sample.data)
end

Vizkit::UiLoader.register_3d_plugin('ContourMapVisualization', "maps", 'ContourMapVisualization' )
Vizkit::UiLoader.register_3d_plugin_for('ContourMapVisualization', "/maps/geometric/ContourMap", :updateData )
Vizkit::UiLoader.register_3d_plugin_for('ContourMapVisualization', "/envire/core/SpatioTemporal</maps/geometric/ContourMap>") do |plugin,sample,_|
    if plugin.getVisualizationFrames.include? sample.frame_id
        plugin.setVisualizationFrame(sample.frame_id)
    end
    plugin.updateData(sample.data)
end
Vizkit::UiLoader.register_3d_plugin('OccupancyGridMapVisualization', 'maps', 'OccupancyGridMapVisualization')
Vizkit::UiLoader.register_3d_plugin_for('OccupancyGridMapVisualization', "/maps/grid/OccupancyGridMap", :updateData )
Vizkit::UiLoader.register_3d_plugin_for('OccupancyGridMapVisualization', "/envire/core/SpatioTemporal</maps/grid/OccupancyGridMap>") do |plugin,sample,_|
    if plugin.getVisualizationFrames.include? sample.frame_id
        plugin.setVisualizationFrame(sample.frame_id)
    end
    plugin.updateData(sample.data)
end