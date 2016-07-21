Vizkit::UiLoader.register_3d_plugin('ElevationMapVisualization', 'maps', 'ElevationMapVisualization')
Vizkit::UiLoader.register_3d_plugin_for('ElevationMapVisualization', "/maps/ElevationMap", :updateData )
Vizkit::UiLoader.register_3d_plugin_for('ElevationMapVisualization', "/envire/core/SpatioTemporal</maps/ElevationMap>") do |plugin,sample,_|
    if plugin.getVisualizationFrames.include? sample.frame_id
        plugin.setVisualizationFrame(sample.frame_id)
    end
    plugin.updateData(sample.data)
end

Vizkit::UiLoader.register_3d_plugin('GridMapVisualization', 'maps', 'GridMapVisualization')
Vizkit::UiLoader.register_3d_plugin_for('GridMapVisualization', "/maps/GridMap<double>", :updateData )
Vizkit::UiLoader.register_3d_plugin_for('GridMapVisualization', "/envire/core/SpatioTemporal</maps/GridMap<double>>") do |plugin,sample,_|
    if plugin.getVisualizationFrames.include? sample.frame_id
        plugin.setVisualizationFrame(sample.frame_id)
    end
    plugin.updateData(sample.data)
end

Vizkit::UiLoader.register_3d_plugin('GridMapVisualization', 'maps', 'GridMapVisualization')
Vizkit::UiLoader.register_3d_plugin_for('GridMapVisualization', "/maps/GridMap<float>", :updateData )
Vizkit::UiLoader.register_3d_plugin_for('GridMapVisualization', "/envire/core/SpatioTemporal</maps/GridMap<float>>") do |plugin,sample,_|
    if plugin.getVisualizationFrames.include? sample.frame_id
        plugin.setVisualizationFrame(sample.frame_id)
    end
    plugin.updateData(sample.data)
end

Vizkit::UiLoader.register_3d_plugin('GridMapVisualization', 'maps', 'GridMapVisualization')
Vizkit::UiLoader.register_3d_plugin_for('GridMapVisualization', "/maps/GridMap<int>", :updateData )
Vizkit::UiLoader.register_3d_plugin_for('GridMapVisualization', "/envire/core/SpatioTemporal</maps/GridMap<int>>") do |plugin,sample,_|
    if plugin.getVisualizationFrames.include? sample.frame_id
        plugin.setVisualizationFrame(sample.frame_id)
    end
    plugin.updateData(sample.data)
end

Vizkit::UiLoader.register_3d_plugin('GridMapVisualization', 'maps', 'GridMapVisualization')
Vizkit::UiLoader.register_3d_plugin_for('GridMapVisualization', "/maps/GridMap<char>", :updateData )
Vizkit::UiLoader.register_3d_plugin_for('GridMapVisualization', "/envire/core/SpatioTemporal</maps/GridMap<char>>") do |plugin,sample,_|
    if plugin.getVisualizationFrames.include? sample.frame_id
        plugin.setVisualizationFrame(sample.frame_id)
    end
    plugin.updateData(sample.data)
end

Vizkit::UiLoader.register_3d_plugin('MLSMapVisualization', 'maps', 'MLSMapVisualization')
Vizkit::UiLoader.register_3d_plugin_for('MLSMapVisualization', "/maps/grid/MLSMap</maps/grid/MLSConfig/KALMAN>", :updateData )
Vizkit::UiLoader.register_3d_plugin_for('MLSMapVisualization', "/envire/core/SpatioTemporal</maps/grid/MLSMap</maps/grid/MLSConfig/KALMAN>>") do |plugin,sample,_|
    if plugin.getVisualizationFrames.include? sample.frame_id
        plugin.setVisualizationFrame(sample.frame_id)
    end
    plugin.updateData(sample.data)
end
Vizkit::UiLoader.register_3d_plugin_for('MLSMapVisualization', "/maps/grid/MLSMap</maps/grid/MLSConfig/SLOPE>", :updateData )
Vizkit::UiLoader.register_3d_plugin_for('MLSMapVisualization', "/envire/core/SpatioTemporal</maps/grid/MLSMap</maps/grid/MLSConfig/SLOPE>>") do |plugin,sample,_|
    if plugin.getVisualizationFrames.include? sample.frame_id
        plugin.setVisualizationFrame(sample.frame_id)
    end
    plugin.updateData(sample.data)
end

Vizkit::UiLoader.register_3d_plugin('ContourMapVisualization', "maps", 'ContourMapVisualization' )
Vizkit::UiLoader.register_3d_plugin_for('ContourMapVisualization', "/maps/geometric/GeometricMap</maps/geometric/LineSegment3d>", :updateData )
Vizkit::UiLoader.register_3d_plugin_for('ContourMapVisualization', "/envire/core/SpatioTemporal</maps/geometric/GeometricMap</maps/geometric/LineSegment3d>>") do |plugin,sample,_|
    if plugin.getVisualizationFrames.include? sample.frame_id
        plugin.setVisualizationFrame(sample.frame_id)
    end
    plugin.updateData(sample.data)
end