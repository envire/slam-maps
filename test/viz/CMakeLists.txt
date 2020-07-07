include_directories(BEFORE ${PROJECT_SOURCE_DIR}/viz)
rock_find_qt4(QtCore QtGui)

rock_executable(heightmap_to_mls
    SOURCES
    mlsFromImageHeightmap.cpp
    HEADERS    
   DEPS_PKGCONFIG QtCore QtGui vizkit3d vizkit3d-viz
   DEPS maps-viz maps)


rock_testsuite(test_viz_elevationmap 
    test_viz_ElevationMap.cpp
    DEPS maps-viz
    DEPS_PKGCONFIG vizkit3d  vizkit3d-viz)

rock_testsuite(test_viz_mlsmap
    test_viz_MLSMap.cpp
    DEPS maps-viz
    DEPS_PKGCONFIG vizkit3d  vizkit3d-viz)

rock_testsuite(test_viz_mlslidar 
    test_viz_MLSLIDAR.cpp
    DEPS maps-viz
    DEPS_PKGCONFIG vizkit3d  vizkit3d-viz)

rock_testsuite(test_viz_countourmap
    test_viz_ContourMap.cpp
    DEPS maps-viz
    DEPS_PKGCONFIG vizkit3d  vizkit3d-viz)

rock_testsuite(test_viz_gridmap
    test_viz_GridMap.cpp
    DEPS maps-viz
    DEPS_PKGCONFIG vizkit3d  vizkit3d-viz)

rock_executable(deserialize_test 
    DeserializeVizTest.cpp
    DEPS_CMAKE Boost
    DEPS maps-viz
    DEPS_PKGCONFIG vizkit3d  vizkit3d-viz)

rock_testsuite(test_viz_tsdf_map
    test_viz_TSDFMap.cpp
    DEPS maps-viz
    DEPS_PKGCONFIG vizkit3d vizkit3d-viz)

rock_testsuite(test_viz_occupancylidar
    test_viz_OccupancyLidar.cpp
    DEPS maps-viz
    DEPS_PKGCONFIG vizkit3d  vizkit3d-viz)

rock_testsuite(test_viz_coverage
    test_viz_Coverage.cpp
    DEPS maps-viz
    DEPS_PKGCONFIG vizkit3d  vizkit3d-viz)
rock_testsuite(test_viz_traversabilitygrid
    test_viz_TraversabilityGrid.cpp
    DEPS maps-viz
    DEPS_PKGCONFIG vizkit3d vizkit3d-viz)

rock_testsuite(test_viz_traversability_map3d
    test_viz_TraversabilityMap3d.cpp
    DEPS maps-viz
    DEPS_PKGCONFIG vizkit3d vizkit3d-viz)
