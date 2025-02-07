include_directories(BEFORE ${PROJECT_SOURCE_DIR}/viz)
rock_find_qt4(OPTIONAL)
rock_find_qt5(OPTIONAL)

add_compile_options(-fPIC)

if (ROCK_QT_VERSION_4)
  rock_executable(heightmap_to_mls
    SOURCES
    mlsFromImageHeightmap.cpp
    DEPS maps-viz maps)

  macro(mapviz_test_qt4 name sourcefile)
    rock_testsuite(${name}
        ${sourcefile}
        DEPS maps-viz)
  endmacro()
else()
  macro(mapviz_test_qt4)
  endmacro()
endif()

if (ROCK_QT_VERSION_5)
  rock_executable(heightmap_to_mls-qt5
    SOURCES
    mlsFromImageHeightmap.cpp
    DEPS maps-viz-qt5 maps)

  macro(mapviz_test_qt5 name sourcefile)
    rock_testsuite(${name}-qt5
        ${sourcefile}
        DEPS maps-viz-qt5)
  endmacro()
else()
  macro(mapviz_test_qt5)
  endmacro()
endif()

macro(mapviz_test name source)
  mapviz_test_qt4(${name} ${source})
  mapviz_test_qt5(${name} ${source})
endmacro()

mapviz_test(test_viz_elevationmap 
    test_viz_ElevationMap.cpp
)

mapviz_test(test_viz_mlsmap
    test_viz_MLSMap.cpp
)

mapviz_test(test_viz_mlslidar 
    test_viz_MLSLIDAR.cpp
)

mapviz_test(test_viz_countourmap
    test_viz_ContourMap.cpp
)

mapviz_test(test_viz_gridmap
    test_viz_GridMap.cpp
)

mapviz_test(deserialize_test 
    DeserializeVizTest.cpp
)

mapviz_test(test_viz_tsdf_map
    test_viz_TSDFMap.cpp
)

mapviz_test(test_viz_occupancylidar
    test_viz_OccupancyLidar.cpp
)

mapviz_test(test_viz_coverage
    test_viz_Coverage.cpp
)

mapviz_test(test_viz_traversabilitygrid
    test_viz_TraversabilityGrid.cpp
)

mapviz_test(test_viz_traversability_map3d
    test_viz_TraversabilityMap3d.cpp
)
