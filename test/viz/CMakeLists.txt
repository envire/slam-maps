include_directories(BEFORE ${PROJECT_SOURCE_DIR}/viz)

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

rock_executable(deserialize_test 
    DeserializeVizTest.cpp
    DEPS_CMAKE Boost
    DEPS maps-viz
    DEPS_PKGCONFIG vizkit3d  vizkit3d-viz)
