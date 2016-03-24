include_directories(BEFORE ${PROJECT_SOURCE_DIR}/viz)

rock_testsuite(testElevation testElevation.cpp
    DEPS maps-viz
    DEPS_PKGCONFIG vizkit3d  vizkit3d-viz)

rock_testsuite(testMLS testMLS.cpp
    DEPS maps-viz
    DEPS_PKGCONFIG vizkit3d  vizkit3d-viz)

rock_testsuite(testMLS_LIDAR testMLS_LIDAR.cpp
    DEPS maps-viz
    DEPS_PKGCONFIG vizkit3d  vizkit3d-viz)
