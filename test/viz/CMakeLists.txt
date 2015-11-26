include_directories(BEFORE ${PROJECT_SOURCE_DIR}/viz)

rock_testsuite(testMLS testMLS.cpp
    DEPS envire_maps-viz
    DEPS_PKGCONFIG vizkit3d  vizkit3d-viz)
