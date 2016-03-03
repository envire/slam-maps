find_package(Boost REQUIRED COMPONENTS system filesystem serialization)

find_package(CGAL REQUIRED COMPONENTS Core)
include(${CGAL_USE_FILE})

rock_library(envire_maps
    SOURCES
        Grid.cpp
        SurfacePatch.cpp
        ElevationMap.cpp
        MLSGrid.cpp
        tools/BresenhamLine.cpp
        TraversabilityMap3d.cpp
    HEADERS
        LocalMap.hpp
        Grid.hpp
        GridMap.hpp
        MultilayerGridMap.hpp
        ElevationMap.hpp
        SurfacePatch.hpp
        SurfacePatches.hpp
        List.hpp
        SPList.hpp
        MLSConfig.hpp
        MLSGrid.hpp
        MLSGridI.hpp
        TraversabilityMap3d.hpp
        tools/BresenhamLine.hpp
        operations/GridInterpolation.hpp
    DEPS_CMAKE
        Boost
    DEPS_PKGCONFIG 
        base-types 
        pcl_registration-1.7
    DEPS_PLAIN
        CGAL)
