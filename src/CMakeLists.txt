find_package(Boost REQUIRED COMPONENTS system filesystem serialization)

find_package(CGAL REQUIRED COMPONENTS Core)

rock_library(maps
    SOURCES
        grid/ElevationMap.cpp
        grid/TraversabilityMap3d.cpp
        tools/BresenhamLine.cpp        
    HEADERS
        LocalMap.hpp
        grid/Index.hpp
        grid/GridMap.hpp
        grid/MultilayerGridMap.hpp
        grid/ElevationMap.hpp
        grid/SurfacePatches.hpp
        grid/LevelList.hpp
        grid/MLSConfig.hpp
        grid/MLSMap.hpp
        grid/MLGrid.hpp
        grid/TraversabilityMap3d.hpp
        grid/AccessIterator.hpp
        grid/GridCell.hpp
        grid/GridCellAccessInterface.hpp
        grid/GridCellAccess.hpp
        grid/GridCellFacade.hpp
        geometric/Point.hpp
        geometric/LineSegment.hpp
        geometric/GeometricMap.hpp
        geometric/ContourMap.hpp
        tools/BresenhamLine.hpp
        operations/GridInterpolation.hpp
    DEPS_CMAKE
        Boost
    DEPS_PKGCONFIG 
        base-types 
        pcl_registration-1.7
    DEPS_PLAIN
        CGAL)
