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
        grid/LevelList.hpp        
        grid/LayeredGridMap.hpp
        grid/MultiLevelGridMap.hpp        
        grid/ElevationMap.hpp
        grid/SurfacePatches.hpp
        grid/MLSConfig.hpp
        grid/MLSMap.hpp
        grid/TraversabilityMap3d.hpp
        grid/AccessIterator.hpp
        grid/GridAccessInterface.hpp
        grid/GridFacade.hpp        
        grid/VectorGrid.hpp        
        grid/VectorGridAccess.hpp
        geometric/Point.hpp
        geometric/LineSegment.hpp
        geometric/GeometricMap.hpp
        geometric/ContourMap.hpp
        tools/BresenhamLine.hpp
        tools/Overlap.hpp
        operations/GridInterpolation.hpp
    DEPS_CMAKE
        Boost
        PCL
    DEPS_PKGCONFIG 
        base-types 
    DEPS_PLAIN
        CGAL)
