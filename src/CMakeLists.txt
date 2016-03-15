find_package(Boost REQUIRED COMPONENTS system filesystem serialization)

find_package(CGAL REQUIRED COMPONENTS Core)
include(${CGAL_USE_FILE})

rock_library(envire_maps
    SOURCES
        ElevationMap.cpp
        MLSGrid.cpp
        TraversabilityMap3d.cpp
        tools/BresenhamLine.cpp        
    HEADERS
        LocalMap.hpp
        Index.hpp
        GridMap.hpp
        MultilayerGridMap.hpp
        ElevationMap.hpp
        SurfacePatches.hpp
        List.hpp
        SPList.hpp
        MLSConfig.hpp
        MLSGrid.hpp
        MLSGridI.hpp
        TraversabilityMap3d.hpp
        operations/GridInterpolation.hpp
        storage/AccessIterator.hpp
        storage/GridStorage.hpp
        storage/GridStorageAccessInterface.hpp
        storage/GridStorageAccess.hpp
        storage/GridStorageFacade.hpp        
        tools/BresenhamLine.hpp
    DEPS_CMAKE
        Boost
    DEPS_PKGCONFIG 
        base-types 
        pcl_registration-1.7
    DEPS_PLAIN
        CGAL)
