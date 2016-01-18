set(deps_pkg_config base-types)

rock_library(envire_maps
    SOURCES
        Grid.cpp
        SurfacePatch.cpp
        MLSGrid.cpp
        tools/BresenhamLine.cpp
    HEADERS
        LocalMap.hpp
        Grid.hpp
        GridMap.hpp
        MultilayerGridMap.hpp
        SurfacePatch.hpp
        List.hpp
        SPList.hpp
        MLSConfig.hpp
        MLSGrid.hpp
        tools/BresenhamLine.hpp
    DEPS_CMAKE 
        Boost
        PCL
    DEPS_PKGCONFIG 
        ${deps_pkg_config})
