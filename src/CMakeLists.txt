set(deps_pkg_config base-types)

rock_library(envire_maps
    SOURCES
        Grid.cpp
        SurfacePatch.cpp
    HEADERS
        LocalMap.hpp
        Grid.hpp
        GridMap.hpp
        MultilayerGridMap.hpp
        SurfacePatch.hpp
        List.hpp
        SPList.hpp
        MLSGrid.hpp
    DEPS_CMAKE 
        Boost
    DEPS_PKGCONFIG 
        ${deps_pkg_config})
