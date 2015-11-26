set(deps_pkg_config base-types)

rock_library(envire_maps
    SOURCES
        Grid.cpp
    HEADERS
        LocalMap.hpp
        Grid.hpp
    DEPS_CMAKE 
        Boost
    DEPS_PKGCONFIG 
        ${deps_pkg_config})
