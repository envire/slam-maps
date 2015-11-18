set(deps_pkg_config base-types)

rock_library(envire_maps
    SOURCES
        ElevationMap.cpp
    HEADERS
        LocalMap.hpp
        GridMap.hpp
    DEPS_CMAKE Boost
    DEPS_PKGCONFIG ${deps_pkg_config})

rock_executable(main_bin main.cpp
    HEADERS
        LocalMap.hpp
        GridMap.hpp
    DEPS_CMAKE Boost
    DEPS_PKGCONFIG ${deps_pkg_config})

#install(FILES
#    all
#    DESTINATION include/envire_maps)

#target_link_libraries(envire_maps ${Boost_SYSTEM_LIBRARY})
