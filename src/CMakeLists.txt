set(deps_pkg_config base-types)

rock_library(envire_maps
    HEADERS
        LocalMap.hpp
        GridMap.hpp
        ElevationMap.hpp
    DEPS_CMAKE Boost
    DEPS_PKGCONFIG ${deps_pkg_config})

install(FILES
    all
    DESTINATION include/envire_maps)
#target_link_libraries(envire_maps ${Boost_SYSTEM_LIBRARY})
