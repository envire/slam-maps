rock_find_qt5(Core)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# TEST GRID MAPS
#
add_subdirectory(grid)

# TEST GEOMETRIC MAPS
#
add_subdirectory(geometric)

# TEST SERIALIZATION
#
add_subdirectory(serialization)

# TEST TOOLS
#
add_subdirectory(tools)

# TEST VISUALIZATION
#
if( vizkit3d_FOUND AND OSGVIZ_PRIMITIVES_FOUND)
    add_subdirectory(viz)
endif()
