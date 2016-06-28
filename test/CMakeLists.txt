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
if( vizkit3d_FOUND )
    add_subdirectory(viz)
endif( vizkit3d_FOUND )