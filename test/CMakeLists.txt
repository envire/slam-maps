# TEST GRID MAPS
#
rock_testsuite(test_index suite.cpp
   test_Index.cpp
   DEPS maps)

rock_testsuite(test_localmap suite.cpp
   test_LocalMap.cpp
   DEPS maps)

rock_testsuite(test_gridmap suite.cpp
   test_GridMap.cpp
   DEPS maps)

rock_testsuite(test_mlGrid suite.cpp
   test_MLGrid.cpp
   DEPS maps)

rock_testsuite(test_layeredgridmap suite.cpp
   test_LayeredGridMap.cpp
   DEPS maps)


#rock_testsuite(test_sp_list suite.cpp
#   test_SPList.cpp
#   DEPS maps)


# TEST GEOMETRIC MAPS
#
rock_testsuite(test_geometricmap suite.cpp
   test_GeometricMap.cpp
   DEPS maps)

rock_testsuite(test_point suite.cpp
   test_Point.cpp
   DEPS maps)

rock_testsuite(test_linesegment suite.cpp
    test_LineSegment.cpp
   DEPS maps)

rock_testsuite(test_contourmap suite.cpp
   test_ContourMap.cpp
   DEPS maps)

# TEST SERIALIZATION
#
rock_testsuite(test_localmap_serialization suite.cpp
   test_localmap_serialization.cpp
   DEPS maps)

rock_testsuite(test_grid_serialization suite.cpp
   test_grid_serialization.cpp
   DEPS maps)

rock_testsuite(test_geometric_serialization suite.cpp
   test_geometric_serialization.cpp
   DEPS maps)

rock_testsuite(test_mls_serialization suite.cpp
   test_mls_serialization.cpp
   DEPS maps)   

# TEST TOOLS
#
rock_testsuite(test_generate_points suite.cpp
   test_generate_points.cpp
   DEPS maps)

if( vizkit3d_FOUND )
    add_subdirectory(viz)
endif( vizkit3d_FOUND )


