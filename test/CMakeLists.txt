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

#rock_testsuite(test_multilayergridmap suite.cpp
#   test_MultilayerGridMap.cpp
#   DEPS maps)


#rock_testsuite(test_list suite.cpp
#    test_List.cpp
#    DEPS maps)

#rock_testsuite(test_sp_list suite.cpp
#   test_SPList.cpp
#   DEPS maps)

rock_testsuite(test_generate_points suite.cpp
   test_generate_points.cpp
   DEPS maps)

rock_testsuite(test_serialization suite.cpp
   test_serialization.cpp
   DEPS maps)

rock_testsuite(test_geometricmap suite.cpp
   test_GeometricMap.cpp
   DEPS maps)

rock_testsuite(test_point suite.cpp
   test_Point.cpp
   DEPS maps)

rock_testsuite(test_linesegment suite.cpp
    test_LineSegment.cpp
   DEPS maps)

if( vizkit3d_FOUND )
    add_subdirectory(viz)
endif( vizkit3d_FOUND )


