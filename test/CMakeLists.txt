rock_testsuite(test_localmap suite.cpp
   test_LocalMap.cpp
   DEPS envire_maps)   

rock_testsuite(test_grid suite.cpp
   test_Grid.cpp
   DEPS envire_maps)

rock_testsuite(test_gridmap suite.cpp
   test_GridMap.cpp
   DEPS envire_maps)   

rock_testsuite(test_multilayergridmap suite.cpp
   test_MultilayerGridMap.cpp
   DEPS envire_maps)     

rock_testsuite(test_surfacepatch suite.cpp
   test_SurfacePatch.cpp
   DEPS envire_maps)

rock_testsuite(test_list suite.cpp
    test_List.cpp
    DEPS envire_maps)   

rock_testsuite(test_sp_list suite.cpp
   test_SPList.cpp
   DEPS envire_maps)

rock_testsuite(test_generate_points suite.cpp
   test_generate_points.cpp
   DEPS envire_maps)

if( vizkit3d_FOUND )
    add_subdirectory(viz)
endif( vizkit3d_FOUND )


