rock_testsuite(test_gridconfig suite.cpp
   test_GridConfig.cpp
   DEPS envire_maps)

rock_testsuite(test_gridbase suite.cpp
   test_GridBase.cpp
   DEPS envire_maps)

rock_testsuite(test_grid suite.cpp
   test_Grid.cpp
   DEPS envire_maps)   

rock_testsuite(test_gridmap suite.cpp
   test_GridMap.cpp
   DEPS envire_maps)      
 

 rock_testsuite(test_list suite.cpp
   test_List.cpp
   DEPS envire_maps)

rock_testsuite(test_surfacepatch suite.cpp
   test_SurfacePatch.cpp
   DEPS envire_maps)

 rock_testsuite(test_sp_list suite.cpp
   test_SPList.cpp
   DEPS envire_maps)
