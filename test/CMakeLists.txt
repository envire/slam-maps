rock_testsuite(test_gridconfig suite.cpp
   test_GridConfig.cpp
   DEPS envire_maps)

rock_testsuite(test_gridbase suite.cpp
   test_GridBase.cpp
   DEPS envire_maps)

rock_testsuite(test_grid suite.cpp
   test_Grid.cpp
   DEPS envire_maps)   
 
