# TEST GRID MAPS
#
rock_testsuite(test_index 
   test_Index.cpp
   DEPS maps)

rock_testsuite(test_vectorgrid
   test_VectorGrid.cpp
   DEPS maps)      

rock_testsuite(test_localmap
   test_LocalMap.cpp
   DEPS maps)

rock_testsuite(test_gridmap
   test_GridMap.cpp
   DEPS maps)

rock_testsuite(test_mlgrid
   test_MLGrid.cpp
   DEPS maps)

rock_testsuite(test_layeredgridmap
   test_LayeredGridMap.cpp
   DEPS maps)

rock_testsuite(test_traversabilitygrid
    test_TraversabilityGrid.cpp
    DEPS maps)
