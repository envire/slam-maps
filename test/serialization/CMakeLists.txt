# TEST SERIALIZATION
#
rock_testsuite(test_serialization_localmap
   test_serialization_LocalMap.cpp
   DEPS maps)

rock_testsuite(test_serialization_grid
   test_serialization_Grid.cpp
   DEPS maps)

rock_testsuite(test_serialization_geometric
   test_serialization_Geometric.cpp
   DEPS maps)

rock_testsuite(test_serialization_mlsmap
   test_serialization_MLS.cpp
   DEPS maps)   