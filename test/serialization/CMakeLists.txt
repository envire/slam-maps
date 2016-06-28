# TEST SERIALIZATION
#
rock_testsuite(test_localmap_serialization
   test_localmap_serialization.cpp
   DEPS maps)

rock_testsuite(test_grid_serialization
   test_grid_serialization.cpp
   DEPS maps)

rock_testsuite(test_geometric_serialization
   test_geometric_serialization.cpp
   DEPS maps)

rock_testsuite(test_mls_serialization
   test_mls_serialization.cpp
   DEPS maps)   