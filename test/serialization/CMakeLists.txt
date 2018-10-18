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
   
rock_testsuite(test_serialization_travmap
   test_serialization_TravMap.cpp
   DEPS maps)

rock_testsuite(test_serialization_discreteTree
    test_serialization_DiscreteTree.cpp
    DEPS maps)

# Testfiles.
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/DiscreteTree_v0.bin
               ${CMAKE_CURRENT_BINARY_DIR}/DiscreteTree_v0.bin COPYONLY)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/TravMap_v00.bin
               ${CMAKE_CURRENT_BINARY_DIR}/TravMap_v00.bin COPYONLY)
