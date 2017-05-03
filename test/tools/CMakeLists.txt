# TEST TOOLS
#
rock_testsuite(test_tools_generatepoints
   test_tools_GeneratePoints.cpp
   DEPS maps)

rock_testsuite(test_voxel_traversal
   test_voxel_traversal.cpp
   DEPS maps)

rock_testsuite(test_surface_intersection
   test_surface_intersection.cpp
   DEPS maps)
