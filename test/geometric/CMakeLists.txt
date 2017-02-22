# TEST GEOMETRIC MAPS
#
rock_testsuite(test_geometricmap
   test_GeometricMap.cpp
   DEPS maps)

rock_testsuite(test_point
   test_Point.cpp
   DEPS maps)

rock_testsuite(test_linesegment
    test_LineSegment.cpp
   DEPS maps)

rock_testsuite(test_contourmap
   test_ContourMap.cpp
   DEPS maps)

rock_testsuite(test_closest_MLS
   test_closest_MLS.cpp
   DEPS maps)
