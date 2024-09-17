find_package(Boost REQUIRED COMPONENTS system filesystem serialization)
find_package(CGAL REQUIRED COMPONENTS Core)
find_package(PCL 1.7 REQUIRED COMPONENTS common)
IF("${PCL_VERSION_MAJOR}.${PCL_VERSION_MINOR}" VERSION_LESS 1.14)
    SET(PCL_VERSION_SUFFIX "-${PCL_VERSION_MAJOR}.${PCL_VERSION_MINOR}")
ENDIF()

rock_library(maps
    SOURCES
        grid/ElevationMap.cpp
        grid/TraversabilityMap3d.cpp
        grid/OccupancyGridMap.cpp
        grid/TraversabilityCell.cpp
        grid/TraversabilityClass.cpp
        grid/TraversabilityGrid.cpp
        grid/TSDFVolumetricMap.cpp
        tools/BresenhamLine.cpp
        tools/VoxelTraversal.cpp
        tools/TSDFPolygonMeshReconstruction.cpp
        tools/TSDF_MLSMapReconstruction.cpp
        operations/CoverageMapGeneration.cpp
        tools/MLSToSlopes.cpp
        tools/SimpleTraversability.cpp
        tools/SimpleTraversabilityRadialLUT.cpp
        tools/TraversabilityGrassfire.cpp
    HEADERS
        LocalMap.hpp
        grid/Index.hpp
        grid/GridMap.hpp
        grid/LevelList.hpp        
        grid/LayeredGridMap.hpp
        grid/MultiLevelGridMap.hpp        
        grid/ElevationMap.hpp
        grid/SurfacePatches.hpp
        grid/MLSConfig.hpp
        grid/MLSMap.hpp
        grid/TraversabilityMap3d.hpp
        grid/AccessIterator.hpp
        grid/GridAccessInterface.hpp
        grid/GridFacade.hpp        
        grid/VectorGrid.hpp        
        grid/VectorGridAccess.hpp
        grid/DiscreteTree.hpp
        grid/VoxelGridMap.hpp
        grid/OccupancyGridMapBase.hpp
        grid/OccupancyGridMap.hpp
        grid/OccupancyConfiguration.hpp
        grid/OccupancyPatch.hpp
        grid/TraversabilityCell.hpp
        grid/TraversabilityClass.hpp
        grid/TraversabilityGrid.hpp
        grid/TSDFPatch.hpp
        grid/TSDFVolumetricMap.hpp
        geometric/Point.hpp
        geometric/LineSegment.hpp
        geometric/GeometricMap.hpp
        geometric/ContourMap.hpp
        tools/BresenhamLine.hpp
        tools/Overlap.hpp
        tools/VoxelTraversal.hpp
        tools/TSDFSurfaceReconstruction.hpp
        tools/TSDFPolygonMeshReconstruction.hpp
        tools/TSDF_MLSMapReconstruction.hpp
        tools/MarchingCubes.hpp
        tools/SurfaceIntersection.hpp
        tools/MLSToSlopes.hpp
        tools/SimpleTraversability.hpp
        tools/SimpleTraversabilityRadialLUT.hpp
        tools/TraversabilityGrassfire.hpp
        tools/TraversabilityGrassfireConfig.hpp
        tools/TraversabilityGrassFireSearchItem.hpp
        operations/GridInterpolation.hpp
        operations/CoverageMapGeneration.hpp
    DEPS_PKGCONFIG 
        base-types 
        base-logging
        boost_serialization
        #pcl_io is used by the orogen task, which cannot tell what version
        #to look for.
        pcl_io${PCL_VERSION_SUFFIX}
        pcl_common${PCL_VERSION_SUFFIX}
    DEPS Boost::system
         Boost::filesystem
         Boost::serialization
)

