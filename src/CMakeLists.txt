find_package(Boost REQUIRED COMPONENTS system filesystem serialization)
find_package(CGAL REQUIRED COMPONENTS Core)
find_package(PCL 1.7 REQUIRED COMPONENTS io)
find_package(PkgConfig REQUIRED)
pkg_check_modules(DEPS REQUIRED IMPORTED_TARGET base-types base-logging boost_serialization)

set(SOURCES
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
)

add_library(${PROJECT_NAME} SHARED ${SOURCES})
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    PkgConfig::DEPS
    Boost::system
    Boost::filesystem 
    Boost::serialization
    ${PCL_IO_LIBRARIES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${PCL_INCLUDE_DIRS}
)

target_compile_definitions(${PROJECT_NAME}
  PUBLIC
    NUMERIC_DEPRECATE=1
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
	FILES_MATCHING PATTERN "*.hpp"
)

install (TARGETS
	${PROJECT_NAME}
	EXPORT ${PROJECT_NAME}-targets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

set(PKG_CONFIG_DEPS
  base-types
  base-logging
  boost_serialization
  pcl_io-${PCL_VERSION_MAJOR}.${PCL_VERSION_MINOR}
)

set(pkg_config_dependencies "")

foreach(value ${PKG_CONFIG_DEPS})
    set(pkg_config_dependencies "${pkg_config_dependencies} ${value}")
endforeach()

set(PKGCONFIG_REQUIRES ${pkg_config_dependencies})
set(PKGCONFIG_CFLAGS)

configure_file(${PROJECT_NAME}.pc.in ${PROJECT_NAME}.pc @ONLY)
install(FILES
	${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.pc
	DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig
)
