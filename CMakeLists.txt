# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.
cmake_minimum_required(VERSION 3.1)
project(maps VERSION 0.1 DESCRIPTION "Standard Maps library")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(BUILD_SHARED_LIBS ON)
include(GNUInstallDirs)

if(COVERAGE)
    if(CMAKE_BUILD_TYPE MATCHES Debug)
        add_definitions(-fprofile-arcs -ftest-coverage)
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fprofile-arcs -ftest-coverage -lgcov")
        set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -fprofile-arcs -ftest-coverage -lgcov")
        set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} -fprofile-arcs -ftest-coverage -lgcov")

        add_custom_target(coverage
          COMMAND lcov --directory . --capture --output-file cov.info
          COMMAND lcov --remove cov.info 'tests/*' '/usr/*' '*/install/include/*' --output-file cov.info.cleaned
          COMMAND genhtml -o ./cov cov.info.cleaned
          COMMAND cmake -E remove cov.info cov.info.cleaned
          WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        )
    else()
        message(FATAL_ERROR "Code coverage only works in Debug versions" )
    endif()
endif()

if(CMAKE_BUILD_TYPE MATCHES Debug)
    add_definitions(-O1) # without any optimization, code is extreamly slow
endif()


add_subdirectory(src)

# Export the library interface
install(EXPORT ${PROJECT_NAME}-targets
        NAMESPACE ${PROJECT_NAME}::
	DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

# Create and install the version file
include(CMakePackageConfigHelpers)
write_basic_package_version_file("${PROJECT_NAME}-config-version.cmake"
	VERSION ${MAPS_VERSION}
        COMPATIBILITY SameMajorVersion
)

install(
        FILES
                ${PROJECT_NAME}-config.cmake
                ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
        DESTINATION
		${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

# Install package-manifest
#install(
#        FILES package.xml
#        DESTINATION ${CMAKE_INSTALL_DIR}/share/${PROJECT_NAME}
#)
