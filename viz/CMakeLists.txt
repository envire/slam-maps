pkg_check_modules(OSGVIZ_PRIMITIVES PrimitivesFactory)
pkg_check_modules(OSG REQUIRED openscenegraph)

if(${OSG_VERSION} VERSION_GREATER_EQUAL 3.6.4)
    add_compile_definitions(NEW_OSG)
endif()

if (vizkit3d_FOUND AND OSGVIZ_PRIMITIVES_FOUND)
    rock_find_qt4()
    rock_vizkit_plugin(maps-viz
            PluginLoader.cpp
            PatchesGeode.cpp
            SurfaceGeode.cpp
            StandaloneVisualizer.cpp
        MOC
            GridMapVisualization.cpp
            ElevationMapVisualization.cpp
            MLSMapVisualization.cpp
            TraversabilityMap3dVisualization.cpp
            ContourMapVisualization.cpp
            OccupancyGridMapVisualization.cpp
            TraversabilityGridVisualization.cpp
        HEADERS
            ColorGradient.hpp
            ExtentsRectangle.hpp
            MapVisualization.hpp
            GridMapVisualization.hpp
            ElevationMapVisualization.hpp
            MLSMapVisualization.hpp
            TraversabilityMap3dVisualization.hpp
            PatchesGeode.hpp
            SurfaceGeode.hpp
            StandaloneVisualizer.hpp
            ContourMapVisualization.hpp
            OccupancyGridMapVisualization.hpp
            TraversabilityGridVisualization.hpp
        DEPS 
            maps
        DEPS_PKGCONFIG 
	    base-logging
            base-viz vizkit3d  vizkit3d-viz
            PrimitivesFactory
    )
else()
    message(STATUS "osgviz not found ... NOT building the maps-viz plugins")
endif()

