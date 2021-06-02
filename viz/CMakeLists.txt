pkg_check_modules(OSGVIZ_PRIMITIVES PrimitivesFactory)

if (vizkit3d_FOUND AND OSGVIZ_PRIMITIVES_FOUND)
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
        HEADERS
            ColorGradient.hpp
            ExtentsRectangle.hpp
            GridMapVisualization.hpp
            ElevationMapVisualization.hpp
            MLSMapVisualization.hpp
            TraversabilityMap3dVisualization.hpp
            PatchesGeode.hpp
            SurfaceGeode.hpp
            StandaloneVisualizer.hpp
            ContourMapVisualization.hpp
            OccupancyGridMapVisualization.hpp
        DEPS 
            maps
        DEPS_PKGCONFIG 
            base-viz vizkit3d  vizkit3d-viz
            PrimitivesFactory
    )
else()
    message(STATUS "osgviz not found ... NOT building the maps-viz plugins")
endif()

