rock_vizkit_plugin(envire_maps-viz
        ElevationMapVisualization.cpp
#        MLSGridVisualization.cpp
        PatchesGeode.cpp
        TraversabilityMap3dVisualization.cpp
#        StandaloneVisualizer.cpp
    DEPS 
        envire_maps
    DEPS_PKGCONFIG 
        base-viz vizkit3d  vizkit3d-viz
    MOC 
        ElevationMapVisualization.hpp
#        MLSGridVisualization.hpp
        TraversabilityMap3dVisualization.hpp
    HEADERS 
        ElevationMapVisualization.hpp
#        MLSGridVisualization.hpp
        TraversabilityMap3dVisualization.hpp
        PatchesGeode.hpp
        ColorGradient.hpp
#        StandaloneVisualizer.hpp
)
