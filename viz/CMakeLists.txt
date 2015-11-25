rock_vizkit_plugin(envire_maps-viz
        MLSGridVisualization.cpp
        PatchesGeode.cpp
    DEPS 
    	envire_maps
    MOC 
        MLSGridVisualization.hpp
    HEADERS 
        MLSGridVisualization.hpp
        PatchesGeode.hpp
)

rock_executable(envire_viewer EnvireViewer.cpp
    DEPS envire_maps-viz
    DEPS_PKGCONFIG vizkit3d vizkit3d-viz)
target_link_libraries(envire_viewer
    ${QT_QTGUI_LIBRARY} ${Boost_SYSTEM_LIBRARY})    

