rock_library(envire_maps
    SOURCES 
    	GridBase.cpp
    	ElevationGrid.cpp
        SurfacePatch.cpp              
    HEADERS 
        GridConfig.hpp
        GridBase.hpp    
        Grid.hpp
        GridMap.hpp
        ElevationGrid.hpp
        List.hpp
        ListGrid.hpp        
        MLSConfig.hpp
        SurfacePatch.hpp        
        SPList.hpp
        MLSGrid.hpp
    DEPS_PKGCONFIG base-types)

target_link_libraries(envire_maps
    ${Boost_SYSTEM_LIBRARY})       
