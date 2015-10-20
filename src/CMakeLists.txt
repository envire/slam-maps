rock_library(envire_maps
    SOURCES 
    	GridBase.cpp            
    HEADERS 
        GridConfig.hpp
        GridBase.hpp    
    DEPS_PKGCONFIG base-types)

target_link_libraries(envire_maps
    ${Boost_SYSTEM_LIBRARY})       
