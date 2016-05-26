#pragma once

namespace maps { namespace tools
{
    inline bool overlap( double a_min, double a_max, double b_min, double b_max )
    {
        return 
            ((b_max >= a_max) && (b_min <= a_max)) ||
            ((b_max >= a_min) && (b_max <= a_max)) ;
    }
}    
}