#ifndef __MAPS_CONTOURMAP_HPP_
#define __MAPS_CONTOURMAP_HPP_

#pragma once

/** Maps classes **/
#include <maps/geometric/LineSegment.hpp>
#include <maps/geometric/GeometricMap.hpp>

namespace maps { namespace geometric
{
    typedef GeometricMap<LineSegment3d> ContourMap;
}}

#endif /* __MAPS_CONTOURMAP_HPP_ */
