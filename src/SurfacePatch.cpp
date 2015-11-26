#include "SurfacePatch.hpp"

namespace envire 
{
namespace maps 
{

SurfacePatch::SurfacePatch( float mean, float stdev, float height, TYPE type)
    : mean(mean), stdev(stdev), height(height), 
    min(mean), max(mean),
    n(1.0),
    normsq(1.0/pow(stdev,4)),
    update_idx(0), 
    type(type) 
{
    plane.n = 1.0/pow(stdev,2);
    plane.z = mean * plane.n;
    plane.zz = pow(mean,2) * plane.n;
}   

SurfacePatch::SurfacePatch( const Eigen::Vector3f &p, float stdev )
    : mean(p.z()), stdev(stdev), height(0),
    plane( p, 1.0f/pow(stdev,2)),  
    min(p.z()), max(p.z()),
    n(1.0), 
    normsq(1.0/pow(stdev,4)),
    update_idx(0),
    type( HORIZONTAL )
{
    updatePlane();
}

void SurfacePatch::updateSum() 
{
    mean = plane.z / plane.n;
    float norm = plane.n / ( pow(plane.n,2) - normsq );
    if( n > 1 )
    {
        float var = 
            std::max(1e-6f, (float)((plane.zz - (pow(mean,2)*(plane.n - 2.0))) * norm - n/plane.n));
        stdev = sqrt(var);
    }
    else
        stdev = sqrt(1.0/plane.n);
}

void SurfacePatch::updatePlane()
{
    if( n <=3 )
    {
        updateSum();
        return;
    }
    base::PlaneFitting<float>::Result res = plane.solve();
    mean = res.getCoeffs()[2];
    float norm = plane.n / ( pow(plane.n,2) - 3.0*normsq );
    float var = std::max(1e-6f, (float)((res.getResiduals()) * norm));
    stdev = sqrt(var);
}

/** Experimental code. Don't use it unless you know what you are
 * doing */
float SurfacePatch::distance( const SurfacePatch& other ) const
{
    if( !isHorizontal() && !other.isHorizontal() )
        return 0;
    if( !isHorizontal() )
        return other.mean > mean ?
        other.mean - mean :
        std::max( 0.0f, mean - height - other.mean);
    if( !other.isHorizontal() )
        return mean > other.mean ?
        mean - other.mean :
        std::max( 0.0f, other.mean - other.height - mean);
    return std::abs( mean - other.mean );
}

double SurfacePatch::getMinZ(double sigma_threshold) const
{
    if (isHorizontal())
        return mean - stdev *  sigma_threshold;
    else
        return mean - height - stdev * sigma_threshold;
}

double SurfacePatch::getMaxZ(double sigma_threshold) const
{
    return mean + stdev * sigma_threshold;
}

bool SurfacePatch::mergeSum( SurfacePatch& o, const MLSConfig& config  )
{
    SurfacePatch &p(*this);

    if( overlap( min-config.gapSize, max+config.gapSize, o.min, o.max ) )
    {
        // use the weighted formulas for calculating 
        // mean and var of occupied space distribution
        // in the patch

        p.plane.z += o.plane.z;
        p.plane.zz += o.plane.zz;
        p.plane.n += o.plane.n;
        p.n += o.n;
        p.normsq += o.normsq;
        p.min = std::min( p.min, o.min );
        p.max = std::max( p.max, o.max );

        p.updateSum();    

        return true;
    }

    return false;
}

bool SurfacePatch::mergePlane( SurfacePatch& o, const MLSConfig& config )
{
    SurfacePatch &p(*this);

    if( overlap( min-config.gapSize, max+config.gapSize, o.min, o.max ) )
    {
        p.n += o.n;
        p.normsq += o.normsq;
        p.min = std::min( p.min, o.min );
        p.max = std::max( p.max, o.max );

        // sum the plane between the two
        p.plane.update( o.plane );
        p.updatePlane();
        
        return true;
    }
    
    return false;
}

bool SurfacePatch::mergeMLS( SurfacePatch& o, const MLSConfig& config )
{
    SurfacePatch &p(*this);
    const double delta_dev = sqrt( p.stdev * p.stdev + o.stdev * o.stdev );

    // see if the distance between the patches is small enough
    if( (p.mean - p.height - config.gapSize - delta_dev) < o.mean 
        && (p.mean + config.gapSize + delta_dev) > (o.mean - o.height) )
    {
        // if both patches are horizontal, we see if we can merge them
        if( p.isHorizontal() && o.isHorizontal() ) 
        {
        if( (p.mean - p.height - config.thickness - delta_dev) < o.mean && 
            (p.mean + config.thickness + delta_dev) > o.mean )
        {
            kalman_update( p.mean, p.stdev, o.mean, o.stdev );
        }
        else
        {
            // convert into a vertical patch element
            //p.mean += p.stdev;
            //p.height = 2 * p.stdev; 
            p.setVertical();
        }
        }

        // if either of the patches is vertical, the result is also going
        // to be vertical
        if( !p.isHorizontal() || !o.isHorizontal())
        {
        if( p.isHorizontal() && o.isVertical() )
            p.setVertical();
        else if( p.isNegative() && o.isNegative() )
            p.setNegative();
        else if( p.isNegative() || o.isNegative() )
        {
            // in this case (one negative one non negative)
            // its a bit hard to decide, since we have to remove
            // something somewhere to make it compatible
            //
            // best is to decide on age (based on update_idx) 
            // of the patch. Newer patches will be preferred

            if( p.update_idx == o.update_idx )
            return false;

            SurfacePatch &rp( p.update_idx < o.update_idx ? p : o );
            SurfacePatch &ro( p.update_idx < o.update_idx ? o : p );

            if( rp.update_idx < ro.update_idx )
            {
            // the new patch fully encloses the old one, 
            // so will overwrite it
            if( ro.mean > rp.mean && ro.mean - ro.height < rp.mean - rp.height )
            {
                p = ro;
                return true; 
            } 

            // the other patch is occupied, so cut
            // the free patch accordingly
            if( ro.mean < rp.mean )
                rp.height = rp.mean - ro.mean;
            else if( ro.mean - ro.height < rp.mean )
            {
                double new_mean = ro.mean - ro.height;
                rp.height -= rp.mean - new_mean;
                rp.mean = new_mean;
            }

            // both patches can live 
            return false;
            }
        }

        if( o.mean > p.mean )
        {
            p.height += ( o.mean - p.mean );
            p.mean = o.mean;
            p.stdev = o.stdev;
        }

        const double o_min = o.mean - o.height;
        const double p_min = p.mean - p.height;
        if( o_min < p_min )
        {
            p.height = p.mean - o_min;
        }
        }

        return true;
    }

    return false;
}

bool SurfacePatch::merge( SurfacePatch& o, const MLSConfig& config )
{
    bool merge = false;

    switch( config.updateModel )
    {
        case MLSConfig::KALMAN:
        merge = mergeMLS( o, config );
        break;

        case MLSConfig::SUM:
        merge = mergeSum( o, config );
        break;

        case MLSConfig::SLOPE:
        merge = mergePlane( o, config );
        break;

        default:
        throw std::runtime_error("MLS update model not implemented.");
    }

    if( merge )
    {
        update_idx = std::max( update_idx, o.update_idx );
        // update cell color
        setColor( (getColor() + o.getColor()) / 2.0 );

        return true;
    }

    return false;
}

}   
}
