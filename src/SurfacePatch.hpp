#ifndef __ENVIRE_MAPS_SURFACEPATCH_HPP__
#define __ENVIRE_MAPS_SURFACEPATCH_HPP__

#include <numeric/PlaneFitting.hpp>
#include <boost/intrusive/list.hpp>

#include <base/Eigen.hpp>
#include <Eigen/Eigenvalues>

#include "MLSConfig.hpp"

namespace envire
{
    namespace maps {

template <class T>
inline bool overlap( T a1, T a2, T b1, T b2 )
{
    return 
    ((a1 < b2) && (a2 > b2)) ||
    ((a1 < b1) && (a2 > b1));
}       

template <class T> inline T sq( T a ) { return a * a; }

template <class T> inline void kalman_update( T& mean, T& stdev, T m_mean, T m_stdev )
{
    const double var = sq( stdev );
    const double m_var = sq( m_stdev );
    double gain = var / (var + m_var);
    if( gain != gain )
        gain = 0.5; // this happens when both stdevs are 0.
    mean = mean + gain * (m_mean - mean);
    stdev = sqrt((1.0-gain)*var);
}

        /** The representation of one surface in a cell
         *
         * This data structure either represents a surface or a vertical block,
         * depending on the value of the \c horizontal field.
         *
         * If \c horizontal is true, a surface is represented as a mean altitude
         * and standard deviation around that mean.
         *
         * If \c horizontal is false, a vertical block is represented, in which
         * the mean value is the top of the block, and mean - height the bottom
         * of it. The standard deviation still applies.
         */

        typedef boost::intrusive::link_mode<boost::intrusive::auto_unlink> auto_unlink_mode;         
        
        class SurfacePatch : public boost::intrusive::list_base_hook<auto_unlink_mode> 
        {
        public:
            /** a surface patch can be three different types,
             * each changing how the cell values are interpreted
             *
             * HORIZONTAL - is a horizontal patch, which does not have a height value
             * VERTICAL - used to represent walls and other vertical structures, has a height
             * NEGATIVE - represent absence of structure, otherwise equal to VERTICAL
             */
            enum TYPE
            {
                VERTICAL = 0,
                HORIZONTAL = 1,
                NEGATIVE = 2
            };

        public:
            SurfacePatch( float mean, float stdev, float height = 0, TYPE type = HORIZONTAL );

            SurfacePatch( const Eigen::Vector3f &p, float stdev );

            void updateSum();

            void updatePlane();

            /**
             * Returns the height of the top of the surface patch
             * */
            float getTop() const;

            /**
             * Returns the height of the bottom of the surface patch
             * */
            float getBottom() const;
            
            /** Experimental code. Don't use it unless you know what you are
             * doing */
            float distance( const SurfacePatch& other ) const;

            /** Returns true if the mean of \c self is lower than the mean of \c
             * other
             */
            bool operator<( const SurfacePatch& other ) const
            {
                return mean < other.mean;
            };

            /** The minimum Z extent of this patch
             *
             * The uncertainty is handled by removing sigma_threshold * stdev to
             * the mean minimum Z
             */
            double getMinZ(double sigma_threshold = 2.0) const;

            /** The maximum Z extent of this patch
             *
             * The uncertainty is handled by adding sigma_threshold * stdev to
             * the mean maximum Z
             */
            double getMaxZ(double sigma_threshold = 2.0) const;

            void getRange(float& minZ, float& maxZ) const
            {
                minZ = min; maxZ = max;
            }

            /** flag the patch as horizontal
            */
            void setHorizontal()
            {
                type = HORIZONTAL;
            }

            /** flag the patch as vertical
            */
            void setVertical()
            {
                type = VERTICAL;
            }

            void setNegative()
            {
                type = NEGATIVE;
            }

            /** @return true if the patch is horizontal
            */
            bool isHorizontal() const
            {
                return type == HORIZONTAL;
            }

            /** @return true if patch is vertical
            */
            bool isVertical() const
            {
                return type == VERTICAL;
            }

            bool isNegative() const
            {
                return type == NEGATIVE;
            }

            base::Vector3d getColor() const
            {
                return base::Vector3d( color[0], color[1], color[2] ) / 255.0;
            }

            void setColor( const base::Vector3d& c )
            {
                color[0] = c[0] * 255;
                color[1] = c[1] * 255;
                color[2] = c[2] * 255;
            }

            float getHeight( const Eigen::Vector2f& pos ) const
            {
                // todo do some caching
                float zpos = Eigen::Vector3f( pos.x(), pos.y(), 1.0 ).dot( plane.getCoeffs() );
                return zpos;
            }

            double getSlope() const
            {
                return atan(plane.getCoeffs().head<2>().cast<double>().norm());
            }

            Eigen::Vector3f getCenter() const
            {
                Eigen::Vector3f center(plane.x, plane.y, plane.z);
                return center * (1.0f/plane.n);
            }
    
            Eigen::Vector3f getNormal() const
            {
                if(n<=1.0f) return Eigen::Vector3f::UnitZ();
                typedef base::PlaneFitting<float>::Matrix3 Mat3;
                Mat3 moments;
                moments << plane.xx, plane.xy, plane.xz,
                           plane.xy, plane.yy, plane.yz,
                           plane.xz, plane.yz, plane.zz;
                moments *= 1.0/plane.n;
                Eigen::Vector3f mu = getCenter();

                moments -= mu * mu.transpose();
                Eigen::SelfAdjointEigenSolver<Mat3> eig;
                eig.computeDirect(moments, Eigen::ComputeEigenvectors);
                return eig.eigenvectors().col(0);
            }

            Eigen::Vector3f getNormal_old() const
            {
                Eigen::Vector3f normal = -plane.getCoeffs();
                normal.z() = 1.0;
                return normal.normalized();
            }

            /**
             * @brief Accumulates 0th to 2nd powers of z values (weighted) which are used to calculate mean and variance
             * @deprecated this may not be supported in future versions
             */
            bool mergeSum( SurfacePatch& o, const MLSConfig& config );

            /**
             * @brief Accumulates moments of (x,y,z) to estimate surface normals
             */
            bool mergePlane( SurfacePatch& o, const MLSConfig& config );

            /**
             * @brief Classical mergeMLS implementation.
             * This distinguishes HORIZONTAL, VERTICAL and NEGATIVE SurfacePatches.
             */
            bool mergeMLS( SurfacePatch& o, const MLSConfig& config );

            bool merge( SurfacePatch& o,const MLSConfig& config );          

            /** 
             * @brief get the weighting for the patch
             * This value will depend on the uncertainty with which it was applied. It
             * is the sum of the inverse square of the standard deviation.
             */ 
            float getWeight() const
            {
                return plane.n;
            }

            /**
             * @brief scale the weighting without affecting the uncertainty calculation
             * @param factor scale factor to apply
             */
            void scaleWeight( float factor )
            {
                plane.scale( factor );
                normsq *= pow( factor, 2 );
            }

            float getMean() const
            {
                return mean;
            }

            float getStdev() const
            {
                return stdev;
            }

            float getHeight() const
            {
                return height;
            }

            /**
             * Returns the amount of measurements
             * which are represented by this SufacePatch
             * */
            float getMeasurementCount() const
            {
                return n;
            }
            
            bool isCovered( const SurfacePatch& o ) const
            {
                const SurfacePatch &p(*this);
                return (p.isNegative() && !o.isNegative()
                   && (p.mean-p.height) <= o.min && p.mean >= o.max
                   && p.update_idx > o.update_idx );
            }
        protected:
            /** The mean Z value. This always represents the top of the patch,
             * regardless whether the patch is horizontal or vertical
             */
            float mean;
            /** The standard deviation in Z */
            float stdev;
            /** For vertical patches, the height of the patch */
            float height;

            base::PlaneFitting<float> plane;
            float min, max;
            float n;
            float normsq;

            size_t update_idx;
            uint8_t color[3];

            /** Horizontal patches are just a mean and standard deviation.
             * Vertical patches also have a height, i.e. the patch is a vertical
             * block between z=(mean-height) and z
             */
            TYPE type;
        };
    }
}

#endif // __ENVIRE_MAPS_SURFACEPATCH_HPP__
