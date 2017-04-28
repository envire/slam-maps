//
// Copyright (c) 2015-2017, Deutsches Forschungszentrum für Künstliche Intelligenz GmbH.
// Copyright (c) 2015-2017, University of Bremen
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
#pragma once

#include "LevelList.hpp"
#include "GridMap.hpp"
#include "../tools/Overlap.hpp"

namespace maps { namespace grid
{

    template <class P>
    class MultiLevelGridMap : public GridMap<LevelList<P> >
    {
    public:
        typedef LevelList<P> CellType; 
        
        typedef P PatchType;
        MultiLevelGridMap(const Vector2ui &num_cells,
                    const Eigen::Vector2d &resolution,
                    const boost::shared_ptr<LocalMapData> &data) : GridMap<LevelList<P> >(num_cells, resolution, LevelList<P>(), data)
        {}

        MultiLevelGridMap(const Vector2ui &num_cells,
                    const Eigen::Vector2d &resolution) : GridMap<LevelList<P> >(num_cells, resolution, LevelList<P>())
        {}
        
        MultiLevelGridMap() {}
        
        template<class Q>
        MultiLevelGridMap(const MultiLevelGridMap<Q> &other) : GridMap<CellType>(other, other)
        {
        }

        class View : public GridMap<LevelList<const P *> >
        {
        public:
            View(const Vector2ui &num_cells,
                const Eigen::Vector2d &resolution) : GridMap<LevelList<const P *> >(num_cells, resolution, LevelList<const P *>())
            {
            };
            
            View() : GridMap<LevelList<const P *> >()
            {
            };
        };
    
        View intersectCuboid(const Eigen::AlignedBox3d& box) const
        {
            size_t ignore;
            return intersectCuboid(box, ignore);
        }
        
        typedef std::vector<std::pair<Index, const P*>> PatchVector;

        /** Intersects @p box with the mls map.
         * The box is in local grid coordinates (i.e., starting at (0,0))
         * @return A list of patches and their grid indices that intersect the @p box.
         * @throw std::runtime_error if any part of @p box is outside the grid*/
        PatchVector intersectAABB(const Eigen::AlignedBox3d& box) const
        {
            PatchVector ret;
            intersectAABB_callback(box,
                    [&ret](Index idx, const P& p)
                    { 
                        ret.emplace_back(idx, &p);
                        return false;
                     });
            return ret;
        }

        // TODO compiling this can get rather expensive
        /** @param cb A callback that is executed for each patch that intersects.
         *            Prototype: bool f(const maps::grid::Index&, const P&)
         *            The return value of the the callback indicates whether
         *            the intersection test should abort or not.
         *            I.e. if the callback returns true, the intersection test
         *            will be aborted.*/
        template<class CallBack>
        void intersectAABB_callback(const Eigen::AlignedBox3d& box, CallBack&& cb) const
        {
            double minHeight = box.min().z();
            double maxHeight = box.max().z();

            Index minIdx = (box.min().head<2>().cwiseQuotient(this->getResolution())).template cast<int>();
            Index maxIdx = (box.max().head<2>().cwiseQuotient(this->getResolution())).template cast<int>();

            minIdx = minIdx.cwiseMax(0);
            maxIdx = maxIdx.cwiseMin(this->getNumCells().template cast<int>());

            for(int y = minIdx.y(); y < maxIdx.y(); y++)
            {
                for(int x = minIdx.x();x < maxIdx.x(); x++)
                {
                    const Index curIdx(x,y);
                    for(const P &p: this->at(curIdx))
                    {
                        if(::maps::tools::overlap(p.getMin(), p.getMax(), minHeight, maxHeight))
                        {
                            if(cb(curIdx, p))
                                return;
                        }
                    }
                }
            }
        }
        



        /** @param outNumIntersections contains the number of mls patches that
                                       intersected the @p box*/
        View intersectCuboid(const Eigen::AlignedBox3d& box, size_t& outNumIntersections) const
        {
            double minHeight = box.min().z();
            double maxHeight = box.max().z();
            outNumIntersections = 0;
            
            Index minIdx;
            Index maxIdx;
            
            if(!this->toGrid(box.min(), minIdx))
            {
                return View();
            }

            if(!this->toGrid(box.max(), maxIdx))
            {
                return View();
            }
            
            Index newSize(maxIdx-minIdx);
            
            View ret(Vector2ui(newSize.x(), newSize.y()) , this->getResolution());
            
            for(Index::Scalar y = minIdx.y(); y < maxIdx.y(); y++)
            {
                for(Index::Scalar x = minIdx.x();x < maxIdx.x(); x++)
                {
                    Index curIdx(x,y);
                    
                    LevelList<const P *> &retList(ret.at(Index(curIdx - minIdx)));
                    
                    for(const P &p: this->at(curIdx))
                    {
                        if(::maps::tools::overlap(p.getMin(), p.getMax(), minHeight, maxHeight))
                        {
                            retList.insert(&p);
                            ++outNumIntersections;
                        }
                    }
                }
            }
            
            return ret;
        }
        
    };

}}
