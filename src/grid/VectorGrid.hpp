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

#include <vector>
#include <stdexcept>

#include <boost/serialization/access.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/vector.hpp>
#include <boost_serialization/ClassVersion.hpp>
#include <boost_serialization/DynamicSizeSerialization.hpp>

#include <maps/grid/Index.hpp>

namespace maps { namespace grid
{

    template <typename CellT>
    class VectorGrid
    {
        /** The cells grid element **/
        std::vector<CellT> cells;

        /** Number of cells in X-axis and Y-axis **/
        Vector2ui num_cells;

        /** Default value **/
        CellT default_value;

    public:

        typedef CellT CellType;
        typedef typename std::vector<CellT>::iterator iterator;
        typedef typename std::vector<CellT>::const_iterator const_iterator;

        VectorGrid(Vector2ui size, CellT default_value) 
            : num_cells(size), 
              default_value(default_value)
        {
            resize(size);
        }

        VectorGrid(Vector2ui size) 
            : VectorGrid(size, CellT())
        {
        }

        VectorGrid() 
            : VectorGrid(Vector2ui(0,0), CellT())
        {
        }

        VectorGrid(const VectorGrid &other) 
            : cells(other.cells), 
              num_cells(other.num_cells), 
              default_value(other.default_value)
        {
        }

        template<class CellT2>
        VectorGrid(const VectorGrid<CellT2>& other)
            : cells(other.begin(), other.end())
            , num_cells(other.getNumCells())
            , default_value(other.getDefaultValue())
        {
        }

        const CellT &getDefaultValue() const
        {
            return default_value;
        }

        iterator begin()
        {
            return cells.begin();
        }

        iterator end()
        {
            return cells.end();
        }

        const_iterator begin() const
        {
            return cells.begin();
        }

        const_iterator end() const
        {
            return cells.end();
        }

        void resize(const Vector2ui &new_number_cells)
        {
            this->num_cells = new_number_cells;
            cells.resize(new_number_cells.prod(), default_value);
        };

        /**
         * @brief Move the content of the grid cells
         * @details by the offset described in the argument
         * @return void
         */
        void moveBy(const Index &idx)
        {
            const Vector2ui num_cells(this->num_cells);

            // if all grid values should be moved outside
            if (abs(idx.x()) >= num_cells.x()
                || abs(idx.y()) >= num_cells.y())
            {
                clear();
                return;
            }

            std::vector<CellT> tmp;
            tmp.resize(num_cells.prod(), default_value);

            //copy pointers to new grid at new position
            for (unsigned int y = 0; y < num_cells.y(); ++y)
            {
                for (unsigned int x = 0; x < num_cells.x(); ++x)
                {
                    int x_new = x + idx.x();
                    int y_new = y + idx.y();

                    if ((x_new >= 0 && x_new < num_cells.x())
                        && (y_new >= 0 && y_new < num_cells.y()))
                    {
                        std::swap(cells[toIdx(x, y)], tmp[toIdx(x_new, y_new)]);
                    }
                }
            }

            cells.swap(tmp);
        }

        const CellT& at(const Index &idx) const
        {
            return this->at(idx.x(), idx.y());
        }

        CellT& at(const Index &idx)
        {
            return this->at(idx.x(), idx.y());
        }

        const CellT& at(size_t x, size_t y) const
        {
            if(x >= num_cells.x() || y >= num_cells.y())
                throw std::runtime_error("Provided index is out of the grid");
            return cells[x + y * num_cells.x()];
        }

        CellT& at(size_t x, size_t y)
        {
            if(x >= num_cells.x() || y >= num_cells.y())
                throw std::runtime_error("Provided index is out of the grid");
            return cells[x + y * num_cells.x()];
        }
        
        const Vector2ui &getNumCells() const
        {
            return num_cells;
        };
        
        void clear()
        {
            for(CellT &e : cells)
            {
                e = default_value;
            }
        };      
        
    protected:
        size_t toIdx(size_t x, size_t y) const
        {
            return x  +  y * num_cells.x();
        }

        /**
         * Returns iterators to the first and one element after the last element
         * containing values unequal to the defaut value.
         */
        std::pair<const_iterator, const_iterator> getRange() const
        {
            const_iterator start_range = std::find_if(cells.begin(), cells.end(), 
                std::bind1st(std::not_equal_to<CellT>(), default_value));

            // cells are all default
            if(start_range == cells.end())
                return std::make_pair(start_range, cells.end());

            typename std::vector<CellT>::const_reverse_iterator r_end_range = 
                std::find_if(cells.crbegin(), cells.crend(),
                    std::bind1st(std::not_equal_to<CellT>(), default_value));

            const_iterator end_range(r_end_range.base());
            return std::pair<const_iterator, const_iterator>(start_range, end_range);
        }

        /** Grants access to boost serialization */
        friend class boost::serialization::access;

        BOOST_SERIALIZATION_SPLIT_MEMBER()

        /** serialize members */
        template<class Archive>
        void save(Archive & ar, const unsigned int version) const
        {
            ar << BOOST_SERIALIZATION_NVP(num_cells.derived());
            ar << BOOST_SERIALIZATION_NVP(default_value);

            const_iterator block_start_cell = cells.begin();
            const_iterator block_end_cell;
            uint64_t block_size;
            bool block_occupied;
            while (block_start_cell != cells.end())
            {
                // identify the next block of occupied or non-occupied cells
                nextBlock(block_start_cell, block_size, block_occupied, block_end_cell);

                // save bock header
                ar << block_occupied;
                saveSizeValue(ar, block_size);

                // set start cell equal to end cell if the block is not occupied
                if (!block_occupied)
                    block_start_cell = block_end_cell;

                // write cells
                while (block_start_cell != block_end_cell)
                {
                    ar << *block_start_cell;
                    block_start_cell++;
                }
            }
        }

        /** deserialize members */
        template<class Archive>
        void load(Archive & ar, const unsigned int version)
        {
            ar >> BOOST_SERIALIZATION_NVP(num_cells.derived());
            ar >> BOOST_SERIALIZATION_NVP(default_value);
            cells.clear();
            cells.resize(num_cells.x() * num_cells.y(), default_value);

            if (version == 0)
            {
                // deserialization of version 0 of VectorGrid
                u_int32_t first_idx;
                u_int32_t last_idx;
                ar >> first_idx;
                ar >> last_idx;
                while(first_idx != last_idx)
                {
                    ar >> cells[first_idx];
                    first_idx++;
                }
            }
            else
            {
                // return of cells are empty
                if (cells.empty())
                    return;

                bool block_occupied;
                uint64_t block_size;
                size_t current_cell = 0;
                size_t block_end;
                while (current_cell < cells.size())
                {
                    // receive block header
                    ar >> block_occupied;
                    loadSizeValue(ar, block_size);
                    block_end = current_cell + block_size;

                    // skip, if cells of this block are not occupied
                    if (!block_occupied)
                        current_cell = block_end;

                    // read cells
                    while (current_cell < block_end)
                    {
                        ar >> cells[current_cell];
                        current_cell++;
                    }
                }
            }
        }

    private:

        /**
         * Identifies the next occupied or non-occupied block from a given start cell.
         * If the start cell is equal to the default cell value the block is considered
         * non-occupied and vice versa.
         */
        void nextBlock(const_iterator start_cell, uint64_t &block_size, bool &block_occupied, const_iterator &end_cell) const
        {
            // check if the start cell is occupied
            block_occupied = *start_cell != default_value;
            // find next either occupied or non-occupied cell
            if (block_occupied)
                end_cell = std::find_if_not(start_cell+1, cells.end(),
                                                     std::bind1st(std::not_equal_to<CellT>(), default_value));
            else
                end_cell = std::find_if_not(start_cell+1, cells.end(),
                                                     std::bind1st(std::equal_to<CellT>(), default_value));
            // compute block size
            block_size = std::distance(start_cell, end_cell);
        }
    };
}}

BOOST_TEMPLATED_CLASS_VERSION(maps::grid::VectorGrid, 1)
