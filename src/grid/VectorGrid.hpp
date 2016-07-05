#pragma once

#include <vector>
#include <stdexcept>

#include <boost/serialization/access.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/vector.hpp>

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
            for (unsigned int x = 0; x < num_cells.x(); ++x)
            {
                for (unsigned int y = 0; y < num_cells.y(); ++y)
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
            if(idx.x() >= num_cells.x() || idx.y() >= num_cells.y())
                throw std::runtime_error("Provided index is out of the grid");
            return cells[idx.x() + idx.y() * num_cells.x()];
        }

        CellT& at(const Index &idx)
        {
            if(idx.x() >= num_cells.x() || idx.y() >= num_cells.y())
                throw std::runtime_error("Provided index is out of the grid");
            return cells[idx.x() + idx.y() * num_cells.x()];
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

            std::pair<const_iterator, const_iterator> ranges = getRange();
            u_int32_t first_idx = std::distance(begin(), ranges.first);
            u_int32_t last_idx = std::distance(begin(), ranges.second);
            ar << first_idx;
            ar << last_idx;
            while(ranges.first != ranges.second)
            {
                ar << *ranges.first;
                ranges.first++;
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
    };
    
}}
