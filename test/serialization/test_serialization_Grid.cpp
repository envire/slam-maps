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
#define BOOST_TEST_MODULE SerializationTest
#include <boost/test/unit_test.hpp>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

/** Grid maps **/
#include <maps/grid/GridMap.hpp>
#include <maps/grid/LevelList.hpp>
#include <maps/grid/MultiLevelGridMap.hpp>
#include <maps/grid/TraversabilityGrid.hpp>

using namespace ::maps::grid;

class A {
public:
    A()
        : min(0), max(0) {};

    A(double min, double max) 
        : min(min), max(max) {};

    bool operator==(const A &other) const
    {
        if (min == other.min && max == other.max)
            return true;
        else
            return false;
    }

    bool operator!=(const A &other) const
    {
        return !(*this == other);
    }

    void test()
    {
        std::cout << min << " " << max << std::endl;
    }

protected:
    double min;
    double max;

    /** Grants access to boost serialization */
    friend class boost::serialization::access;  

    /** Serializes the members of this class*/
    template <typename Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        ar & BOOST_SERIALIZATION_NVP(min);
        ar & BOOST_SERIALIZATION_NVP(max);
    }
};

BOOST_AUTO_TEST_CASE(test_grid_serialization)
{
    A default_value(-5.5, 3);
    Vector2ui storage_size(2, 3);
    VectorGrid<A> storage_o(storage_size, default_value);

    // a dv dv
    // dv a dv
    storage_o.at(0, 0) = A(0, 0);
    storage_o.at(1, 1) = A(1, 1);

    std::stringstream stream;
    boost::archive::binary_oarchive oa(stream);
    oa << storage_o;

    // deserialize from string stream
    boost::archive::binary_iarchive *ia = new boost::archive::binary_iarchive(stream);
    VectorGrid<A> storage_i;
    (*ia) >> storage_i; 

    BOOST_CHECK(storage_i.getDefaultValue() == storage_o.getDefaultValue()); 
    BOOST_CHECK(storage_i.getDefaultValue() == default_value);
    BOOST_CHECK(storage_i.getNumCells() == storage_o.getNumCells());
    BOOST_CHECK(storage_i.getNumCells() == storage_size);

    for (unsigned int x = 0; x < storage_size.x(); ++x)
    {
        for (unsigned int y = 0; y < storage_size.y(); ++y)
        {
            if (x == 0 && y == 0)
            {
                BOOST_CHECK(storage_i.at(x, y) == A(0, 0));
            } else if (x == 1 && y == 1)
            {
                BOOST_CHECK(storage_i.at(x, y) == A(1, 1));
            } else 
            {
                BOOST_CHECK(storage_i.at(x, y) == default_value);
            }
        }
    }
}

BOOST_AUTO_TEST_CASE(test_gridmap_serialization)
{
    A default_value(-5.5, 3);
    Vector2ui storage_size(2, 3);
    Vector2d resolution(0.1, 0.5);
    GridMap<A> grid_map_o(storage_size, resolution, default_value);

    // a dv dv
    // dv a dv
    grid_map_o.at(0, 0) = A(0, 0);
    grid_map_o.at(1, 1) = A(1, 1);

    std::stringstream stream;
    boost::archive::binary_oarchive oa(stream);
    oa << grid_map_o;

    // deserialize from string stream
    boost::archive::binary_iarchive *ia = new boost::archive::binary_iarchive(stream);
    GridMap<A> grid_map_i;
    (*ia) >> grid_map_i; 

    BOOST_CHECK(grid_map_i.getDefaultValue() == grid_map_o.getDefaultValue()); 
    BOOST_CHECK(grid_map_i.getDefaultValue() == default_value);
    BOOST_CHECK(grid_map_i.getResolution() == grid_map_o.getResolution()); 
    BOOST_CHECK(grid_map_i.getResolution() == resolution);    
    BOOST_CHECK(grid_map_i.getNumCells() == grid_map_o.getNumCells());
    BOOST_CHECK(grid_map_i.getNumCells() == storage_size);

    for (unsigned int x = 0; x < storage_size.x(); ++x)
    {
        for (unsigned int y = 0; y < storage_size.y(); ++y)
        {
            if (x == 0 && y == 0)
            {
                BOOST_CHECK(grid_map_i.at(x, y) == A(0, 0));
            } else if (x == 1 && y == 1)
            {
                BOOST_CHECK(grid_map_i.at(x, y) == A(1, 1));
            } else 
            {
                BOOST_CHECK(grid_map_i.at(x, y) == default_value);
            }
        }
    }
}

BOOST_AUTO_TEST_CASE(test_levellist_serialization)
{
    LevelList<int> list;
    list.insert(42);
    list.insert(-1337);
    std::stringstream stream;
    boost::archive::binary_oarchive oa(stream);
    oa << list;

    // deserialize from string stream
    boost::archive::binary_iarchive ia(stream);
    LevelList<int> list_out;
    ia >> list_out;

    BOOST_CHECK_EQUAL(list_out.size(), 2);
    BOOST_CHECK(list_out.find(42) != list_out.end());
    BOOST_CHECK(list_out.find(-1337) != list_out.end());
}


BOOST_AUTO_TEST_CASE(test_mlgrid_serialization)
{
    Vector2ui size(100, 200);
    Vector2d res(0.25, 0.125);
    MultiLevelGridMap<int> grid(size, res), grid2;

    Eigen::Vector2d gridSize = grid.getSize();

    for(int i=0; i<100000; ++i)
    {
        Eigen::Vector3d v = Eigen::Vector3d::Random().cwiseAbs();
        v.head<2>().array()*=gridSize.array();
        grid.at(v).insert(int(rand()));
    }

    std::stringstream stream;
    boost::archive::binary_oarchive oa(stream);
    std::cout << stream.str().size() << std::endl;
    oa << grid;
    std::cout << stream.str().size() << std::endl;
    // deserialize from string stream
    boost::archive::binary_iarchive ia(stream);
    ia >> grid2;


    for(size_t x=0; x<size.x(); ++x)
        for(size_t y=0; y<size.y(); ++y)
        {
            Index idx(x,y);
            typedef MultiLevelGridMap<int>::CellType Cell;
            const Cell &cell1 = grid.at(idx), &cell2=grid2.at(idx);
            BOOST_CHECK_EQUAL(cell1.size(), cell2.size());
            Cell::const_iterator it1=cell1.begin(), it2=cell2.begin(), end1=cell1.end(), end2 = cell2.end();
            for(; it1 != end1; ++it1, ++it2)
            {
                BOOST_CHECK(it2 != end2);
                BOOST_CHECK_EQUAL(*it1, *it2);
            }
            BOOST_CHECK(it2 == end2);

        }
}

BOOST_AUTO_TEST_CASE(test_traversabilityGrid_serialization)
{
    Vector2ui size(4, 4);
    Vector2d resolution(0.2, 0.6);
    TraversabilityCell default_value = TraversabilityCell();
    
    TraversabilityGrid traversabilityGrid_out = TraversabilityGrid(size, resolution, default_value);
    
    for(int i = 0; i < 4; i++)
    {
	uint8_t returnId;
	TraversabilityClass travClass = TraversabilityClass(0.1 * i);
	BOOST_CHECK(traversabilityGrid_out.registerNewTraversabilityClass(returnId, travClass));
    }
    
    for(unsigned i = 0; i < size.x(); i++)
    {
	for(unsigned j = 0; j < size.y(); j++)
	{
	    traversabilityGrid_out.setTraversabilityAndProbability(i % 4, 0.1 * j, i, j);
	}
    }
    
    std::stringstream stream;
    boost::archive::binary_oarchive outarchive(stream);
    outarchive << traversabilityGrid_out;
    
    boost::archive::binary_iarchive inarchchive(stream);
    TraversabilityGrid traversabilityGrid_in = TraversabilityGrid();
    inarchchive >> traversabilityGrid_in;
    
    BOOST_CHECK_EQUAL(traversabilityGrid_out.getSize(), traversabilityGrid_in.getSize());
    
    std::vector<TraversabilityClass> classes_out = traversabilityGrid_out.getTraversabilityClasses();
    std::vector<TraversabilityClass> classes_in = traversabilityGrid_in.getTraversabilityClasses();
    
    BOOST_CHECK_EQUAL(classes_out.size(), classes_in.size());
        
    for(unsigned i = 0; i < 4; i++)
    {    
	BOOST_CHECK_EQUAL(classes_out[i].getDrivability(), classes_in[i].getDrivability());
    }
    
    for(unsigned i = 0; i < size.x(); i++)
    {
	for(unsigned j = 0; j < size.y(); j++)
	{
	    BOOST_CHECK_EQUAL(traversabilityGrid_in.getTraversability(i, j).getDrivability(), traversabilityGrid_out.getTraversability(i, j).getDrivability());
	    BOOST_CHECK_EQUAL(traversabilityGrid_in.getProbability(i, j), traversabilityGrid_out.getProbability(i, j)); 
	}
    }
}

/*BOOST_AUTO_TEST_CASE(test_grid_serialization)
{
    Grid grid_o(Vector2ui(100, 100), Vector2d(0.153, 0.257));

    std::stringstream stream;
    boost::archive::polymorphic_binary_oarchive oa(stream);
    oa << grid_o;

    // deserialize from string stream
    boost::archive::polymorphic_binary_iarchive ia(stream);
    Grid grid_i;
    ia >> grid_i;

    BOOST_CHECK_EQUAL(grid_o.getNumCells(), grid_i.getNumCells());
    BOOST_CHECK_EQUAL(grid_o.getResolution(), grid_i.getResolution());
    BOOST_CHECK_EQUAL(grid_o.getSize().isApprox(grid_i.getSize(), 0.0001), true);

    boost::shared_ptr<LocalMapData> local_map_data_i = grid_i.getLocalMap();
    boost::shared_ptr<LocalMapData> local_map_data_o = grid_o.getLocalMap();

    BOOST_CHECK(local_map_data_o->id == local_map_data_i->id);
    BOOST_CHECK(local_map_data_o->map_type == local_map_data_i->map_type);
    BOOST_CHECK(local_map_data_o->EPSG_code == local_map_data_i->EPSG_code);
    BOOST_CHECK_EQUAL(local_map_data_o->offset.matrix().isApprox(local_map_data_i->offset.matrix()), true);  
}

BOOST_AUTO_TEST_CASE(test_gridmap_serialization)
{
    Vector2ui num_cells(100, 200);
    Vector2d resolution(0.1, 0.5);

    GridMap<double> grid_o(num_cells, resolution, default_value); 
    grid_o.getOffset().translate(Eigen::Vector3d::Random(3));

    for (unsigned int x = 0; x < grid_o.getNumCells().x(); ++x)
    {
        for (unsigned int y = 0; y < grid_o.getNumCells().y(); ++y)
        {
            double cell_value = rand();
            grid_o.at(x, y) = cell_value;
        }
    } 

    std::stringstream stream;
    boost::archive::polymorphic_binary_oarchive oa(stream);
    oa << grid_o;

    // deserialize from string stream
    boost::archive::polymorphic_binary_iarchive ia(stream);
    GridMap<double> grid_i;
    ia >> grid_i;

    BOOST_CHECK_EQUAL(grid_o.getNumCells(), grid_i.getNumCells());
    BOOST_CHECK_EQUAL(grid_o.getResolution(), grid_i.getResolution());
    BOOST_CHECK_EQUAL(grid_o.getSize().isApprox(grid_i.getSize(), 0.0001), true);
}*/
