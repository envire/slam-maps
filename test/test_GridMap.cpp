#include <boost/test/unit_test.hpp>
#include <maps/grid/GridMap.hpp>

#include <boost/math/special_functions/fpclassify.hpp>

#include <chrono>

using namespace ::maps;

double default_value = std::numeric_limits<double>::quiet_NaN();
//double default_value = std::numeric_limits<double>::infinity();

bool compare_default_value(const double &value1, const double &value2, const double &d_value = default_value)
{
   if (boost::math::isnan(d_value))
   {
       return boost::math::isnan(value1) && boost::math::isnan(value2);
   }
   else
   {
       return (value1 == value2)&&(value1 == d_value);
   }
}


BOOST_AUTO_TEST_CASE(test_grid_empty)
{
    GridMap<double> *grid = new GridMap<double>();

    /** Check grid members **/
    BOOST_CHECK_EQUAL(grid->getNumCells(), Vector2ui(0, 0));
    BOOST_CHECK_EQUAL(grid->getResolution(), Vector2d(0, 0));
    BOOST_CHECK_EQUAL(grid->getSize(), Vector2d(0, 0));    
    BOOST_CHECK_EQUAL(grid->getNumElements(), 0);       
    
    // if the grid has size of (0,0)
    BOOST_CHECK_THROW(grid->at(Vector3d(0,0,0)), std::exception);    
    BOOST_CHECK_THROW(grid->getMax(), std::exception);  
    BOOST_CHECK_THROW(grid->getMin(), std::exception);  

    // since the grid has no size, return false
    // the pos should be unchanged
    Vector3d gt_pos(-5.13, 100.57, -89.89);
    Vector3d pos(gt_pos);
    BOOST_CHECK_EQUAL(grid->fromGrid(Index(0, 0), pos), false);
    BOOST_CHECK_EQUAL(pos, gt_pos);    

    // since the grid has no size, return false
    // the pos should be unchanged
    Index gt_idx(57, 13);
    Index idx(gt_idx);
    BOOST_CHECK_EQUAL(grid->toGrid(Eigen::Vector3d(0, 0, 0), idx), false);
    BOOST_CHECK_EQUAL(idx, gt_idx);    

    // function inherited through storage    
    BOOST_CHECK_THROW(grid->at(Index(0,0)), std::exception);
    BOOST_CHECK_THROW(grid->at(0, 0), std::exception);    
    BOOST_CHECK_EQUAL((grid->begin() == grid->end()), true);
    BOOST_CHECK_NO_THROW(grid->clear());
    BOOST_CHECK_NO_THROW(grid->moveBy(Index(0,0)));

    delete grid;
}

BOOST_AUTO_TEST_CASE(test_grid_constructor)
{
    GridMap<double> grid(Vector2ui(100, 100), Vector2d(0.153, 0.257), -5.5);

    /** Check grid members **/
    BOOST_CHECK_EQUAL(grid.getNumCells(), Vector2ui(100, 100));
    BOOST_CHECK_EQUAL(grid.getResolution(), Vector2d(0.153, 0.257));
    BOOST_CHECK_EQUAL(grid.getSize().isApprox(Vector2d(15.3, 25.7), 0.0001), true);
    BOOST_CHECK_EQUAL(grid.getNumElements(), (100*100)); 

    BOOST_CHECK_EQUAL(grid.at(Vector3d(0,0,0)), -5.5);

    /** Check local map members **/
    BOOST_CHECK_EQUAL(grid.getId(), ::maps::UNKNOWN_MAP_ID);
    BOOST_CHECK_EQUAL(grid.getMapType(), ::maps::LocalMapType::GRID_MAP);
    BOOST_CHECK_EQUAL(grid.getEPSGCode(), ::maps::UNKNOWN_EPSG_CODE);
    BOOST_CHECK_EQUAL(grid.getLocalFrame().translation(), base::Transform3d::Identity().translation());
    BOOST_CHECK_EQUAL(grid.getLocalFrame().rotation(), base::Transform3d::Identity().rotation());
}

BOOST_AUTO_TEST_CASE(test_grid_index_in_grid)
{
    //Grid of 10x100
    GridMap<double> grid(Vector2ui(100, 200), Vector2d(0.1, 0.5), -5.5);

    BOOST_CHECK_EQUAL(grid.inGrid(Index(0, 0)), true);          // bottom left (according to 1873-2015 IEEE standard)
    BOOST_CHECK_EQUAL(grid.inGrid(Index(99, 199)), true);       // top right (according to 1873-2015 IEEE standard)
    BOOST_CHECK_EQUAL(grid.inGrid(Index(50, 100)), true);       // middle

    BOOST_CHECK_EQUAL(grid.inGrid(Index(100, 200)), false);     // outside
    BOOST_CHECK_EQUAL(grid.inGrid(Index(99, 200)), false);      // outside
    BOOST_CHECK_EQUAL(grid.inGrid(Index(100, 199)), false);     // outside
    BOOST_CHECK_EQUAL(grid.inGrid(Index(-1, -1)), false);       // outside
    BOOST_CHECK_EQUAL(grid.inGrid(Index(-1, 2)), false);        // outside
    BOOST_CHECK_EQUAL(grid.inGrid(Index(2, -1)), false);        // outside
}

BOOST_AUTO_TEST_CASE(test_grid_copy)
{
    Vector2ui num_cells(100, 200);
    Vector2d resolution(0.1, 0.5);

    /* Grid map of 10x100 meters **/
    GridMap<double> *grid = new GridMap<double>(num_cells, resolution, default_value);
    grid->getLocalFrame().translate(Eigen::Vector3d::Random(3));

    for (unsigned int x = 0; x < grid->getNumCells().x(); ++x)
    {
        for (unsigned int y = 0; y < grid->getNumCells().y(); ++y)
        {
            double cell_value = rand();
            grid->at(x, y) = cell_value;
        }
    }

    GridMap<double> *grid_copy = new GridMap<double>(*grid);

    // check configuration
    BOOST_CHECK_EQUAL(grid_copy->getNumCells(), grid->getNumCells()); 
    BOOST_CHECK_EQUAL(grid_copy->getResolution(), grid->getResolution()); 
    BOOST_CHECK_EQUAL(grid_copy->getLocalFrame().translation(), grid->getLocalFrame().translation());    
    BOOST_CHECK_EQUAL(grid_copy->getLocalFrame().rotation(), grid->getLocalFrame().rotation());    

    // check default value
    BOOST_CHECK_EQUAL(compare_default_value(grid_copy->getDefaultValue(), grid->getDefaultValue()), true);

    // check cell value
    for (unsigned int x = 0; x < grid->getNumCells().x(); ++x)
    {
        for (unsigned int y = 0; y < grid->getNumCells().y(); ++y)
        {
            BOOST_CHECK_EQUAL(grid->at(x, y), grid_copy->at(x,y));
        }
    }

    delete grid;
    delete grid_copy;
}

BOOST_AUTO_TEST_CASE(test_grid_cell_access)
{
    Vector2ui num_cells(100, 200);
    Vector2d resolution(0.1, 0.5);

    /* Grid map of 10x100 meters **/
    GridMap<double> *grid = new GridMap<double>(num_cells, resolution, default_value);

    // check default value
    BOOST_CHECK_EQUAL(compare_default_value(grid->getDefaultValue(), default_value), true);

    // access cell that is not in grid = > should throw error
    BOOST_CHECK_THROW(grid->at(Index(num_cells.x(), num_cells.y())), std::exception);
    BOOST_CHECK_THROW(grid->at(Vector3d(grid->getSize().x(), grid->getSize().y(), 0)), std::exception);
    BOOST_CHECK_THROW(grid->at(num_cells.x(), num_cells.y()), std::exception);

    // access cell that is in grid
    BOOST_CHECK_EQUAL(compare_default_value(grid->at(Index(num_cells.x() - 1, num_cells.y() - 1)), default_value), true);

    BOOST_CHECK_EQUAL(compare_default_value(grid->at(Vector3d(grid->getSize().x() - resolution.x() - 0.01,
                                        grid->getSize().y() - resolution.y() - 0.01,
                                        0)), default_value), true);

    BOOST_CHECK_EQUAL(compare_default_value(grid->at(num_cells.x() - 1, num_cells.y() - 1), default_value), true);

    GridMap<double> const* grid_const = grid;

    // access cell that is not in grid = > should throw error
    BOOST_CHECK_THROW(grid_const->at(Index(num_cells.x(), num_cells.y())), std::exception); 
    BOOST_CHECK_THROW(grid_const->at(Vector3d(grid->getSize().x(), grid->getSize().y(), 0)), std::exception);
    BOOST_CHECK_THROW(grid_const->at(num_cells.x(), num_cells.y()), std::exception);

    // access cell that is in grid
    BOOST_CHECK_EQUAL(compare_default_value(grid_const->at(Index(num_cells.x() - 1, num_cells.y() - 1)), default_value), true);
    BOOST_CHECK_EQUAL(compare_default_value(grid_const->at(Vector3d(grid->getSize().x() - resolution.x() - 0.01,
                                              grid->getSize().y() - resolution.y() - 0.01,
                                              0)), default_value), true);
    BOOST_CHECK_EQUAL(compare_default_value(grid_const->at(num_cells.x() - 1, num_cells.y() - 1), default_value), true);

    //delete grid;
    //delete grid_const;
}

BOOST_AUTO_TEST_CASE(test_grid_minmax)
{
    Vector2ui num_cells(100, 200);
    Vector2d resolution(0.1, 0.5);

    /** Create a grid map to test the max values **/
    GridMap<double> *grid_max = new GridMap<double>(num_cells, resolution, default_value);

    double min = std::numeric_limits<double>::max();
    double max = std::numeric_limits<double>::min();
    for (unsigned int x = 0; x < grid_max->getNumCells().x(); ++x)
    {
        for (unsigned int y = 0; y < grid_max->getNumCells().y(); ++y)
        {
            double cell_value = rand();

            if (min > cell_value)
                min = cell_value;
            if (max < cell_value)
                max = cell_value;

            grid_max->at(x, y) = cell_value;
        }
    }

    BOOST_CHECK_CLOSE(grid_max->getMax(), max, 0.000001);

    /** Include a default value in the grid map **/
    grid_max->at(0,0) = grid_max->getDefaultValue();

    /** Get the max with and without the default value **/
    BOOST_CHECK_EQUAL(compare_default_value(grid_max->getMax(), grid_max->getDefaultValue()), true);
    BOOST_CHECK_CLOSE(grid_max->getMax(false), max, 0.000001);

    /** Create a grid map to test the min values **/
    GridMap<double> *grid_min = new GridMap<double>(num_cells, resolution, -default_value);

    min = std::numeric_limits<double>::max();
    max = std::numeric_limits<double>::min();
    for (unsigned int x = 0; x < grid_min->getNumCells().x(); ++x)
    {
        for (unsigned int y = 0; y < grid_min->getNumCells().y(); ++y)
        {
            double cell_value = rand();

            if (min > cell_value)
                min = cell_value;
            if (max < cell_value)
                max = cell_value;

            grid_min->at(x, y) = cell_value;
        }
    }

    BOOST_CHECK_CLOSE(grid_min->getMin(), min, 0.000001);

    /** Include a default value in the grid map **/
    grid_min->at(0,0) = grid_min->getDefaultValue();

    /** Get the min with and without the default value **/
    BOOST_CHECK_EQUAL(compare_default_value(grid_min->getMin(), grid_min->getDefaultValue(), -default_value), true);
    BOOST_CHECK_CLOSE(grid_min->getMin(false), min, 0.000001);

    delete grid_max;
    delete grid_min;
}

BOOST_AUTO_TEST_CASE(test_grid_clear)
{
    Vector2ui num_cells(100, 200);
    Vector2d resolution(0.1, 0.5);

    GridMap<double> *grid = new GridMap<double>(num_cells, resolution, default_value);

    for (unsigned int x = 0; x < grid->getNumCells().x(); ++x)
    {
        for (unsigned int y = 0; y < grid->getNumCells().y(); ++y)
        {
            double cell_value = rand();
            grid->at(x, y) = cell_value;
        }
    }

    // check if all cell are set
    for (unsigned int x = 0; x < grid->getNumCells().x(); ++x)
    {
        for (unsigned int y = 0; y < grid->getNumCells().y(); ++y)
        {
            BOOST_CHECK_EQUAL(!compare_default_value(grid->at(x,y), grid->getDefaultValue()), true);
        }
    }

    grid->clear();

    // after the clean all cell should have default value
    for (unsigned int x = 0; x < grid->getNumCells().x(); ++x)
    {
        for (unsigned int y = 0; y < grid->getNumCells().y(); ++y)
        {
            BOOST_CHECK_EQUAL(compare_default_value(grid->at(x,y), grid->getDefaultValue()), true);
        }
    }

    delete grid;
}

BOOST_AUTO_TEST_CASE(test_grid_move_complete)
{
    Vector2ui num_cells(7, 5);
    Vector2d resolution(0.1, 0.5);

    GridMap<double> *grid = new GridMap<double>(num_cells, resolution, default_value);

    // random values
    for (unsigned int x = 0; x < grid->getNumCells().x(); ++x)
    {
        for (unsigned int y = 0; y < grid->getNumCells().y(); ++y)
        {
            grid->at(Index(x,y)) = rand();
        }
    }

    // -------------- Test 1: move all values out of grid

    grid->moveBy(Eigen::Vector2i(7,5));

    // in grid should be only default values
    for (unsigned int x = 0; x < grid->getNumCells().x(); ++x)
    {
        for (unsigned int y = 0; y < grid->getNumCells().y(); ++y)
        {
            BOOST_CHECK_EQUAL(compare_default_value(grid->at(Index(x,y)), grid->getDefaultValue()), true);
        }
    }

    // random values
    for (unsigned int x = 0; x < grid->getNumCells().x(); ++x)
    {
        for (unsigned int y = 0; y < grid->getNumCells().y(); ++y)
        {
            grid->at(Index(x,y)) = rand();
        }
    }

    // -------------- Test 2: move all values out of grid

    grid->moveBy(Eigen::Vector2i(-7,-5));

    // in grid should be only default values
    for (unsigned int x = 0; x < grid->getNumCells().x(); ++x)
    {
        for (unsigned int y = 0; y < grid->getNumCells().y(); ++y)
        {
            BOOST_CHECK_EQUAL(compare_default_value(grid->at(Index(x,y)), grid->getDefaultValue()), true);
        }
    }


    // random values
    for (unsigned int x = 0; x < grid->getNumCells().x(); ++x)
    {
        for (unsigned int y = 0; y < grid->getNumCells().y(); ++y)
        {
            grid->at(Index(x,y)) = rand();
        }
    }

    // -------------- Test 3: move all values out of grid

    grid->moveBy(Eigen::Vector2i(7,0));

    // in grid should be only default values
    for (unsigned int x = 0; x < grid->getNumCells().x(); ++x)
    {
        for (unsigned int y = 0; y < grid->getNumCells().y(); ++y)
        {
            BOOST_CHECK_EQUAL(compare_default_value(grid->at(Index(x,y)), grid->getDefaultValue()), true);
        }
    }


    // random values
    for (unsigned int x = 0; x < grid->getNumCells().x(); ++x)
    {
        for (unsigned int y = 0; y < grid->getNumCells().y(); ++y)
        {
            grid->at(Index(x,y)) = rand();
        }
    }

    // -------------- Test 4: move all values out of grid

    grid->moveBy(Eigen::Vector2i(0,5));

    // in grid should be only default values
    for (unsigned int x = 0; x < grid->getNumCells().x(); ++x)
    {
        for (unsigned int y = 0; y < grid->getNumCells().y(); ++y)
        {
            BOOST_CHECK_EQUAL(compare_default_value(grid->at(Index(x,y)), grid->getDefaultValue()), true);
        }
    }


    // -------------- Test 5: move all values out of grid

    grid->moveBy(Eigen::Vector2i(-7,0));

    // in grid should be only default values
    for (unsigned int x = 0; x < grid->getNumCells().x(); ++x)
    {
        for (unsigned int y = 0; y < grid->getNumCells().y(); ++y)
        {
            BOOST_CHECK_EQUAL(compare_default_value(grid->at(Index(x,y)), grid->getDefaultValue()), true);
        }
    }


    // random values
    for (unsigned int x = 0; x < grid->getNumCells().x(); ++x)
    {
        for (unsigned int y = 0; y < grid->getNumCells().y(); ++y)
        {
            grid->at(Index(x,y)) = rand();
        }
    }

    // -------------- Test 6: move all values out of grid

    grid->moveBy(Eigen::Vector2i(0,-5));

    // in grid should be only default values
    for (unsigned int x = 0; x < grid->getNumCells().x(); ++x)
    {
        for (unsigned int y = 0; y < grid->getNumCells().y(); ++y)
        {
            BOOST_CHECK_EQUAL(compare_default_value(grid->at(Index(x,y)), grid->getDefaultValue()), true);
        }
    }
}

BOOST_AUTO_TEST_CASE(test_grid_move_partially)
{
    Vector2ui num_cells(7, 5);
    Vector2d resolution(0.1, 0.5);

    GridMap<double> *grid = new GridMap<double>(num_cells, resolution, default_value);

    // -------------- Test 1: move by Index Offset (0,0);

    //Y
    //^  0 1 2 3 4 5 6
    //|  0 1 2 3 4 5 6
    //|  0 1 2 3 4 5 6
    //|  0 1 2 3 4 5 6
    //|  0 1 2 3 4 5 6
    //O - - - - - - - -> X
    for (unsigned int y = 0; y < grid->getNumCells().y(); ++y)
    {
        for (unsigned int x = 0; x < grid->getNumCells().x(); ++x)
        {
            grid->at(Index(x,y)) = x;
        }
    }

    // Printing utility loop
    //for (unsigned int y = grid->getNumCells().y(); y-->0;)
    //{
    //    for (unsigned int x = 0; x < grid->getNumCells().x(); ++x)
    //    {
    //        std::cout<<"["<<x<<","<<y<<"]";
    //        std::cout<<grid->at(Index(x,y))<<"\t";
    //    }
    //    std::cout<<"\n";
    //}


    grid->moveBy(Eigen::Vector2i(0, 0));

    for (unsigned int y = 0; y < grid->getNumCells().y(); ++y)
    {
        for (unsigned int x = 0; x < grid->getNumCells().x(); ++x)
        {
            BOOST_CHECK_EQUAL(grid->at(Index(x,y)), x);
        }
    }

    // -------------- Test 2: move by Index Offset (1,0);

    //Y
    //^  0 1 2 3 4 5 6
    //|  0 1 2 3 4 5 6
    //|  0 1 2 3 4 5 6
    //|  0 1 2 3 4 5 6
    //|  0 1 2 3 4 5 6
    //O - - - - - - - -> X
    for (unsigned int y = 0; y < grid->getNumCells().y(); ++y)
    {
        for (unsigned int x = 0; x < grid->getNumCells().x(); ++x)
        {
            grid->at(Index(x,y)) = x;
        }
    }

    //Y
    //^  inf 0 1 2 3 4 5
    //|  inf 0 1 2 3 4 5
    //|  inf 0 1 2 3 4 5
    //|  inf 0 1 2 3 4 5
    //|  inf 0 1 2 3 4 5
    //O - - - - - - - -> X

    grid->moveBy(Eigen::Vector2i(1, 0));

    for (unsigned int y = 0; y < grid->getNumCells().y(); ++y)
    {
        for (unsigned int x = 0; x < grid->getNumCells().x(); ++x)
        {
            if (x == 0)
            {
                BOOST_CHECK_EQUAL(compare_default_value(grid->at(Index(x,y)), grid->getDefaultValue()), true);
            }
            else
                BOOST_CHECK_EQUAL(grid->at(Index(x,y)), x-1);
        }
    }

   // -------------- Test 3: move by Index Offset (0,1);

    //Y
    //^  0 1 2 3 4 5 6
    //|  0 1 2 3 4 5 6
    //|  0 1 2 3 4 5 6
    //|  0 1 2 3 4 5 6
    //|  0 1 2 3 4 5 6
    //O - - - - - - - -> X
    for (unsigned int y = 0; y < grid->getNumCells().y(); ++y)
    {
        for (unsigned int x = 0; x < grid->getNumCells().x(); ++x)
        {
            grid->at(Index(x,y)) = x;
        }
    }

    //Y
    //^  0 1 2 3 4 5 6
    //|  0 1 2 3 4 5 6
    //|  0 1 2 3 4 5 6
    //|  0 1 2 3 4 5 6
    //| inf inf inf inf inf inf inf
    //O - - - - - - - -> X

    grid->moveBy(Eigen::Vector2i(0, 1));

    for (unsigned int y = 0; y < grid->getNumCells().y(); ++y)
    {
        for (unsigned int x = 0; x < grid->getNumCells().x(); ++x)
        {
            if (y == 0)
            {
                BOOST_CHECK_EQUAL(compare_default_value(grid->at(Index(x,y)), grid->getDefaultValue()), true);
            }
            else
                BOOST_CHECK_EQUAL(grid->at(Index(x,y)), x);
        }
    }

    // -------------- Test 4: move by Index Offset (1,1);

    //Y
    //^  0 1 2 3 4 5 6
    //|  0 1 2 3 4 5 6
    //|  0 1 2 3 4 5 6
    //|  0 1 2 3 4 5 6
    //|  0 1 2 3 4 5 6
    //O - - - - - - - -> X
    for (unsigned int y = 0; y < grid->getNumCells().y(); ++y)
    {
        for (unsigned int x = 0; x < grid->getNumCells().x(); ++x)
        {
            grid->at(Index(x,y)) = x;
        }
    }

    //Y
    //^  inf 0 1 2 3 4 5
    //|  inf 0 1 2 3 4 5
    //|  inf 0 1 2 3 4 5
    //|  inf 0 1 2 3 4 5
    //|  inf inf inf inf inf inf inf
    //O - - - - - - - -> X

    grid->moveBy(Eigen::Vector2i(1, 1));

    for (unsigned int y = 0; y < grid->getNumCells().y(); ++y)
    {
        for (unsigned int x = 0; x < grid->getNumCells().x(); ++x)
        {
            if (x==0 || y == 0)
            {
                BOOST_CHECK_EQUAL(compare_default_value(grid->at(Index(x,y)), grid->getDefaultValue()), true);
            }
            else
                BOOST_CHECK_EQUAL(grid->at(Index(x,y)), x-1);
        }
    }

    // -------------- Test 4: move by Index Offset (-1,-1);

    //Y
    //^  0 1 2 3 4 5 6
    //|  0 1 2 3 4 5 6
    //|  0 1 2 3 4 5 6
    //|  0 1 2 3 4 5 6
    //|  0 1 2 3 4 5 6
    //O - - - - - - - -> X
    for (unsigned int y = 0; y < grid->getNumCells().y(); ++y)
    {
        for (unsigned int x = 0; x < grid->getNumCells().x(); ++x)
        {
            grid->at(Index(x,y)) = x;
        }
    }

    //Y
    //^  inf inf inf inf inf inf inf
    //|  1 2 3 4 5 6 inf
    //|  1 2 3 4 5 6 inf
    //|  1 2 3 4 5 6 inf
    //|  1 2 3 4 5 6 inf
    //O - - - - - - - -> X

    grid->moveBy(Eigen::Vector2i(-1, -1));

    for (unsigned int y = 0; y < grid->getNumCells().y(); ++y)
    {
        for (unsigned int x = 0; x < grid->getNumCells().x(); ++x)
        {
            if (x==grid->getNumCells().x()-1 || y == grid->getNumCells().y()-1)
            {
                BOOST_CHECK_EQUAL(compare_default_value(grid->at(Index(x,y)), grid->getDefaultValue()), true);
            }
            else
                BOOST_CHECK_EQUAL(grid->at(Index(x,y)), x+1);
        }
    }


    // -------------- Test 5: move by Index Offset (3,3);

    //Y
    //^  0 1 2 3 4 5 6
    //|  0 1 2 3 4 5 6
    //|  0 1 2 3 4 5 6
    //|  0 1 2 3 4 5 6
    //|  0 1 2 3 4 5 6
    //O - - - - - - - -> X
    for (unsigned int y = 0; y < grid->getNumCells().y(); ++y)
    {
        for (unsigned int x = 0; x < grid->getNumCells().x(); ++x)
        {
            grid->at(Index(x,y)) = x;
        }
    }

    grid->moveBy(Eigen::Vector2i(3, 3));

    for (unsigned int y = 0; y < grid->getNumCells().y(); ++y)
    {
        for (unsigned int x = 0; x < grid->getNumCells().x(); ++x)
        {
            if (x < grid->getNumCells().x()-(grid->getNumCells().x()-3)  || y < grid->getNumCells().y()-(grid->getNumCells().y()-3))
            {
                BOOST_CHECK_EQUAL(compare_default_value(grid->at(Index(x,y)), grid->getDefaultValue()), true);
            }
            else
                BOOST_CHECK_EQUAL(grid->at(Index(x,y)), x-3);
        }
    }

    // -------------- Test 6: move by Index Offset (-3,-3);

    //Y
    //^  0 1 2 3 4 5 6
    //|  0 1 2 3 4 5 6
    //|  0 1 2 3 4 5 6
    //|  0 1 2 3 4 5 6
    //|  0 1 2 3 4 5 6
    //O - - - - - - - -> X
    for (unsigned int y = 0; y < grid->getNumCells().y(); ++y)
    {
        for (unsigned int x = 0; x < grid->getNumCells().x(); ++x)
        {
            grid->at(Index(x,y)) = x;
        }
    }

    grid->moveBy(Eigen::Vector2i(-3, -3));

    for (unsigned int y = 0; y < grid->getNumCells().y(); ++y)
    {
        for (unsigned int x = 0; x < grid->getNumCells().x(); ++x)
        {
            if (x >= grid->getNumCells().x()-3 ||y >= grid->getNumCells().y()-3)
            {
                BOOST_CHECK_EQUAL(compare_default_value(grid->at(Index(x,y)), grid->getDefaultValue()), true);
            }
            else
                BOOST_CHECK_EQUAL(grid->at(Index(x,y)), x+3);
        }
    }

    delete grid;
}


BOOST_AUTO_TEST_CASE(test_grid_move_partially_check_magic_number)
{
    double magic_number = 8.00;
    Vector2ui num_cells(7, 5);
    Vector2d resolution(0.1, 0.5);

    GridMap<double> *grid = new GridMap<double>(num_cells, resolution, 0.0);

    // -------------- Test 1: move by Index Offset (2,2);

    //Y
    //^  0 0 0 0 0 0 0
    //|  0 0 0 0 0 0 0
    //|  0 0 0 8 0 0 0
    //|  0 0 0 0 0 0 0
    //|  0 0 0 0 0 0 0
    //O - - - - - - - -> X
    grid->at(Index(3,2)) = magic_number;

    // Printing utility loop
    //for (unsigned int y = grid->getNumCells().y(); y-->0;)
    //{
    //    for (unsigned int x = 0; x < grid->getNumCells().x(); ++x)
    //    {
    //        std::cout<<"["<<x<<","<<y<<"]";
    //        std::cout<<grid->at(Index(x,y))<<"\t";
    //    }
    //    std::cout<<"\n";
    //}

    grid->moveBy(Eigen::Vector2i(2, 2));

    BOOST_CHECK_EQUAL(grid->at(Index(5,4)), magic_number);
}

BOOST_AUTO_TEST_CASE(test_grid_from_grid_without_offset)
{
    BOOST_TEST_MESSAGE("test_grid_from_grid_without_offset");

    // size: 10 x 100
    GridMap<double> grid(Vector2ui(100, 200), Vector2d(0.1, 0.5), -5.5);

    Vector3d pos;

    // bottom left (according to 1873-2015 IEEE standard) is you see it as x
    // horizontal and y vertical
    // Y
    // ^
    // |
    // | (0,0) (1,0)...
    // O - - - - - - - > X
    // or bottom right if you see it x vertical and y horizontal. The important
    // thing is that the coordinate system does not change.
    //            X
    //            ^
    //            |
    //       (2,0)|
    //       (1,0)|
    //       (0,0)|
    //  Y < - - - O
    BOOST_CHECK_EQUAL(grid.fromGrid(Index(0, 0), pos), true);
    BOOST_CHECK_EQUAL(pos.isApprox(Vector3d(0.05, 0.25, 0.), 0.0001), true);
    BOOST_TEST_MESSAGE("Position from grid: "<<pos[0]<<","<<pos[1]<<","<<pos[2]);

    // Some other indexes close to the origin
    BOOST_CHECK_EQUAL(grid.fromGrid(Index(0, 1), pos), true);
    BOOST_CHECK_EQUAL(pos.isApprox(Vector3d(0.05, 0.75, 0.), 0.0001), true);
    BOOST_TEST_MESSAGE("Position from grid: "<<pos[0]<<","<<pos[1]<<","<<pos[2]);

    BOOST_CHECK_EQUAL(grid.fromGrid(Index(1, 0), pos), true);
    BOOST_CHECK_EQUAL(pos.isApprox(Vector3d(0.15, 0.25, 0.), 0.0001), true);
    BOOST_TEST_MESSAGE("Position from grid: "<<pos[0]<<","<<pos[1]<<","<<pos[2]);

    BOOST_CHECK_EQUAL(grid.fromGrid(Index(1, 1), pos), true);
    BOOST_CHECK_EQUAL(pos.isApprox(Vector3d(0.15, 0.75, 0.), 0.0001), true); 
    BOOST_TEST_MESSAGE("Position from grid: "<<pos[0]<<","<<pos[1]<<","<<pos[2]);

    //  top right (according to 1873-2015 IEEE standard)
    BOOST_CHECK_EQUAL(grid.fromGrid(Index(99, 199), pos), true);
    BOOST_CHECK_EQUAL(pos.isApprox(Vector3d(9.95, 99.75, 0.), 0.0001), true);
    BOOST_TEST_MESSAGE("Position from grid: "<<pos[0]<<","<<pos[1]<<","<<pos[2]);

    // middle in X axis
    BOOST_CHECK_EQUAL(grid.fromGrid(Index(49, 0), pos), true);
    BOOST_CHECK_EQUAL(pos.isApprox(Vector3d(4.95, 0.25, 0.), 0.0001), true);
    BOOST_TEST_MESSAGE("Middle in X-axis Position from grid: "<<pos[0]<<","<<pos[1]<<","<<pos[2]);

    // middle in Y axis
    BOOST_CHECK_EQUAL(grid.fromGrid(Index(0, 99), pos), true);
    BOOST_CHECK_EQUAL(pos.isApprox(Vector3d(0.05, 49.75, 0.), 0.0001), true);
    BOOST_TEST_MESSAGE("Middle in Y-axis Position from grid: "<<pos[0]<<","<<pos[1]<<","<<pos[2]);

    // middle case 1
    BOOST_CHECK_EQUAL(grid.fromGrid(Index(49, 99), pos), true);
    BOOST_CHECK_EQUAL(pos.isApprox(Vector3d(4.95, 49.75, 0.), 0.0001), true);
    BOOST_TEST_MESSAGE("Middle Position from grid: "<<pos[0]<<","<<pos[1]<<","<<pos[2]);

    // middle case 2
    BOOST_CHECK_EQUAL(grid.fromGrid(Index(50, 100), pos), true);
    BOOST_CHECK_EQUAL(pos.isApprox(Vector3d(5.05, 50.25, 0.), 0.0001), true);
    BOOST_TEST_MESSAGE("Middle Position from grid: "<<pos[0]<<","<<pos[1]<<","<<pos[2]);

    // outside: pos should be unchanged
    BOOST_CHECK_EQUAL(grid.fromGrid(Index(100, 200), pos), false);
    BOOST_CHECK_EQUAL(pos.isApprox(Vector3d(5.05, 50.25, 0.), 0.0001), true);
}

BOOST_AUTO_TEST_CASE(test_grid_from_grid_with_offset)
{
    BOOST_TEST_MESSAGE("test_grid_from_grid_with_offset");

    // size: 10 x 100
    GridMap<double> grid(Vector2ui(100, 200), Vector2d(0.1, 0.5), -5.5);   
    grid.getLocalFrame().translate(Vector3d(5, 50, 0));

    // ---- Index 2 Position ----
    Vector3d pos;

    // bottom left (according to 1873-2015 IEEE standard)
    BOOST_CHECK_EQUAL(grid.fromGrid(Index(0, 0), pos), true);
    BOOST_CHECK_EQUAL(pos.isApprox(Vector3d(-4.95, -49.75, 0.), 0.0001), true);

    // top right (according to 1873-2015 IEEE standard)
    BOOST_CHECK_EQUAL(grid.fromGrid(Index(99, 199), pos), true);
    BOOST_CHECK_EQUAL(pos.isApprox(Vector3d(4.95, 49.75, 0.), 0.0001), true);

    // middle
    BOOST_CHECK_EQUAL(grid.fromGrid(Index(50, 100), pos), true);
    BOOST_CHECK_EQUAL(pos.isApprox(Vector3d(0.05, 0.25, 0.), 0.0001), true);

    // outside: pos should be unchanged
    BOOST_CHECK_EQUAL(grid.fromGrid(Index(100, 200), pos), false);
    BOOST_CHECK_EQUAL(pos.isApprox(Vector3d(0.05, 0.25, 0.), 0.0001), true);

    //TODO: write more tests with the Transform3d including rotation
}

BOOST_AUTO_TEST_CASE(test_grid_from_grid_translate_grid)
{
    BOOST_TEST_MESSAGE("test_grid_from_grid_with_offset");

    // size: 10 x 100
    GridMap<double> grid(Vector2ui(100, 200), Vector2d(0.1, 0.5), -5.5);   
    grid.translate(Vector3d(-5, -50, 0));

    // ---- Index 2 Position ----
    Vector3d pos;

    // bottom left (according to 1873-2015 IEEE standard)
    BOOST_CHECK_EQUAL(grid.fromGrid(Index(0, 0), pos), true);
    BOOST_CHECK_EQUAL(pos.isApprox(Vector3d(-4.95, -49.75, 0.), 0.0001), true);

    // top right (according to 1873-2015 IEEE standard)
    BOOST_CHECK_EQUAL(grid.fromGrid(Index(99, 199), pos), true);
    BOOST_CHECK_EQUAL(pos.isApprox(Vector3d(4.95, 49.75, 0.), 0.0001), true);

    // middle
    BOOST_CHECK_EQUAL(grid.fromGrid(Index(50, 100), pos), true);
    BOOST_CHECK_EQUAL(pos.isApprox(Vector3d(0.05, 0.25, 0.), 0.0001), true);

    // outside: pos should be unchanged
    BOOST_CHECK_EQUAL(grid.fromGrid(Index(100, 200), pos), false);
    BOOST_CHECK_EQUAL(pos.isApprox(Vector3d(0.05, 0.25, 0.), 0.0001), true);

    //TODO: write more tests with the Transform3d including rotation
}

BOOST_AUTO_TEST_CASE(test_grid_from_grid_in_specific_frame)
{
    BOOST_TEST_MESSAGE("test_grid_from_grid_in_specific_frame");

    // size: 10 x 50
    GridMap<double> grid(Vector2ui(100, 100), Vector2d(0.1, 0.5), -5.5);
    grid.getLocalFrame().translate(Vector3d(5, 50, 0));

    // Cartesian position
    Eigen::Vector3d pos;

    // bottom left (according to 1873-2015 IEEE standard)
    // (without specific frame given)
    BOOST_CHECK_EQUAL(grid.fromGrid(Index(0, 0), pos), true);
    BOOST_CHECK_EQUAL(pos.isApprox(Vector3d(-4.95, -49.75, 0.0), 0.0001), true);
    BOOST_TEST_MESSAGE("Position from grid: "<<pos[0]<<","<<pos[1]<<","<<pos[2]);

    // ---- Index to Position in the specific frame ----
    base::Transform3d frame_in_grid(Eigen::Translation3d(-5, -50, -0.5 ));

    // bottom left (according to 1873-2015 IEEE standard)
    BOOST_CHECK_EQUAL(grid.fromGrid(Index(0, 0), pos, frame_in_grid), true);
    BOOST_CHECK_EQUAL(pos.isApprox(Vector3d(0.05, 0.25, 0.5), 0.0001), true);
    BOOST_TEST_MESSAGE("Position from grid: "<<pos[0]<<","<<pos[1]<<","<<pos[2]);

    // top right (according to 1873-2015 IEEE standard)
    BOOST_CHECK_EQUAL(grid.fromGrid(Index(99, 99), pos, frame_in_grid), true);
    BOOST_CHECK_EQUAL(pos.isApprox(Vector3d(9.95, 49.75, 0.5), 0.0001), true);
    BOOST_TEST_MESSAGE("Position from grid: "<<pos[0]<<","<<pos[1]<<","<<pos[2]);

    // index middle in X
    BOOST_CHECK_EQUAL(grid.fromGrid(Index(50, 99), pos, frame_in_grid), true);
    BOOST_CHECK_EQUAL(pos.isApprox(Vector3d(5.05, 49.75, 0.5), 0.0001), true);
    BOOST_TEST_MESSAGE("Middle in X-axis Position from grid: "<<pos[0]<<","<<pos[1]<<","<<pos[2]);

    // index middle in Y
    BOOST_CHECK_EQUAL(grid.fromGrid(Index(99, 50), pos, frame_in_grid), true);
    BOOST_CHECK_EQUAL(pos.isApprox(Vector3d(9.95, 25.25, 0.5), 0.0001), true);
    BOOST_TEST_MESSAGE("Middle in Y-axis Position from grid: "<<pos[0]<<","<<pos[1]<<","<<pos[2]);

    // index middle in X and Y
    BOOST_CHECK_EQUAL(grid.fromGrid(Index(50, 50), pos, frame_in_grid), true);
    BOOST_CHECK_EQUAL(pos.isApprox(Vector3d(5.05, 25.25, 0.5), 0.0001), true);
    BOOST_TEST_MESSAGE("Middle Position from grid: "<<pos[0]<<","<<pos[1]<<","<<pos[2]);

    pos.setZero();

    // outside: pos should be unchanged
    BOOST_CHECK_EQUAL(grid.fromGrid(Index(100, 100), pos, frame_in_grid), false);
    BOOST_CHECK_EQUAL(pos.isApprox(Vector3d(0.00, 0.00, 0.00), 0.0001), true);

    // outside: pos should be unchanged
    BOOST_CHECK_EQUAL(grid.fromGrid(Index(100, 50), pos, frame_in_grid), false);
    BOOST_CHECK_EQUAL(pos.isApprox(Vector3d(0.00, 0.00, 0.00), 0.0001), true);

    // outside: pos should be unchanged
    BOOST_CHECK_EQUAL(grid.fromGrid(Index(50, 100), pos, frame_in_grid), false);
    BOOST_CHECK_EQUAL(pos.isApprox(Vector3d(0.00, 0.00, 0.00), 0.0001), true);
}

BOOST_AUTO_TEST_CASE(test_grid_to_grid_without_offset)
{
    // size: 10 x 100
    GridMap<double> grid(Vector2ui(100, 200), Vector2d(0.1, 0.5), -5.5);

    // ---- Position 2 Index ---- 

    Index idx;
    Vector3d pos_diff;

    // bottom left (according to 1873-2015 IEEE standard)
    // pos is corner of the cell
    BOOST_CHECK_EQUAL(grid.toGrid(Vector3d(0.0, 0.0, 0.), idx, pos_diff), true);
    BOOST_CHECK_EQUAL(idx, Index(0,0));
    BOOST_CHECK_EQUAL(pos_diff.isApprox(Vector3d(-0.05, -0.25, 0.0), 0.0001), true);

    // bottom left (according to 1873-2015 IEEE standard)
    // pos is center of the cell
    BOOST_CHECK_EQUAL(grid.toGrid(Vector3d(0.05, 0.25, 0.), idx, pos_diff), true);
    BOOST_CHECK_EQUAL(idx, Index(0,0));
    BOOST_CHECK_EQUAL(pos_diff.norm() <= 1e-6, true); 

    BOOST_CHECK_EQUAL(grid.toGrid(Vector3d(0.1, 0.5, 0.), idx, pos_diff), true);
    BOOST_CHECK_EQUAL(idx, Index(1,1));
    BOOST_CHECK_EQUAL(pos_diff.isApprox(Vector3d(-0.05, -0.25, 0.0), 0.0001), true);

    BOOST_CHECK_EQUAL(grid.toGrid(Vector3d(0.1, 0.0, 0.), idx, pos_diff), true);
    BOOST_CHECK_EQUAL(idx, Index(1,0));
    BOOST_CHECK_EQUAL(pos_diff.isApprox(Vector3d(-0.05, -0.25, 0.0), 0.0001), true);

    // top right (according to 1873-2015 IEEE standard)
    // pos is center of the cell
    BOOST_CHECK_EQUAL(grid.toGrid(Vector3d(9.95, 99.75, 0.), idx, pos_diff), true);
    BOOST_CHECK_EQUAL(idx, Index(99, 199));
    BOOST_CHECK_EQUAL(pos_diff.norm() <= 1e-6, true);

    BOOST_CHECK_EQUAL(grid.toGrid(Vector3d(9.99, 99.99, 0.), idx, pos_diff), true);
    BOOST_CHECK_EQUAL(idx, Index(99, 199));
    BOOST_CHECK_EQUAL(pos_diff.isApprox(Vector3d(0.04, 0.24, 0.0), 0.0001), true); //difference between 9.99 - 9.95 = 0.04 (x coordinate)

    idx.setZero();
    pos_diff.setZero();
    // Outside of the grid (it does not modify the grid)
    BOOST_CHECK_EQUAL(grid.toGrid(Vector3d(10, 100, 0.), idx, pos_diff), false);
    BOOST_CHECK_EQUAL(idx, Index(0, 0));
    BOOST_CHECK_EQUAL(pos_diff.norm() <= 1e-6, true);
}

BOOST_AUTO_TEST_CASE(test_grid_to_grid_with_offset)
{
    // size: 10 x 100
    GridMap<double> grid(Vector2ui(100, 200), Vector2d(0.1, 0.5), -5.5);
    grid.getLocalFrame().translate(Vector3d(5, 50, 0));

    // ---- Index 2 Position ---- 
    Index idx;
    Vector3d pos_diff;

    // bottom left (according to 1873-2015 IEEE standard)
    // the position on the center of the cell
    BOOST_CHECK_EQUAL(grid.toGrid(Vector3d(-5, -50, 0.), idx, pos_diff), true);
    BOOST_CHECK_EQUAL(idx, Index(0,0));
    BOOST_CHECK_EQUAL(pos_diff.isApprox(Vector3d(-0.05, -0.25, 0.0), 0.0001), true);
    BOOST_TEST_MESSAGE("Position in grid: "<<idx[0]<<","<<idx[1]);

    BOOST_CHECK_EQUAL(grid.toGrid(Vector3d(-4.91, -49.51, 0.), idx, pos_diff), true);
    BOOST_CHECK_EQUAL(idx, Index(0,0));
    BOOST_CHECK_EQUAL(pos_diff.isApprox(Vector3d(0.04, 0.24, 0.0), 0.0001), true); // difference 4.95 - 4.91 = 0.04 (x coordinate)
    BOOST_TEST_MESSAGE("Position in grid: "<<idx[0]<<","<<idx[1]);

    BOOST_CHECK_EQUAL(grid.toGrid(Vector3d(-4.95, -49.75, 0.), idx, pos_diff), true);
    BOOST_CHECK_EQUAL(idx, Index(0,0));
    BOOST_CHECK_EQUAL(pos_diff.norm() <= 1e-6, true);
    BOOST_TEST_MESSAGE("Position in grid: "<<idx[0]<<","<<idx[1]);

    // bottom left (according to 1873-2015 IEEE standard)
    // the position on the corner of the cell
    BOOST_CHECK_EQUAL(grid.toGrid(Vector3d(-4.89, -49.49, 0.), idx, pos_diff), true);
    BOOST_CHECK_EQUAL(idx, Index(1,1));
    BOOST_CHECK_EQUAL(pos_diff.isApprox(Vector3d(-0.04, -0.24, 0.0), 0.0001), true);
    BOOST_TEST_MESSAGE("Position in grid: "<<idx[0]<<","<<idx[1]);

    // top right (according to 1873-2015 IEEE standard)
    // the position on the center of the cell
    BOOST_CHECK_EQUAL(grid.toGrid(Vector3d(4.95, 49.75, 0.), idx, pos_diff), true);
    BOOST_CHECK_EQUAL(idx, Index(99, 199));
    BOOST_CHECK_EQUAL(pos_diff.norm() <= 1e-6, true);
    BOOST_TEST_MESSAGE("Position in grid: "<<idx[0]<<","<<idx[1]);

    idx.setZero();
    pos_diff.setZero();

    // top right (according to 1873-2015 IEEE standard)
    // the position is the corner of the cell is out of the grid
    BOOST_CHECK_EQUAL(grid.toGrid(Vector3d(5, 50, 0.), idx, pos_diff), false);
    BOOST_CHECK_EQUAL(idx, Index(0, 0));
    BOOST_CHECK_EQUAL(pos_diff.norm() <= 1e-6, true);
    BOOST_TEST_MESSAGE("Position in grid: "<<idx[0]<<","<<idx[1]);

    //TODO: write more tests with the Transform3d including rotation
}

BOOST_AUTO_TEST_CASE(test_grid_to_grid_translate_grid)
{
    // size: 10 x 100
    GridMap<double> grid(Vector2ui(100, 200), Vector2d(0.1, 0.5), -5.5);
    grid.translate(Vector3d(-5, -50, 0));

    // ---- Index 2 Position ---- 
    Index idx;
    Vector3d pos_diff;

    // bottom left (according to 1873-2015 IEEE standard)
    // the position on the center of the cell
    BOOST_CHECK_EQUAL(grid.toGrid(Vector3d(-5, -50, 0.), idx, pos_diff), true);
    BOOST_CHECK_EQUAL(idx, Index(0,0));
    BOOST_CHECK_EQUAL(pos_diff.isApprox(Vector3d(-0.05, -0.25, 0.0), 0.0001), true);
    BOOST_TEST_MESSAGE("Position in grid: "<<idx[0]<<","<<idx[1]);

    BOOST_CHECK_EQUAL(grid.toGrid(Vector3d(-4.91, -49.51, 0.), idx, pos_diff), true);
    BOOST_CHECK_EQUAL(idx, Index(0,0));
    BOOST_CHECK_EQUAL(pos_diff.isApprox(Vector3d(0.04, 0.24, 0.0), 0.0001), true); // difference 4.95 - 4.91 = 0.04 (x coordinate)
    BOOST_TEST_MESSAGE("Position in grid: "<<idx[0]<<","<<idx[1]);

    BOOST_CHECK_EQUAL(grid.toGrid(Vector3d(-4.95, -49.75, 0.), idx, pos_diff), true);
    BOOST_CHECK_EQUAL(idx, Index(0,0));
    BOOST_CHECK_EQUAL(pos_diff.norm() <= 1e-6, true);
    BOOST_TEST_MESSAGE("Position in grid: "<<idx[0]<<","<<idx[1]);

    // bottom left (according to 1873-2015 IEEE standard)
    // the position on the corner of the cell
    BOOST_CHECK_EQUAL(grid.toGrid(Vector3d(-4.89, -49.49, 0.), idx, pos_diff), true);
    BOOST_CHECK_EQUAL(idx, Index(1,1));
    BOOST_CHECK_EQUAL(pos_diff.isApprox(Vector3d(-0.04, -0.24, 0.0), 0.0001), true);
    BOOST_TEST_MESSAGE("Position in grid: "<<idx[0]<<","<<idx[1]);

    // top right (according to 1873-2015 IEEE standard)
    // the position on the center of the cell
    BOOST_CHECK_EQUAL(grid.toGrid(Vector3d(4.95, 49.75, 0.), idx, pos_diff), true);
    BOOST_CHECK_EQUAL(idx, Index(99, 199));
    BOOST_CHECK_EQUAL(pos_diff.norm() <= 1e-6, true);
    BOOST_TEST_MESSAGE("Position in grid: "<<idx[0]<<","<<idx[1]);

    idx.setZero();
    pos_diff.setZero();

    // top right (according to 1873-2015 IEEE standard)
    // the position is the corner of the cell is out of the grid
    BOOST_CHECK_EQUAL(grid.toGrid(Vector3d(5, 50, 0.), idx, pos_diff), false);
    BOOST_CHECK_EQUAL(idx, Index(0, 0));
    BOOST_CHECK_EQUAL(pos_diff.norm() <= 1e-6, true);
    BOOST_TEST_MESSAGE("Position in grid: "<<idx[0]<<","<<idx[1]);

    //TODO: write more tests with the Transform3d including rotation
}

BOOST_AUTO_TEST_CASE(test_grid_to_grid_in_specific_frame)
{
    // size: 10 x 50
    GridMap<double> grid(Vector2ui(100, 100), Vector2d(0.1, 0.5), -5.5);
    grid.getLocalFrame().translate(Vector3d(5, 50, 0));


    /** Index and Position **/
    Index idx;
    Eigen::Vector3d pos;

    // bottom left (according to 1873-2015 IEEE standard)
    // (without specific frame given)
    BOOST_CHECK_EQUAL(grid.toGrid(Vector3d(-4.95, -49.75, 0.0), idx), true);
    BOOST_CHECK_EQUAL(idx, Index(0,0));

    // ---- Index 2 Position in the specific frame ----
    base::Transform3d frame_in_grid(Eigen::Translation3d(-5, -50, -0.5 ));

    // bottom left (according to 1873-2015 IEEE standard)
    BOOST_CHECK_EQUAL(grid.toGrid(Vector3d(0, 0, 0), idx, frame_in_grid), true);
    BOOST_CHECK_EQUAL(idx, Index(0, 0));

    BOOST_CHECK_EQUAL(grid.toGrid(Vector3d(0.05, 0.25, 0.5), idx, frame_in_grid), true);
    BOOST_CHECK_EQUAL(idx, Index(0,0));

    // bottom left (according to 1873-2015 IEEE standard)
    BOOST_CHECK_EQUAL(grid.toGrid(Vector3d(9.95, 49.75, 0.5), idx, frame_in_grid), true);
    BOOST_CHECK_EQUAL(idx, Index(99, 99));

    // middle in X
    BOOST_CHECK_EQUAL(grid.toGrid(Vector3d(5.05, 49.75, 0.5), idx, frame_in_grid), true);
    BOOST_CHECK_EQUAL(idx, Index(50, 99));

    // middle in Y
    BOOST_CHECK_EQUAL(grid.toGrid(Vector3d(9.95, 25.25, 0.5), idx, frame_in_grid), true);
    BOOST_CHECK_EQUAL(idx, Index(99, 50));

    // middle in X and Y
    BOOST_CHECK_EQUAL(grid.toGrid(Vector3d(5.05, 25.25, 0.5), idx, frame_in_grid), true);
    BOOST_CHECK_EQUAL(idx, Index(50, 50));

    idx.setZero();

    // outside: pos should be unchanged
    BOOST_CHECK_EQUAL(grid.toGrid(Vector3d(5, 50, 0.5), idx, frame_in_grid), false);
    BOOST_CHECK_EQUAL(idx, Index(0, 0));

    BOOST_CHECK_EQUAL(grid.toGrid(Vector3d(10, 25, 0.5), idx, frame_in_grid), false);
    BOOST_CHECK_EQUAL(idx, Index(0, 0));

    BOOST_CHECK_EQUAL(grid.toGrid(Vector3d(10, 50, 0.5), idx, frame_in_grid), false);
    BOOST_CHECK_EQUAL(idx, Index(0, 0));

}
