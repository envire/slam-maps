#include <boost/test/unit_test.hpp>
#include <envire_maps/GridMap.hpp>

#include <boost/math/special_functions/fpclassify.hpp>

#include <chrono>

using namespace envire::maps;

double default_value = std::numeric_limits<double>::infinity();

BOOST_AUTO_TEST_CASE(test_grid_empty)
{
    GridMap<double> *grid = new GridMap<double>();

    // if the grid has size of (0,0)
    BOOST_CHECK_THROW(grid->at(Index(0,0)), std::exception);
    BOOST_CHECK_THROW(grid->getMax(), std::exception);  
    BOOST_CHECK_THROW(grid->getMin(), std::exception);      
    BOOST_CHECK_THROW(grid->clear(), std::exception);       
    BOOST_CHECK_THROW(grid->moveBy(Eigen::Vector2i(0,0)), std::exception);          

    delete grid;
}


BOOST_AUTO_TEST_CASE(test_grid_reset)
{
    Vector2ui num_cells(100, 200);
    Vector2d resolution(0.1, 0.5);
    GridMap<double> *grid = new GridMap<double>();

    grid->reset(num_cells, resolution, default_value);

    // check configuration
    BOOST_CHECK_EQUAL(grid->getNumCells(), num_cells);
    BOOST_CHECK_EQUAL(grid->getResolution(), resolution);
    BOOST_CHECK_EQUAL(grid->getDefaultValue(), default_value);

    Vector2ui new_num_cells(50, 50);
    Vector2d new_resolution(0.1, 0.1);
    double new_default_value = std::numeric_limits<double>::quiet_NaN();

    grid->reset(new_num_cells, new_resolution, new_default_value);

    // check configuration
    BOOST_CHECK_EQUAL(grid->getNumCells(), new_num_cells);
    BOOST_CHECK_EQUAL(grid->getResolution(), new_resolution);
    BOOST_CHECK_EQUAL( boost::math::isnan(grid->getDefaultValue()), true);


    delete grid;
}



BOOST_AUTO_TEST_CASE(test_grid_copy)
{
    Vector2ui num_cells(100, 200);
    Vector2d resolution(0.1, 0.5);

    /* Grid map of 10x100 meters **/
    GridMap<double> *grid = new GridMap<double>(num_cells, resolution, default_value);
    grid->localFrame().translate(Eigen::Vector3d::Random(3));

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
    BOOST_CHECK_EQUAL(grid_copy->localFrame().translation(), grid->localFrame().translation());    
    BOOST_CHECK_EQUAL(grid_copy->localFrame().rotation(), grid->localFrame().rotation());    

    // check default value
    BOOST_CHECK_EQUAL(grid_copy->getDefaultValue(), grid->getDefaultValue());

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

    // check default valuegitk --all

    BOOST_CHECK_EQUAL(grid->getDefaultValue(), std::numeric_limits<double>::infinity());

    // access cell that is not in grid = > should throw error
    BOOST_CHECK_THROW(grid->at(Index(num_cells.x(), num_cells.y())), std::exception);
    BOOST_CHECK_THROW(grid->at(Vector3d(grid->getSize().x(), grid->getSize().y(), 0)), std::exception);
    BOOST_CHECK_THROW(grid->at(num_cells.x(), num_cells.y()), std::exception);

    // access cell that is in grid
    BOOST_CHECK_EQUAL(grid->at(Index(num_cells.x() - 1, num_cells.y() - 1)), std::numeric_limits<double>::infinity());
    BOOST_CHECK_EQUAL(grid->at(Vector3d(grid->getSize().x() - resolution.x() - 0.01,
                                        grid->getSize().y() - resolution.y() - 0.01,
                                        0)), 
                      std::numeric_limits<double>::infinity());
    BOOST_CHECK_EQUAL(grid->at(num_cells.x() - 1, num_cells.y() - 1), std::numeric_limits<double>::infinity());

    GridMap<double> const* grid_const = grid;

    // access cell that is not in grid = > should throw error
    BOOST_CHECK_THROW(grid_const->at(Index(num_cells.x(), num_cells.y())), std::exception); 
    BOOST_CHECK_THROW(grid_const->at(Vector3d(grid->getSize().x(), grid->getSize().y(), 0)), std::exception);
    BOOST_CHECK_THROW(grid_const->at(num_cells.x(), num_cells.y()), std::exception);

    // access cell that is in grid
    BOOST_CHECK_EQUAL(grid_const->at(Index(num_cells.x() - 1, num_cells.y() - 1)), std::numeric_limits<double>::infinity());
    BOOST_CHECK_EQUAL(grid_const->at(Vector3d(grid->getSize().x() - resolution.x() - 0.01,
                                              grid->getSize().y() - resolution.y() - 0.01,
                                              0)),
                      std::numeric_limits<double>::infinity());
    BOOST_CHECK_EQUAL(grid_const->at(num_cells.x() - 1, num_cells.y() - 1), std::numeric_limits<double>::infinity());

    //delete grid;
    //delete grid_const;
}

BOOST_AUTO_TEST_CASE(test_grid_minmax)
{
    Vector2ui num_cells(100, 200);
    Vector2d resolution(0.1, 0.5);

    GridMap<double> *grid = new GridMap<double>(num_cells, resolution, default_value); 

    double min = std::numeric_limits<double>::max();
    double max = std::numeric_limits<double>::min();
    for (unsigned int x = 0; x < grid->getNumCells().x(); ++x)
    {
        for (unsigned int y = 0; y < grid->getNumCells().y(); ++y)
        {
            double cell_value = rand();

            if (min > cell_value)
                min = cell_value;
            if (max < cell_value)
                max = cell_value;

            grid->at(x, y) = cell_value;
        }
    }

    BOOST_CHECK_CLOSE(grid->getMax(), max, 0.000001);
    BOOST_CHECK_CLOSE(grid->getMin(), min, 0.000001);

    delete grid;
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
            BOOST_CHECK_EQUAL((grid->at(x, y) != grid->getDefaultValue()), true);
        }
    }

    grid->clear();

    // after the clean all cell should have default value
    for (unsigned int x = 0; x < grid->getNumCells().x(); ++x)
    {
        for (unsigned int y = 0; y < grid->getNumCells().y(); ++y)
        {
            BOOST_CHECK_EQUAL(grid->at(x, y), grid->getDefaultValue());
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
            BOOST_CHECK_EQUAL(grid->at(Index(x,y)), grid->getDefaultValue());
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
            BOOST_CHECK_EQUAL(grid->at(Index(x,y)), grid->getDefaultValue());
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
            BOOST_CHECK_EQUAL(grid->at(Index(x,y)), grid->getDefaultValue());
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
            BOOST_CHECK_EQUAL(grid->at(Index(x,y)), grid->getDefaultValue());
        }
    }


    // -------------- Test 5: move all values out of grid

    grid->moveBy(Eigen::Vector2i(-7,0));

    // in grid should be only default values
    for (unsigned int x = 0; x < grid->getNumCells().x(); ++x)
    {
        for (unsigned int y = 0; y < grid->getNumCells().y(); ++y)
        {
            BOOST_CHECK_EQUAL(grid->at(Index(x,y)), grid->getDefaultValue());
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
            BOOST_CHECK_EQUAL(grid->at(Index(x,y)), grid->getDefaultValue());
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
                BOOST_CHECK_EQUAL(grid->at(Index(x,y)), grid->getDefaultValue());
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
                BOOST_CHECK_EQUAL(grid->at(Index(x,y)), grid->getDefaultValue());
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
                BOOST_CHECK_EQUAL(grid->at(Index(x,y)), grid->getDefaultValue());
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
                BOOST_CHECK_EQUAL(grid->at(Index(x,y)), grid->getDefaultValue());
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
                BOOST_CHECK_EQUAL(grid->at(Index(x,y)), grid->getDefaultValue());
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
                BOOST_CHECK_EQUAL(grid->at(Index(x,y)), grid->getDefaultValue());
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
