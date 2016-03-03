#include <boost/test/unit_test.hpp>
#include <envire_maps/GridMap.hpp>

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
    BOOST_CHECK_NO_THROW(grid->clear());
    BOOST_CHECK_NO_THROW(grid->moveBy(Index(0,0)));

    delete grid;
}

BOOST_AUTO_TEST_CASE(test_grid_copy)
{
    Vector2ui num_cells(100, 200);
    Vector2d resolution(0.1, 0.5);

    GridMap<double> *grid = new GridMap<double>(num_cells, resolution, default_value); 
    grid->getOffset().translate(Eigen::Vector3d::Random(3));

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
    BOOST_CHECK_EQUAL(grid_copy->getOffset().translation(), grid->getOffset().translation());    
    BOOST_CHECK_EQUAL(grid_copy->getOffset().rotation(), grid->getOffset().rotation());    

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

BOOST_AUTO_TEST_CASE(test_grid_move)
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

    grid->moveBy(Index(7,5));

    // in grid should be only default values
    for (unsigned int x = 0; x < grid->getNumCells().x(); ++x)
    {
        for (unsigned int y = 0; y < grid->getNumCells().y(); ++y)
        {
            BOOST_CHECK_EQUAL(grid->at(Index(x,y)), grid->getDefaultValue());
        }
    }

    // -------------- Test 2: move by Index(-2,-3);

    // 0 1 2 3 4 5 6 
    // 0 1 2 3 4 5 6 
    // 0 1 2 3 4 5 6 
    // 0 1 2 3 4 5 6 
    // 0 1 2 3 4 5 6 
    for (unsigned int y = 0; y < grid->getNumCells().y(); ++y)
    {
        for (unsigned int x = 0; x < grid->getNumCells().x(); ++x)
        {
            grid->at(Index(x,y)) = x;
        }
    }

    grid->moveBy(Index(-2, -3)); 

    // 2 3 4 5 6 inf inf 
    // 2 3 4 5 6 inf inf 
    // inf inf inf inf inf inf inf 
    // inf inf inf inf inf inf inf 
    // inf inf inf inf inf inf inf 
    for (unsigned int y = 0; y < grid->getNumCells().y(); ++y)
    {
        for (unsigned int x = 0; x < grid->getNumCells().x(); ++x)
        {
            if (y < (grid->getNumCells().y() - 3) && x < (grid->getNumCells().x() - 2))
                BOOST_CHECK_EQUAL(grid->at(Index(x,y)), x + 2);
            else
                BOOST_CHECK_EQUAL(grid->at(Index(x,y)), grid->getDefaultValue());
        }
    }

    delete grid;
}
