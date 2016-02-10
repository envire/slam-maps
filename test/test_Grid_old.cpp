#include <boost/test/unit_test.hpp>
#include <envire_maps/Grid.hpp>

#include <chrono>

using namespace envire::maps;
/*// the grid 20m x 50m
//size_t cellSizeX = 10000;
//size_t cellSizeY = 10000;

// cell size 0.2m x 0.5m
double scaleX = 1;
double scaleY = 1;

// the grid center is 20m x 50m
double offsetX = 0;
double offsetY = 0;

//GridConfig config(cellSizeX, cellSizeY, scaleX, scaleY, offsetX, offsetY);

double default_value = std::numeric_limits<double>::infinity();

BOOST_AUTO_TEST_CASE(test_grid_empty)
{
	Grid<double> *grid = new Grid<double>();

	// if the grid has size of (0,0)
	BOOST_CHECK_THROW(grid->at(GridBase::Index(0,0)), std::exception);
	BOOST_CHECK_THROW(grid->getMax(), std::exception);	
	BOOST_CHECK_THROW(grid->getMin(), std::exception);		
	BOOST_CHECK_THROW(grid->clear(), std::exception);		
	BOOST_CHECK_THROW(grid->moveBy(GridBase::Index(0,0)), std::exception);			

	delete grid;
}

BOOST_AUTO_TEST_CASE(test_grid_cell_access)
{
	double cellSizeX = 5;
	double cellSizeY = 7;

	GridConfig config_t(cellSizeX, cellSizeY, scaleX, scaleY, offsetX, offsetY);
	Grid<double> *grid = new Grid<double>(default_value, config_t);	

	// check default valuegitk --all

	BOOST_CHECK_EQUAL(grid->getDefaultValue(), std::numeric_limits<double>::infinity());

	// access cell that is not in grid = > should throw error
	BOOST_CHECK_THROW(grid->at(GridBase::Index(cellSizeX + 1, cellSizeY + 1)), std::exception);	
	BOOST_CHECK_THROW(grid->at(Eigen::Vector2d(cellSizeX + 1, cellSizeY + 1)), std::exception);		
	BOOST_CHECK_THROW(grid->at(cellSizeX + 1, cellSizeY + 1), std::exception);			

	// access cell that is in grid
	BOOST_CHECK_EQUAL(grid->at(GridBase::Index(cellSizeX - 1, cellSizeY - 1)), std::numeric_limits<double>::infinity());	
	BOOST_CHECK_EQUAL(grid->at(Eigen::Vector2d(cellSizeX - 1, cellSizeY - 1)), std::numeric_limits<double>::infinity());		
	BOOST_CHECK_EQUAL(grid->at(cellSizeX - 1, cellSizeY - 1), std::numeric_limits<double>::infinity());			

	Grid<double> const* grid_const = grid;

	// access cell that is not in grid = > should throw error
	BOOST_CHECK_THROW(grid_const->at(GridBase::Index(cellSizeX + 1, cellSizeY + 1)), std::exception);	
	BOOST_CHECK_THROW(grid_const->at(Eigen::Vector2d(cellSizeX + 1, cellSizeY + 1)), std::exception);		
	BOOST_CHECK_THROW(grid_const->at(cellSizeX + 1, cellSizeY + 1), std::exception);			

	// access cell that is in grid
	BOOST_CHECK_EQUAL(grid_const->at(GridBase::Index(cellSizeX - 1, cellSizeY - 1)), std::numeric_limits<double>::infinity());	
	BOOST_CHECK_EQUAL(grid_const->at(Eigen::Vector2d(cellSizeX - 1, cellSizeY - 1)), std::numeric_limits<double>::infinity());		
	BOOST_CHECK_EQUAL(grid_const->at(cellSizeX - 1, cellSizeY - 1), std::numeric_limits<double>::infinity());			

	delete grid;
}

BOOST_AUTO_TEST_CASE(test_grid_minmax)
{
	double cellSizeX = 5;
	double cellSizeY = 7;

	GridConfig config_t(cellSizeX, cellSizeY, scaleX, scaleY, offsetX, offsetY);
	Grid<double> *grid = new Grid<double>(default_value, config_t);	

	double min = std::numeric_limits<double>::max();
	double max = std::numeric_limits<double>::min();
	for (unsigned int x = 0; x < grid->getCellSizeX(); ++x)
	{
		for (unsigned int y = 0; y < grid->getCellSizeY(); ++y)
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
	double cellSizeX = 5;
	double cellSizeY = 7;

	GridConfig config_t(cellSizeX, cellSizeY, scaleX, scaleY, offsetX, offsetY);
	Grid<double> *grid = new Grid<double>(default_value, config_t);	

	for (unsigned int x = 0; x < grid->getCellSizeX(); ++x)
	{
		for (unsigned int y = 0; y < grid->getCellSizeY(); ++y)
		{
			double cell_value = rand();
			grid->at(x, y) = cell_value;
		}
	}

	// check if all cell are set 
	for (unsigned int x = 0; x < grid->getCellSizeX(); ++x)
	{
		for (unsigned int y = 0; y < grid->getCellSizeY(); ++y)
		{
			BOOST_CHECK_EQUAL((grid->at(x, y) != grid->getDefaultValue()), true);
		}
	}		

	grid->clear();

	// after the clean all cell should have default value
	for (unsigned int x = 0; x < grid->getCellSizeX(); ++x)
	{
		for (unsigned int y = 0; y < grid->getCellSizeY(); ++y)
		{
			BOOST_CHECK_EQUAL(grid->at(x, y), grid->getDefaultValue());
		}
	}	

	delete grid;
}

BOOST_AUTO_TEST_CASE(test_grid_copy)
{
	double cellSizeX = 5;
	double cellSizeY = 7;

	GridConfig config_t(cellSizeX, cellSizeY, scaleX, scaleY, offsetX, offsetY);
	Grid<double> *grid = new Grid<double>(default_value, config_t);	

	for (unsigned int x = 0; x < grid->getCellSizeX(); ++x)
	{
		for (unsigned int y = 0; y < grid->getCellSizeY(); ++y)
		{
			double cell_value = rand();
			grid->at(x, y) = cell_value;
		}
	}

	Grid<double> *grid_copy = new Grid<double>(*grid);

	// check configuration
	BOOST_CHECK_EQUAL(grid_copy->getCellSizeX(), grid->getCellSizeX());	
	BOOST_CHECK_EQUAL(grid_copy->getCellSizeY(), grid->getCellSizeY());	
	BOOST_CHECK_EQUAL(grid_copy->getScaleX(), grid->getScaleX());		
	BOOST_CHECK_EQUAL(grid_copy->getScaleY(), grid->getScaleY());	
	BOOST_CHECK_EQUAL(grid_copy->localFrameX(), grid->localFrameX());	
	BOOST_CHECK_EQUAL(grid_copy->localFrameY(), grid->localFrameY());		

	// check default value
	BOOST_CHECK_EQUAL(grid_copy->getDefaultValue(), grid->getDefaultValue());

	// check cell value
	for (unsigned int x = 0; x < grid->getCellSizeX(); ++x)
	{
		for (unsigned int y = 0; y < grid->getCellSizeY(); ++y)
		{
			BOOST_CHECK_EQUAL(grid->at(x, y), grid_copy->at(x,y));
		}
	}		
}

BOOST_AUTO_TEST_CASE(test_grid_move)
{
	GridConfig config_t(5, 7, scaleX, scaleY, offsetX, offsetY);

	Grid<double> *grid = new Grid<double>(default_value, config_t);	

	// -------------- Test 1: move all values out of grid

	grid->moveBy(GridBase::Index(5,7));

	for (unsigned int x = 0; x < grid->getCellSizeX(); ++x)
	{
		for (unsigned int y = 0; y < grid->getCellSizeY(); ++y)
		{
			BOOST_CHECK_EQUAL(grid->at(GridBase::Index(x,y)), grid->getDefaultValue());
		}
	}

	// -------------- Test 2: move by Index(-2,-3);

	// 0 0 0 0 0 0 0 
	// 1 1 1 1 1 1 1 
	// 2 2 2 2 2 2 2 
	// 3 3 3 3 3 3 3 
	// 4 4 4 4 4 4 4 
	for (unsigned int x = 0; x < grid->getCellSizeX(); ++x)
	{
		for (unsigned int y = 0; y < grid->getCellSizeY(); ++y)
		{
			grid->at(GridBase::Index(x,y)) = x;
		}
	}

	grid->moveBy(GridBase::Index(-2, -3));

	// 2 2 2 2 inf inf inf 
	// 3 3 3 3 inf inf inf 
	// 4 4 4 4 inf inf inf 
	// inf inf inf inf inf inf inf 
	// inf inf inf inf inf inf inf 
	for (unsigned int x = 0; x < grid->getCellSizeX(); ++x)
	{
		for (unsigned int y = 0; y < grid->getCellSizeY(); ++y)
		{
			if (y < (grid->getCellSizeY() - 3) && x < (grid->getCellSizeX() - 2))
				BOOST_CHECK_EQUAL(grid->at(GridBase::Index(x,y)), x + 2);
			else
				BOOST_CHECK_EQUAL(grid->at(GridBase::Index(x,y)), grid->getDefaultValue());
		}
	}

	delete grid;
}

*/