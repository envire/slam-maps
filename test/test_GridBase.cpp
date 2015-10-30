#include <boost/test/unit_test.hpp>
#include <envire_maps/GridBase.hpp>

#include <chrono>

using namespace envire::maps;

BOOST_AUTO_TEST_CASE(test_grid_index)
{
    // check default constructor
    GridBase::Index index;
    BOOST_CHECK_EQUAL(index.x, 0);
    BOOST_CHECK_EQUAL(index.y, 0);

    // check constructor with parameter
    GridBase::Index index2(2, 4);
    BOOST_CHECK_EQUAL(index2.x, 2);
    BOOST_CHECK_EQUAL(index2.y, 4);         

    // check the "<" operator
    BOOST_CHECK_EQUAL((index2 < GridBase::Index(index2.x + 1, index2.y + 1)), true);  
    BOOST_CHECK_EQUAL((index2 < GridBase::Index(index2.x, index2.y)), false);       
    BOOST_CHECK_EQUAL((index2 < GridBase::Index(index2.x, index2.y + 1)), true);    
    BOOST_CHECK_EQUAL((index2 < GridBase::Index(index2.x, index2.y - 1)), false); 
    BOOST_CHECK_EQUAL((index2 < GridBase::Index(index2.x - 1, index2.y + 1)), false);     
    BOOST_CHECK_EQUAL((index2 < GridBase::Index(index2.x - 1, index2.y - 1)), false);     

    // check the "==" operator
    BOOST_CHECK_EQUAL((index2 == GridBase::Index(index2.x, index2.y)), true);
    BOOST_CHECK_EQUAL((index2 == GridBase::Index(index2.x, index2.y + 1)), false);        
    BOOST_CHECK_EQUAL((index2 == GridBase::Index(index2.x + 1, index2.y)), false);    
    BOOST_CHECK_EQUAL((index2 == GridBase::Index(index2.x + 1, index2.y + 1)), false);    

    // check the "!=" operator
    BOOST_CHECK_EQUAL((index2 != GridBase::Index(index2.x, index2.y)), false);
    BOOST_CHECK_EQUAL((index2 != GridBase::Index(index2.x, index2.y + 1)), true);     
    BOOST_CHECK_EQUAL((index2 != GridBase::Index(index2.x + 1, index2.y)), true); 
    BOOST_CHECK_EQUAL((index2 != GridBase::Index(index2.x + 1, index2.y + 1)), true);     

    // check the sum operator
    BOOST_CHECK((index2 + GridBase::Index(3, 5)) == GridBase::Index(5, 9));     

    // TODO: check otherwise => it will not be 3,5
    // check the diff operator
    BOOST_CHECK((GridBase::Index(5, 9) - index2) == GridBase::Index(3, 5));     
}

// the grid 100 x 200 (10m x 100m)
size_t cellSizeX = 100;
size_t cellSizeY = 200;

// cell size 0.1m x 0.5m
double scaleX = 0.1;
double scaleY = 0.5;

// the grid center is -5m x -50m
double offsetX = -5;
double offsetY = -50;

BOOST_AUTO_TEST_CASE(test_gridbase_default_constructor)
{
    GridBase grid_base;
    BOOST_CHECK_EQUAL(grid_base.getCellSizeX(), 0);
    BOOST_CHECK_EQUAL(grid_base.getCellSizeY(), 0);
    BOOST_CHECK_EQUAL(grid_base.getScaleX(), 0);
    BOOST_CHECK_EQUAL(grid_base.getScaleY(), 0);
    BOOST_CHECK_EQUAL(grid_base.getOffsetX(), 0);
    BOOST_CHECK_EQUAL(grid_base.getOffsetY(), 0); 
    BOOST_CHECK_EQUAL(grid_base.getSizeX(), 0);
    BOOST_CHECK_EQUAL(grid_base.getSizeY(), 0); 

    BOOST_CHECK_EQUAL(grid_base.inGrid(GridBase::Index(0, 0)), false);   

    Eigen::Vector2d pos;    
    BOOST_CHECK_EQUAL(grid_base.fromGrid(GridBase::Index(0, 0), pos), false);       

    GridBase::Index idx;    
    BOOST_CHECK_EQUAL(grid_base.toGrid(Eigen::Vector2d(0, 0), idx), false);           

}

BOOST_AUTO_TEST_CASE(test_gridbase_default_param_constructor)
{
    GridConfig config(cellSizeX, cellSizeY, scaleX, scaleY, offsetX, offsetY);  
    GridBase grid_base(config);

    BOOST_CHECK_EQUAL(grid_base.getCellSizeX(), cellSizeX);
    BOOST_CHECK_EQUAL(grid_base.getCellSizeY(), cellSizeY);
    BOOST_CHECK_EQUAL(grid_base.getScaleX(), scaleX);
    BOOST_CHECK_EQUAL(grid_base.getScaleY(), scaleY);
    BOOST_CHECK_EQUAL(grid_base.getOffsetX(), offsetX);
    BOOST_CHECK_EQUAL(grid_base.getOffsetY(), offsetY); 
    BOOST_CHECK_EQUAL(grid_base.getSizeX(), 10);
    BOOST_CHECK_EQUAL(grid_base.getSizeY(), 100);

    const GridConfig &config_t = grid_base.getGridConfig();
    BOOST_CHECK_EQUAL(config.cellSizeX, config_t.cellSizeX);
    BOOST_CHECK_EQUAL(config.cellSizeY, config_t.cellSizeY);
    BOOST_CHECK_EQUAL(config.scaleX, config_t.scaleX);
    BOOST_CHECK_EQUAL(config.scaleY, config_t.scaleY);
    BOOST_CHECK_EQUAL(config.offsetX, config_t.offsetX);
    BOOST_CHECK_EQUAL(config.offsetY, config_t.offsetY); 
}

BOOST_AUTO_TEST_CASE(test_gridbase_in_grid)
{
    GridConfig config(cellSizeX, cellSizeY, scaleX, scaleY, offsetX, offsetY);  
    GridBase grid_base(config); 

    // ---- Index In Grid ---- 
    // bottom right
    BOOST_CHECK_EQUAL(grid_base.inGrid(GridBase::Index(0, 0)), true);   

    // top left
    BOOST_CHECK_EQUAL(grid_base.inGrid(GridBase::Index(99, 199)), true);   

    // middle
    BOOST_CHECK_EQUAL(grid_base.inGrid(GridBase::Index(50, 100)), true);    

    // outside
    BOOST_CHECK_EQUAL(grid_base.inGrid(GridBase::Index(100, 200)), false);     
}

BOOST_AUTO_TEST_CASE(test_gridbase_index2pos)
{
    GridConfig config(cellSizeX, cellSizeY, scaleX, scaleY, offsetX, offsetY);  
    GridBase grid_base(config); 

    // ---- Index 2 Position ---- 

    Eigen::Vector2d pos;

    // bottom right
    BOOST_CHECK_EQUAL(grid_base.fromGrid(GridBase::Index(0, 0), pos), true);
    BOOST_CHECK_CLOSE(pos.x(), -4.95, 0.0001);
    BOOST_CHECK_CLOSE(pos.y(), -49.75, 0.0001); 

    // top left
    BOOST_CHECK_EQUAL(grid_base.fromGrid(GridBase::Index(99, 199), pos), true);
    BOOST_CHECK_CLOSE(pos.x(), 4.95, 0.0001);
    BOOST_CHECK_CLOSE(pos.y(), 49.75, 0.0001);      

    // middle
    BOOST_CHECK_EQUAL(grid_base.fromGrid(GridBase::Index(50, 100), pos), true);
    BOOST_CHECK_CLOSE(pos.x(), 0.05, 0.0001);
    BOOST_CHECK_CLOSE(pos.y(), 0.25, 0.0001);

    // outside: pos should be unchanged
    BOOST_CHECK_EQUAL(grid_base.fromGrid(GridBase::Index(100, 200), pos), false);   
    BOOST_CHECK_CLOSE(pos.x(), 0.05, 0.0001);
    BOOST_CHECK_CLOSE(pos.y(), 0.25, 0.0001);   
}

BOOST_AUTO_TEST_CASE(test_gridbase_index2pos_in_frame)
{
    GridConfig config(cellSizeX, cellSizeY, scaleX, scaleY, offsetX, offsetY);  
    GridBase grid_base(config); 

    // ---- Index 2 Position ---- 

    Eigen::Vector3d pos;

    Eigen::Affine3d frame_in_grid(Eigen::Translation3d( 0.5, 1.7, -0.5 ));

    // bottom right
    BOOST_CHECK_EQUAL(grid_base.fromGrid(GridBase::Index(0, 0), pos, frame_in_grid), true);
    BOOST_CHECK_CLOSE(pos.x(), -5.45, 0.0001);
    BOOST_CHECK_CLOSE(pos.y(), -51.45, 0.0001); 
    BOOST_CHECK_CLOSE(pos.z(), 0.5, 0.0001); 

    // top left
    BOOST_CHECK_EQUAL(grid_base.fromGrid(GridBase::Index(99, 199), pos, frame_in_grid), true);
    BOOST_CHECK_CLOSE(pos.x(), 4.45, 0.0001);
    BOOST_CHECK_CLOSE(pos.y(), 48.05, 0.0001); 
    BOOST_CHECK_CLOSE(pos.z(), 0.5, 0.0001);          

    // middle
    BOOST_CHECK_EQUAL(grid_base.fromGrid(GridBase::Index(50, 100), pos, frame_in_grid), true);
    BOOST_CHECK_CLOSE(pos.x(), -0.45, 0.0001);
    BOOST_CHECK_CLOSE(pos.y(), -1.45, 0.0001);
    BOOST_CHECK_CLOSE(pos.z(), 0.5, 0.0001);     

    // outside: pos should be unchanged
    BOOST_CHECK_EQUAL(grid_base.fromGrid(GridBase::Index(100, 200), pos, frame_in_grid), false);   
    BOOST_CHECK_CLOSE(pos.x(), -0.45, 0.0001);
    BOOST_CHECK_CLOSE(pos.y(), -1.45, 0.0001);
    BOOST_CHECK_CLOSE(pos.z(), 0.5, 0.0001);       
}

BOOST_AUTO_TEST_CASE(test_gridbase_pos2index)
{
    GridConfig config(cellSizeX, cellSizeY, scaleX, scaleY, offsetX, offsetY);  
    GridBase grid_base(config);     

    // ---- Position 2 Index ---- 

    GridBase::Index idx;
    Eigen::Vector2d pos_diff;

    // bottom right
    BOOST_CHECK_EQUAL(grid_base.toGrid(Eigen::Vector2d(-4.95, -49.75), idx, pos_diff), true);
    BOOST_CHECK_EQUAL(idx.x, 0);
    BOOST_CHECK_EQUAL(idx.y, 0);
    BOOST_CHECK_CLOSE(pos_diff.x(), 0.05, 0.0001);
    BOOST_CHECK_CLOSE(pos_diff.y(), 0.25, 0.0001);

    BOOST_CHECK_EQUAL(grid_base.toGrid(Eigen::Vector2d(-4.95, -49.75), idx), true);
    BOOST_CHECK_EQUAL(idx.x, 0);
    BOOST_CHECK_EQUAL(idx.y, 0);    

    // top left
    BOOST_CHECK_EQUAL(grid_base.toGrid(Eigen::Vector2d(4.95, 49.75), idx, pos_diff), true);
    BOOST_CHECK_EQUAL(idx.x, 99);
    BOOST_CHECK_EQUAL(idx.y, 199);
    BOOST_CHECK_CLOSE(pos_diff.x(), 0.05, 0.0001);
    BOOST_CHECK_CLOSE(pos_diff.y(), 0.25, 0.0001);  

    BOOST_CHECK_EQUAL(grid_base.toGrid(Eigen::Vector2d(4.95, 49.75), idx), true);
    BOOST_CHECK_EQUAL(idx.x, 99);
    BOOST_CHECK_EQUAL(idx.y, 199);     

    // middle
    BOOST_CHECK_EQUAL(grid_base.toGrid(Eigen::Vector2d(0, 0), idx, pos_diff), true);
    BOOST_CHECK_EQUAL(idx.x, 50);
    BOOST_CHECK_EQUAL(idx.y, 100);
    BOOST_CHECK_CLOSE(pos_diff.x(), 0.0, 0.0001);
    BOOST_CHECK_CLOSE(pos_diff.y(), 0.0, 0.0001);   

    BOOST_CHECK_EQUAL(grid_base.toGrid(Eigen::Vector2d(0, 0), idx), true);
    BOOST_CHECK_EQUAL(idx.x, 50);
    BOOST_CHECK_EQUAL(idx.y, 100);    

    // outside: the index and pos_diff should be unchanged
    BOOST_CHECK_EQUAL(grid_base.toGrid(Eigen::Vector2d(5, 50), idx, pos_diff), false);
    BOOST_CHECK_EQUAL(idx.x, 50);
    BOOST_CHECK_EQUAL(idx.y, 100);     
    BOOST_CHECK_CLOSE(pos_diff.x(), 0.0, 0.0001);
    BOOST_CHECK_CLOSE(pos_diff.y(), 0.0, 0.0001);   

    BOOST_CHECK_EQUAL(grid_base.toGrid(Eigen::Vector2d(5, 50), idx), false);    
    BOOST_CHECK_EQUAL(idx.x, 50);
    BOOST_CHECK_EQUAL(idx.y, 100);       
}

BOOST_AUTO_TEST_CASE(test_gridbase_pos2index_in_frame)
{
    GridConfig config(cellSizeX, cellSizeY, scaleX, scaleY, offsetX, offsetY);  
    GridBase grid_base(config);     

    // ---- Position 2 Index ---- 

    GridBase::Index idx;
    Eigen::Vector2d pos_diff;

    // 30, 45, -30
    //Eigen::Quaterniond orientation(0.8876262680160252, -0.13529902503654923, -0.3266407412190941, 0.2951603095403302);
    //Eigen::Translation3d translation(0.5, 1.7, -0.5);
    //Eigen::Affine3d frame_in_grid(translation * orientation);  

    Eigen::Affine3d frame_in_grid(Eigen::Translation3d( 0.5, 1.7, -0.5 ));

    // bottom right
    BOOST_CHECK_EQUAL(grid_base.toGrid(Eigen::Vector3d(-5.45, -51.45, 0.5), idx, frame_in_grid), true);
    BOOST_CHECK_EQUAL(idx.x, 0);
    BOOST_CHECK_EQUAL(idx.y, 0);    

    // top left
    BOOST_CHECK_EQUAL(grid_base.toGrid(Eigen::Vector3d(4.45, 48.05, 0.5), idx, frame_in_grid), true);
    BOOST_CHECK_EQUAL(idx.x, 99);
    BOOST_CHECK_EQUAL(idx.y, 199);     

    // middle
    BOOST_CHECK_EQUAL(grid_base.toGrid(Eigen::Vector3d(-0.45, -1.45, 0.5), idx, frame_in_grid), true);
    BOOST_CHECK_EQUAL(idx.x, 50);
    BOOST_CHECK_EQUAL(idx.y, 100);    

    // outside: the index should be unchanged
    BOOST_CHECK_EQUAL(grid_base.toGrid(Eigen::Vector3d(20, 150, 0.5), idx, frame_in_grid), false);    
    BOOST_CHECK_EQUAL(idx.x, 50);
    BOOST_CHECK_EQUAL(idx.y, 100);      
}



