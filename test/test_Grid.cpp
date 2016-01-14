#include <boost/test/unit_test.hpp>
#include <envire_maps/Grid.hpp>

#include <iostream>

using namespace envire::maps;

BOOST_AUTO_TEST_CASE(test_grid_index)
{
    // check default constructor
    Index index;
    BOOST_CHECK_EQUAL(index.x(), 0);
    BOOST_CHECK_EQUAL(index.y(), 0);

    // check constructor with parameter
    Index index2(2, 4);
    BOOST_CHECK_EQUAL(index2.x(), 2);
    BOOST_CHECK_EQUAL(index2.y(), 4);         

    // check the "<" operator
    BOOST_CHECK_EQUAL((index2 < Index(index2.x() + 1, index2.y() + 1)), true);  
    BOOST_CHECK_EQUAL((index2 < Index(index2.x(), index2.y())), false);       
    BOOST_CHECK_EQUAL((index2 < Index(index2.x(), index2.y() + 1)), true);    
    BOOST_CHECK_EQUAL((index2 < Index(index2.x(), index2.y() - 1)), false); 
    BOOST_CHECK_EQUAL((index2 < Index(index2.x() - 1, index2.y() + 1)), false);     
    BOOST_CHECK_EQUAL((index2 < Index(index2.x() - 1, index2.y() - 1)), false);   

    // check the ">" operator
    BOOST_CHECK_EQUAL((index2 > Index(index2.x() - 1, index2.y() - 1)), true);  
    BOOST_CHECK_EQUAL((index2 > Index(index2.x(), index2.y())), false);       
    BOOST_CHECK_EQUAL((index2 > Index(index2.x(), index2.y() - 1)), true);    
    BOOST_CHECK_EQUAL((index2 > Index(index2.x(), index2.y() + 1)), false); 
    BOOST_CHECK_EQUAL((index2 > Index(index2.x() + 1, index2.y() - 1)), false);     
    BOOST_CHECK_EQUAL((index2 > Index(index2.x() + 1, index2.y() + 1)), false);        

    // check the "==" operator
    BOOST_CHECK_EQUAL((index2 == Index(index2.x(), index2.y())), true);
    BOOST_CHECK_EQUAL((index2 == Index(index2.x(), index2.y() + 1)), false);        
    BOOST_CHECK_EQUAL((index2 == Index(index2.x() + 1, index2.y())), false);    
    BOOST_CHECK_EQUAL((index2 == Index(index2.x() + 1, index2.y() + 1)), false);    

    // check the "!=" operator
    BOOST_CHECK_EQUAL((index2 != Index(index2.x(), index2.y())), false);
    BOOST_CHECK_EQUAL((index2 != Index(index2.x(), index2.y() + 1)), true);     
    BOOST_CHECK_EQUAL((index2 != Index(index2.x() + 1, index2.y())), true); 
    BOOST_CHECK_EQUAL((index2 != Index(index2.x() + 1, index2.y() + 1)), true);     

    // check the sum operator
    BOOST_CHECK((index2 + Index(3, 5)) == Index(5, 9));     

    BOOST_CHECK((Index(5, 9) - index2) == Index(3, 5));  

    // Implement!
    //BOOST_CHECK((index2 - Index(5, 9)) == Index(3, 5)); 

    Index index3(Eigen::Matrix<unsigned int, 2, 1>(3, 4));
    BOOST_CHECK_EQUAL(index3.x(), 3);
    BOOST_CHECK_EQUAL(index3.y(), 4);
}

BOOST_AUTO_TEST_CASE(test_grid_constructor_default)
{
    Grid grid;

    BOOST_CHECK_EQUAL(grid.getNumCells(), Vector2ui(0, 0));
    BOOST_CHECK_EQUAL(grid.getResolution(), Vector2d(0, 0));    
    BOOST_CHECK_EQUAL(grid.getSize(), Vector2d(0, 0));      

    BOOST_CHECK_EQUAL(grid.inGrid(Index(0, 0)), false);  

    Vector3d pos;
    BOOST_CHECK_EQUAL(grid.fromGrid(Index(0, 0), pos), false);

    Index idx;    
    BOOST_CHECK_EQUAL(grid.toGrid(Eigen::Vector3d(0, 0, 0), idx), false);     
}

BOOST_AUTO_TEST_CASE(test_grid_constructor)
{
    // TODO: check transformation (offset)
    Grid grid(Vector2ui(100, 100), Vector2d(0.153, 0.257));

    BOOST_CHECK_EQUAL(grid.getNumCells(), Vector2ui(100, 100));
    BOOST_CHECK_EQUAL(grid.getResolution(), Vector2d(0.153, 0.257));    
    BOOST_CHECK_EQUAL(grid.getSize().isApprox(Vector2d(15.3, 25.7), 0.0001), true);      
}

BOOST_AUTO_TEST_CASE(test_grid_index_in_grid)
{ 
    Grid grid(Vector2ui(100, 200), Vector2d(0.1, 0.5)); 

    BOOST_CHECK_EQUAL(grid.inGrid(Index(0, 0)), true);          // bottom right
    BOOST_CHECK_EQUAL(grid.inGrid(Index(99, 199)), true);       // top left   
    BOOST_CHECK_EQUAL(grid.inGrid(Index(50, 100)), true);       // middle    
    BOOST_CHECK_EQUAL(grid.inGrid(Index(100, 200)), false);     // outside     
}

BOOST_AUTO_TEST_CASE(test_grid_index2pos_without_offset)
{
    // size: 10 x 100
    Grid grid(Vector2ui(100, 200), Vector2d(0.1, 0.5)); 

    Vector3d pos;

    // bottom right
    BOOST_CHECK_EQUAL(grid.fromGrid(Index(0, 0), pos), true);
    BOOST_CHECK_EQUAL(pos.isApprox(Vector3d(0.05, 0.25, 0.), 0.0001), true);  

    BOOST_CHECK_EQUAL(grid.fromGrid(Index(0, 1), pos), true);
    BOOST_CHECK_EQUAL(pos.isApprox(Vector3d(0.05, 0.75, 0.), 0.0001), true);    

    BOOST_CHECK_EQUAL(grid.fromGrid(Index(1, 0), pos), true);
    BOOST_CHECK_EQUAL(pos.isApprox(Vector3d(0.15, 0.25, 0.), 0.0001), true);         

    BOOST_CHECK_EQUAL(grid.fromGrid(Index(1, 1), pos), true);
    BOOST_CHECK_EQUAL(pos.isApprox(Vector3d(0.15, 0.75, 0.), 0.0001), true); 

    // top left
    BOOST_CHECK_EQUAL(grid.fromGrid(Index(99, 199), pos), true);
    BOOST_CHECK_EQUAL(pos.isApprox(Vector3d(9.95, 99.75, 0.), 0.0001), true);        

    // middle
    BOOST_CHECK_EQUAL(grid.fromGrid(Index(49, 99), pos), true);
    BOOST_CHECK_EQUAL(pos.isApprox(Vector3d(4.95, 49.75, 0.), 0.0001), true);        

    // outside: pos should be unchanged
    BOOST_CHECK_EQUAL(grid.fromGrid(Index(100, 200), pos), false); 
    BOOST_CHECK_EQUAL(pos.isApprox(Vector3d(4.95, 49.75, 0.), 0.0001), true);   
}

BOOST_AUTO_TEST_CASE(test_grid_index2pos_with_offset)
{
    Grid grid(Vector2ui(100, 200), Vector2d(0.1, 0.5));   
    grid.getOffset().translate(Vector3d(-5, -50, 0));

    // ---- Index 2 Position ---- 
    Vector3d pos;

    // bottom right
    BOOST_CHECK_EQUAL(grid.fromGrid(Index(0, 0), pos), true);
    BOOST_CHECK_EQUAL(pos.isApprox(Vector3d(-4.95, -49.75, 0.), 0.0001), true);  

    // top left
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

BOOST_AUTO_TEST_CASE(test_grid_index2pos_in_frame)
{
    Grid grid(Vector2ui(100, 100), Vector2d(0.1, 0.5));
    grid.getOffset().translate(Vector3d(-5, -50, 0));

    // ---- Index 2 Position in the specific frame ---- 

    Eigen::Vector3d pos;

    base::Transform3d frame_in_grid(Eigen::Translation3d( 0.5, 1.7, -0.5 ));

    // bottom right
    BOOST_CHECK_EQUAL(grid.fromGrid(Index(0, 0), pos, frame_in_grid), true);
    BOOST_CHECK_EQUAL(pos.isApprox(Vector3d(-5.45, -51.45, 0.5), 0.0001), true);      

    // top left
    BOOST_CHECK_EQUAL(grid.fromGrid(Index(99, 199), pos, frame_in_grid), true);
    BOOST_CHECK_EQUAL(pos.isApprox(Vector3d(4.45, 48.05, 0.5), 0.0001), true);           

    // middle
    BOOST_CHECK_EQUAL(grid.fromGrid(Index(50, 100), pos, frame_in_grid), true);
    BOOST_CHECK_EQUAL(pos.isApprox(Vector3d(-0.45, -1.45, 0.5), 0.0001), true); 

    // outside: pos should be unchanged
    BOOST_CHECK_EQUAL(grid.fromGrid(Index(100, 200), pos, frame_in_grid), false);  
    BOOST_CHECK_EQUAL(pos.isApprox(Vector3d(-0.45, -1.45, 0.5), 0.0001), true);          
}

BOOST_AUTO_TEST_CASE(test_grid_pos2index_without_offset)
{
    // size: 10 x 100
    Grid grid(Vector2ui(100, 200), Vector2d(0.1, 0.5)); 

    // ---- Position 2 Index ---- 

    Index idx;
    Vector3d pos_diff;

    // bottom right: pos is corner of the cell
    BOOST_CHECK_EQUAL(grid.toGrid(Vector3d(0.0, 0.0, 0.), idx, pos_diff), true);
    BOOST_CHECK_EQUAL(idx, Index(0,0));
    BOOST_CHECK_EQUAL(pos_diff.isApprox(Vector3d(-0.05, -0.25, 0.0), 0.0001), true);  

    // bottom right: pos is center of the cell
    BOOST_CHECK_EQUAL(grid.toGrid(Vector3d(0.05, 0.25, 0.), idx, pos_diff), true);
    BOOST_CHECK_EQUAL(idx, Index(0,0));
    BOOST_CHECK_EQUAL(pos_diff.norm() <= 1e-6, true); 

    BOOST_CHECK_EQUAL(grid.toGrid(Vector3d(0.1, 0.5, 0.), idx, pos_diff), true);
    BOOST_CHECK_EQUAL(idx, Index(1,1));
    BOOST_CHECK_EQUAL(pos_diff.isApprox(Vector3d(-0.05, -0.25, 0.0), 0.0001), true);   

    BOOST_CHECK_EQUAL(grid.toGrid(Vector3d(0.1, 0.0, 0.), idx, pos_diff), true);
    BOOST_CHECK_EQUAL(idx, Index(1,0));
    BOOST_CHECK_EQUAL(pos_diff.isApprox(Vector3d(-0.05, -0.25, 0.0), 0.0001), true);   

    // top left: pos is center of the cell
    BOOST_CHECK_EQUAL(grid.toGrid(Vector3d(9.95, 99.75, 0.), idx, pos_diff), true);
    BOOST_CHECK_EQUAL(idx, Index(99, 199)); 
    BOOST_CHECK_EQUAL(pos_diff.norm() <= 1e-6, true); 

    BOOST_CHECK_EQUAL(grid.toGrid(Vector3d(9.99, 99.99, 0.), idx, pos_diff), true);
    BOOST_CHECK_EQUAL(idx, Index(99, 199)); 
    BOOST_CHECK_EQUAL(pos_diff.isApprox(Vector3d(0.04, 0.24, 0.0), 0.0001), true); 

    // top left: pos is top left corner
    BOOST_CHECK_EQUAL(grid.toGrid(Vector3d(10, 100, 0.), idx, pos_diff), false);
    BOOST_CHECK_EQUAL(idx, Index(99, 199)); 
    BOOST_CHECK_EQUAL(pos_diff.isApprox(Vector3d(0.04, 0.24, 0.0), 0.0001), true);  
}

BOOST_AUTO_TEST_CASE(test_grid_pos2index_with_offset)
{
    Grid grid(Vector2ui(100, 200), Vector2d(0.1, 0.5));   
    grid.getOffset().translate(Vector3d(-5, -50, 0));

    // ---- Index 2 Position ---- 
    Index idx;
    Vector3d pos_diff;

    // bottom right
    BOOST_CHECK_EQUAL(grid.toGrid(Vector3d(-5, -50, 0.), idx, pos_diff), true);
    BOOST_CHECK_EQUAL(idx, Index(0,0));
    BOOST_CHECK_EQUAL(pos_diff.isApprox(Vector3d(-0.05, -0.25, 0.0), 0.0001), true);  

    BOOST_CHECK_EQUAL(grid.toGrid(Vector3d(-4.91, -49.51, 0.), idx, pos_diff), true);
    BOOST_CHECK_EQUAL(idx, Index(0,0));
    BOOST_CHECK_EQUAL(pos_diff.isApprox(Vector3d(0.04, 0.24, 0.0), 0.0001), true);  

    BOOST_CHECK_EQUAL(grid.toGrid(Vector3d(-4.95, -49.75, 0.), idx, pos_diff), true);
    BOOST_CHECK_EQUAL(idx, Index(0,0));
    BOOST_CHECK_EQUAL(pos_diff.norm() <= 1e-6, true); 

    BOOST_CHECK_EQUAL(grid.toGrid(Vector3d(-4.90, -49.50, 0.), idx, pos_diff), true);
    BOOST_CHECK_EQUAL(idx, Index(1,1));
    BOOST_CHECK_EQUAL(pos_diff.isApprox(Vector3d(-0.05, -0.25, 0.0), 0.0001), true);     

    // top left: pos is center of the cell
    BOOST_CHECK_EQUAL(grid.toGrid(Vector3d(4.95, 49.75, 0.), idx, pos_diff), true);
    BOOST_CHECK_EQUAL(idx, Index(99, 199));
    BOOST_CHECK_EQUAL(pos_diff.norm() <= 1e-6, true);

    // top left: pos is top left corner
    BOOST_CHECK_EQUAL(grid.toGrid(Vector3d(5, 50, 0.), idx, pos_diff), false);
    BOOST_CHECK_EQUAL(idx, Index(99, 199));
    BOOST_CHECK_EQUAL(pos_diff.norm() <= 1e-6, true);

    //TODO: write more tests with the Transform3d including rotation
}

BOOST_AUTO_TEST_CASE(test_grid_pos2index_in_frame)
{
    Grid grid(Vector2ui(100, 100), Vector2d(0.1, 0.5));
    grid.getOffset().translate(Vector3d(-5, -50, 0));

    // ---- Index 2 Position in the specific frame ---- 

    Eigen::Vector3d pos;

    base::Transform3d frame_in_grid(Eigen::Translation3d( 0.5, 1.7, -0.5 ));

    // bottom right
    Index idx;
    BOOST_CHECK_EQUAL(grid.toGrid(Vector3d(-5.45, -51.45, 0.5), idx, frame_in_grid), true);
    BOOST_CHECK_EQUAL(idx, Index(0,0));    

    // top left
    BOOST_CHECK_EQUAL(grid.toGrid(Vector3d(4.45, 48.05, 0.5), idx, frame_in_grid), true);    
    BOOST_CHECK_EQUAL(idx, Index(99, 199));   

    // middle
    BOOST_CHECK_EQUAL(grid.toGrid(Vector3d(-0.45, -1.45, 0.5), idx, frame_in_grid), true);    
    BOOST_CHECK_EQUAL(idx, Index(50, 100));   

    // outside: pos should be unchanged
    BOOST_CHECK_EQUAL(grid.toGrid(Vector3d(5, 50, 0.5), idx, frame_in_grid), false);    
    BOOST_CHECK_EQUAL(idx, Index(50, 100));         
}