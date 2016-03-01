#include <boost/test/unit_test.hpp>
#include <envire_maps/Grid.hpp>
#include <iostream>

using namespace envire::maps;

BOOST_AUTO_TEST_CASE(test_grid_index)
{
    // Index is (x,y)

    // check default constructor
    Index index;
    BOOST_CHECK_EQUAL(index.x(), 0);
    BOOST_CHECK_EQUAL(index.y(), 0);

    // check constructor with parameter
    Index index2(2, 4);
    BOOST_CHECK_EQUAL(index2.x(), 2);
    BOOST_CHECK_EQUAL(index2.y(), 4);

    // Check constructor using Eigen expressions
    Index index3(Eigen::Matrix<unsigned int, 2, 1>(3, 4));
    BOOST_CHECK_EQUAL(index3.x(), 3);
    BOOST_CHECK_EQUAL(index3.y(), 4);
    // check the "<" operator
    BOOST_CHECK_EQUAL((index2 < Index(index2.x() + 1, index2.y() + 1)), true);  //(2,4) < (3,5)
    BOOST_CHECK_EQUAL((index2 < Index(index2.x(), index2.y())), false);         //(2,4) < (2,4)
    BOOST_CHECK_EQUAL((index2 < Index(index2.x(), index2.y() + 1)), false);     //(2,4) < (2,5)
    BOOST_CHECK_EQUAL((index2 < Index(index2.x(), index2.y() - 1)), false);     //(2,4) < (2,3)
    BOOST_CHECK_EQUAL((index2 < Index(index2.x() - 1, index2.y() + 1)), false); //(2,4) < (1,5))
    BOOST_CHECK_EQUAL((index2 < Index(index2.x() - 1, index2.y() - 1)), false); //(2,4) < (1,3)
    BOOST_CHECK_EQUAL((index2 < Index(0,0)), false); //(2,4) < (0,0)

    // check the ">" operator
    BOOST_CHECK_EQUAL((index2 > Index(index2.x() - 1, index2.y() - 1)), true);  //(2,4) > (1,3)
    BOOST_CHECK_EQUAL((index2 > Index(index2.x(), index2.y())), false);         //(2,4) > (2,4)
    BOOST_CHECK_EQUAL((index2 > Index(index2.x(), index2.y() - 1)), false);     //(2,4) > (2,3)
    BOOST_CHECK_EQUAL((index2 > Index(index2.x(), index2.y() + 1)), false);     //(2,4) > (2,5)
    BOOST_CHECK_EQUAL((index2 > Index(index2.x() + 1, index2.y() - 1)), false); //(2,4) > (3,3)
    BOOST_CHECK_EQUAL((index2 > Index(index2.x() + 1, index2.y() + 1)), false); //(2,4) > (3,5)

    // check the "==" operator
    BOOST_CHECK_EQUAL((index2 == Index(index2.x(), index2.y())), true);         //(2,4) == (2,4)
    BOOST_CHECK_EQUAL((index2 == Index(index2.x(), index2.y() + 1)), false);    //(2,4) == (2,5)
    BOOST_CHECK_EQUAL((index2 == Index(index2.x() + 1, index2.y())), false);    //(2,4) == (3,4)
    BOOST_CHECK_EQUAL((index2 == Index(index2.x() + 1, index2.y() + 1)), false);//(2,4) == (3,5)    

    // check the "!=" operator
    BOOST_CHECK_EQUAL((index2 != Index(index2.x(), index2.y())), false);        //(2,4) == (2,4)
    BOOST_CHECK_EQUAL((index2 != Index(index2.x(), index2.y() + 1)), true);     //(2,4) == (2,5)
    BOOST_CHECK_EQUAL((index2 != Index(index2.x() + 1, index2.y())), true);     //(2,4) == (3,4)
    BOOST_CHECK_EQUAL((index2 != Index(index2.x() + 1, index2.y() + 1)), true); //(2,4) == (3,5)

    // check the sum operator
    BOOST_CHECK((index2 + Index(3, 5)) == Index(5, 9));     //(2,4) + (3,5) = (5,9)

    // check the subtract operator
    BOOST_CHECK((Index(5, 9) - index2) == Index(3, 5));     //(5,9) - (2,4) = (3,5)
}

BOOST_AUTO_TEST_CASE(test_grid_constructor_default)
{
    BOOST_TEST_MESSAGE("test_grid_constructor_default");
    Grid grid;

    BOOST_CHECK_EQUAL(grid.getNumCells(), Vector2ui(0, 0));
    BOOST_CHECK_EQUAL(grid.getResolution(), Vector2d(0, 0));
    BOOST_CHECK_EQUAL(grid.getSize(), Vector2d(0, 0));

    BOOST_CHECK_EQUAL(grid.inGrid(Index(0, 0)), false);

    Vector3d pos(0.00,0.00,0.00);
    BOOST_CHECK_EQUAL(grid.fromGrid(Index(0, 0), pos), false);
    BOOST_TEST_MESSAGE("Position from grid: "<<pos[0]<<","<<pos[1]<<","<<pos[2]);

    Index idx;
    BOOST_CHECK_EQUAL(grid.toGrid(Eigen::Vector3d(0, 0, 0), idx), false);
    BOOST_TEST_MESSAGE("Idx to grid: "<<idx[0]<<","<<idx[1]);
}

BOOST_AUTO_TEST_CASE(test_grid_constructor)
{
    Grid grid(Vector2ui(100, 100), Vector2d(0.153, 0.257));

    /** Check grid members **/
    BOOST_CHECK_EQUAL(grid.getNumCells(), Vector2ui(100, 100));
    BOOST_CHECK_EQUAL(grid.getResolution(), Vector2d(0.153, 0.257));
    BOOST_CHECK_EQUAL(grid.getSize().isApprox(Vector2d(15.3, 25.7), 0.0001), true);

    /** Check local map members **/
    BOOST_CHECK_EQUAL(grid.getId(), envire::maps::UNKNOWN_MAP_ID);
    BOOST_CHECK_EQUAL(grid.getMapType(), envire::maps::LocalMapType::UNKNOWN_MAP);
    BOOST_CHECK_EQUAL(grid.getLocalFrame().translation(), base::Transform3d::Identity().translation());
    BOOST_CHECK_EQUAL(grid.getLocalFrame().rotation(), base::Transform3d::Identity().rotation());

}

BOOST_AUTO_TEST_CASE(test_grid_index_in_grid)
{
    //Grid of 10x100
    Grid grid(Vector2ui(100, 200), Vector2d(0.1, 0.5));

    BOOST_CHECK_EQUAL(grid.inGrid(Index(0, 0)), true);          // bottom left (according to 1873-2015 IEEE standard)
    BOOST_CHECK_EQUAL(grid.inGrid(Index(99, 199)), true);       // top right (according to 1873-2015 IEEE standard)
    BOOST_CHECK_EQUAL(grid.inGrid(Index(50, 100)), true);       // middle
    BOOST_CHECK_EQUAL(grid.inGrid(Index(100, 200)), false);     // outside
    BOOST_CHECK_EQUAL(grid.inGrid(Index(99, 200)), false);     // outside
    BOOST_CHECK_EQUAL(grid.inGrid(Index(100, 199)), false);     // outside
    BOOST_CHECK_EQUAL(grid.inGrid(Index(-1, -1)), false);     // outside
    BOOST_CHECK_EQUAL(grid.inGrid(Index(-1, 2)), false);     // outside
    BOOST_CHECK_EQUAL(grid.inGrid(Index(2, -1)), false);     // outside
}

BOOST_AUTO_TEST_CASE(test_grid_from_grid_without_offset)
{
    BOOST_TEST_MESSAGE("test_grid_from_grid_without_offset");

    // size: 10 x 100
    Grid grid(Vector2ui(100, 200), Vector2d(0.1, 0.5));

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
    Grid grid(Vector2ui(100, 200), Vector2d(0.1, 0.5));   
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
    Grid grid(Vector2ui(100, 200), Vector2d(0.1, 0.5));   
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
    Grid grid(Vector2ui(100, 100), Vector2d(0.1, 0.5));
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
    Grid grid(Vector2ui(100, 200), Vector2d(0.1, 0.5));

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
    Grid grid(Vector2ui(100, 200), Vector2d(0.1, 0.5));
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
    Grid grid(Vector2ui(100, 200), Vector2d(0.1, 0.5));
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
    Grid grid(Vector2ui(100, 100), Vector2d(0.1, 0.5));
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
