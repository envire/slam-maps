#include <boost/test/unit_test.hpp>
#include <envire_maps/Grid.hpp>

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

    // check the "<" operator
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
}

BOOST_AUTO_TEST_CASE(test_grid_empty)
{
    Grid grid(Eigen::Vector2d(0.1, 0.1), Vector2ui(100, 100));
}