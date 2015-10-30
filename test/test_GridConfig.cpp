#include <boost/test/unit_test.hpp>
#include <envire_maps/GridConfig.hpp>

#include <chrono>

using namespace envire::maps;

BOOST_AUTO_TEST_CASE(test_constructor)
{
	// empty constructor
	GridConfig conf_empty = GridConfig();

	BOOST_CHECK_EQUAL(conf_empty.cellSizeX, 0);
	BOOST_CHECK_EQUAL(conf_empty.cellSizeY, 0);
	BOOST_CHECK_EQUAL(conf_empty.scaleX, 0);
	BOOST_CHECK_EQUAL(conf_empty.scaleY, 0);
	BOOST_CHECK_EQUAL(conf_empty.offsetX, 0);
	BOOST_CHECK_EQUAL(conf_empty.offsetY, 0);

	// constructor with parameter

	GridConfig conf_param(100, 130, 0.5, 0.3, 0.25, -7.25);

	BOOST_CHECK_EQUAL(conf_param.cellSizeX, 100);
	BOOST_CHECK_EQUAL(conf_param.cellSizeY, 130);
	BOOST_CHECK_CLOSE(conf_param.scaleX, 0.5, 0.0001);
	BOOST_CHECK_CLOSE(conf_param.scaleY, 0.3, 0.0001);
	BOOST_CHECK_CLOSE(conf_param.offsetX, 0.25, 0.0001);
	BOOST_CHECK_CLOSE(conf_param.offsetY, -7.25, 0.0001);	
}

