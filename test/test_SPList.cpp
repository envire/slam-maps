#include <boost/test/unit_test.hpp>
#include <envire_maps/SPList.hpp>

#include <chrono>

using namespace envire::maps;


BOOST_AUTO_TEST_CASE(test_sp_list_basic)
{
    SPList list;
    BOOST_CHECK_EQUAL(list.size(), 0);

    SurfacePatch sp(Eigen::Vector3f::Random(), 1.0);
    list.insertHead(sp);
    BOOST_CHECK_EQUAL(list.size(), 1);
    BOOST_CHECK(list.begin()->getHeight() == sp.getHeight());

    SPList list2 = list;
    BOOST_CHECK_EQUAL(list2.size(), 1);




    std::cout << sizeof list << " " << sizeof(MLSConfig) << " " << sizeof(List<SurfacePatch>) << std::endl;
}

BOOST_AUTO_TEST_CASE(test_sp_list_update)
{
    MLSConfig config;
    config.updateModel = MLSConfig::SLOPE;
    const int num_layers = 8;
    const int num_points = 100;
    for(int k=0; k<20; ++k)
    {
        // Create random surface patches:
        std::vector<SurfacePatch> patches;
        for(int i=0; i<num_points; ++i)
        {
            Eigen::Vector3f point = Eigen::Vector3f::Random();
            point.z()*=0.1;
            point.z() += (rand()%num_layers)*2.0; // distribute points to layers 2 meters apart
            patches.push_back(SurfacePatch(point, 1.0));
        }

        SPList list1(config), list2(config);
        for(int i=0; i<num_points; ++i)
        {
            list1.update(patches[i]);
            list2.update(patches[num_points-1-i]);
        }
        // insert every other surface patch again (in reverse order)
        for(int i=0; i<num_points; i+=2)
        {
            list1.update(patches[num_points-2-i]);
            list2.update(patches[i]);
        }
        // Both lists shall have the same number of patches
        BOOST_CHECK_EQUAL(list1.size(), list2.size());
        // There must not be more than 8 elements:
        BOOST_CHECK_LE(list1.size(), num_layers);
        for(int i=0; i<20; ++i)
        {
            Eigen::Vector3f point = Eigen::Vector3f::Random();
            point.z()*=0.1;
            point.z() += (rand()%num_layers)*2.0; // distribute points to layers 2 meters apart
            std::pair<SPList::iterator, double>
                near1 = list1.getNearestPatch(SurfacePatch(point, 1)),
                near2 = list2.getNearestPatch(SurfacePatch(point, 1));
            BOOST_CHECK_CLOSE(1+near1.second, 1+near2.second, 0.05);
            BOOST_CHECK_SMALL(near1.first->getHeight(point.head<2>())- point.z(), 0.25f);

        }
        for(int i=0; i<2; ++i)
        {
            // check that lists are orderd
            SPList & list = i == 0? list1 : list2;
            float prev = -1e30;
            for(SPList::iterator it = list.begin(); it != list.end(); ++it)
            {
                float mean = it->getMean();
                BOOST_CHECK_LE(prev, mean);
                prev = mean;
                std::cout  << mean << " ";
            }
            std::cout << std::endl;
        }

        // Insert measurements every 0.5 meters. This will cause adjacent planes to get joined
        // TODO Actually, this is not desired behavior!
        for(float z = 0; z< num_layers*2.0; z+=0.5)
        {
            Eigen::Vector3f pt(0,0,z);
            list1.update(SurfacePatch(pt, 1e-3));
        }
        BOOST_CHECK(list1.size()==1);

    }
}
