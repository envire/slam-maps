#include <boost/test/unit_test.hpp>
#include "../src/MLGrid.hpp"
#include "../src/GridStorageInterface.hpp"
#include "../src/GridStorageAccess.hpp"
#include "../src/GridStorageFacade.hpp"

using namespace envire::maps;
class PatchBase
{
    public:
        PatchBase(double m, double ma) : min(m), max(ma)
        {
        }
        
        double min;
        double max;

        double getMiddle() const 
        {
            return min + (max - min) / 2.0;
        }
        
    bool operator<(const PatchBase &other) const{
        return  getMiddle() < other.getMiddle();
    }

    double getMin() const {
        return min;
    };

    double getMax() const {
        return max;
    };
    
    virtual void test()
    {
        std::cout << "Base class " << std::endl; 
    };
};

class Patch : public PatchBase
{
public:
    Patch() : PatchBase(0,0)
    {
    };
    
    Patch(double m, double ma) : PatchBase(m, ma)
    {
    };
    
    virtual void test()
    {
        std::cout << "Derived Class " << std::endl;
    };
    
    double someValue;
};

BOOST_AUTO_TEST_CASE(test_levelAccess)
{
    Patch p;
    
    PatchBase b = p;
    
    p.test();
    
    b.test();

    LevelList<Patch> list;
    
    LevelListAccess<PatchBase> *access = new LevelListAccessImpl<Patch, PatchBase>(&list);
    
    delete access;

}

BOOST_AUTO_TEST_CASE(test_levelAccess2)
{
    DerivableLevelList<Patch, PatchBase> list;

    std::cout << "SuperClass " << std::endl;
    
    list.begin();
    
    std::cout << "BaseClass " << std::endl;
    DerivableLevelList<PatchBase> *listBase = &list;
    
    listBase->begin();
    
    
//     LevelListAccess<PatchBase> *access = new LevelListAccessImpl<Patch, PatchBase>(&list);
    
}

BOOST_AUTO_TEST_CASE(test_mapAccess)
{
    GridMap<Patch> map;

    GridStorageInterface<PatchBase> *test = new GridStorageAccess<Patch, PatchBase>(&map);

    GridMap<PatchBase, GridStorageFacade<PatchBase> > test2(map, GridStorageFacade<PatchBase>(test));
    
    
}

BOOST_AUTO_TEST_CASE(test_mapAccess2)
{
    GridMap<DerivableLevelList<Patch, PatchBase> > map(Vector2ui(5,5), Eigen::Vector2d(0.5,0.5), DerivableLevelList<Patch, PatchBase>());

    GridStorageInterface<DerivableLevelList<PatchBase, PatchBase> > *test = new GridStorageAccess<DerivableLevelList<Patch, PatchBase>, DerivableLevelList<PatchBase, PatchBase> >(&map);

    GridMap<DerivableLevelList<PatchBase, PatchBase>, GridStorageFacade<DerivableLevelList<PatchBase, PatchBase> > > test2(map, GridStorageFacade<DerivableLevelList<PatchBase, PatchBase> >(test));
    
    Patch p(38, 50);
    Patch p2(55, 80);
    
    
    map.at(2,2).insert(p2);
    map.at(2,2).insert(p);
    
    BOOST_CHECK_EQUAL(map.at(2,2).size(), 2);
    
    {
    auto it = map.at(2,2).begin();
    
    BOOST_CHECK_EQUAL(it->getMin(), 38);
    it++;
    BOOST_CHECK_EQUAL(it->getMin(), 55);
    }

    
    BOOST_CHECK_EQUAL(test2.at(2,2).size(), 2);
    
    {
    auto it = test2.at(2,2).begin();
    
    BOOST_CHECK_EQUAL(it->getMin(), 38);
    it++;
    BOOST_CHECK_EQUAL(it->getMin(), 55);
    }

}

BOOST_AUTO_TEST_CASE(test_base_class)
{

    
    MLGrid<PatchBase> grid(Vector2ui(5,5), Eigen::Vector2d(0.5,0.5));

    PatchBase p(38, 50);
    PatchBase p2(55, 80);
    
    grid.at(2,2).insert(p2);
    grid.at(2,2).insert(p);
    
    BOOST_CHECK_EQUAL(grid.at(2,2).size(), 2);
    
    {
    auto it = grid.at(2,2).begin();
    
    BOOST_CHECK_EQUAL(it->getMin(), 38);
    it++;
    BOOST_CHECK_EQUAL(it->getMin(), 55);
    }
//     for(const Patch &p : grid.at(2,2))
//         std::cout << "Bar is " << p.getMin() << std::endl;

//     Eigen::Vector3d pos;
//     grid.fromGrid(Index(2,2), pos);
//     
//     std::cout << "From" << pos.transpose() << std::endl;
    
    {
    Eigen::AlignedBox3f box(Eigen::Vector3f(0.5, .5, 49),Eigen::Vector3f(2.0, 2.0, 60));
    MLGrid<PatchBase>::MLView view = grid.intersectCuboid(box);

//     std::cout << "Subview Size : " << view.getNumCells().transpose() << std::endl;
//     for(size_t x = 0; x < view.getNumCells().x(); x++)
//     {
//         for(size_t y = 0; y < view.getNumCells().y(); y++)
//         {
//             for(const Patch *p : view.at(x,y))
//                 std::cout << "X " << x << " Y " << y << " Bar is " << p->getMin() << std::endl;
//         }
//     }
//     
    BOOST_CHECK_EQUAL(view.at(1,1).size(), 2);
    auto it = view.at(1,1).begin();
    
    BOOST_CHECK_EQUAL((*it)->getMin(), 38);
    it++;
    BOOST_CHECK_EQUAL((*it)->getMin(), 55);
    }

    {
    Eigen::AlignedBox3f box(Eigen::Vector3f(0.5, .5, 37),Eigen::Vector3f(2.0, 2.0, 38));
    MLGrid<PatchBase>::MLView view = grid.intersectCuboid(box);

    BOOST_CHECK_EQUAL(view.at(1,1).size(), 1);
    auto it = view.at(1,1).begin();
    
    BOOST_CHECK_EQUAL((*it)->getMin(), 38);
    }

    {
    Eigen::AlignedBox3f box(Eigen::Vector3f(0.5, .5, 80),Eigen::Vector3f(2.0, 2.0, 105));
    MLGrid<PatchBase>::MLView view = grid.intersectCuboid(box);

    BOOST_CHECK_EQUAL(view.at(1,1).size(), 1);
    auto it = view.at(1,1).begin();
    
    BOOST_CHECK_EQUAL((*it)->getMin(), 55);
    }
}


