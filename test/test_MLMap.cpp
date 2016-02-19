#include <boost/test/unit_test.hpp>
#include "../src/MLSGrid.hpp"

using namespace envire::maps;

BOOST_AUTO_TEST_CASE(test_base_class)
{
    class Patch
    {
        public:
            Patch(double m, double ma) : min(m), max(ma)
            {
            }
            
            double min;
            double max;

            double getMiddle() const 
            {
                return min + (max - min) / 2.0;
            }
            
        bool operator<(const Patch &other) const{
            return  getMiddle() < other.getMiddle();
        }

        double getMin() const {
            return min;
        };

        double getMax() const {
            return max;
        };
    };
    
    MLGrid<Patch> grid(Vector2ui(5,5), Eigen::Vector2d(0.5,0.5));

    Patch p(38, 50);
    Patch p2(55, 80);
    
    grid.at(2,2).insert(p2);
    grid.at(2,2).insert(p);
    
    for(const Patch &p : grid.at(2,2))
        std::cout << "Bar is " << p.getMin() << std::endl;

    Eigen::Vector3d pos;
    grid.fromGrid(Index(2,2), pos);
    
    std::cout << "From" << pos.transpose() << std::endl;
    
    Eigen::AlignedBox3f box(Eigen::Vector3f(0.5, .5, 49),Eigen::Vector3f(2.0, 2.0, 60));
    
    MLGrid<Patch>::MLView view = grid.intersectCuboid(box);
    
    std::cout << "Subview Size : " << view.getNumCells().transpose() << std::endl;
    for(size_t x = 0; x < view.getNumCells().x(); x++)
    {
        for(size_t y = 0; y < view.getNumCells().y(); y++)
        {
            for(const Patch *p : view.at(x,y))
                std::cout << "X " << x << " Y " << y << " Bar is " << p->getMin() << std::endl;
        }
    }
    

    
    
}


