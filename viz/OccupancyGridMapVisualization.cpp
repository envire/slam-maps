#include "OccupancyGridMapVisualization.hpp"
#include "PatchesGeode.hpp"

using namespace vizkit3d;

OccupancyGridMapVisualization::OccupancyGridMapVisualization(): Vizkit3DPlugin< maps::grid::OccupancyGridMap >()
{
    show_occupied = true;
    show_freespace = false;
    occupied_color.setRgbF(0.1,0.5,0.9,1.0);
    free_space_color.setRgbF(0.1,0.5,0.9,0.2);
}

OccupancyGridMapVisualization::~OccupancyGridMapVisualization()
{

}

osg::ref_ptr< osg::Node > OccupancyGridMapVisualization::createMainNode()
{
    return vizkit3d::VizPluginBase::createMainNode();
}

void OccupancyGridMapVisualization::updateMainNode(osg::Node* node)
{
    osg::Group* group = static_cast<osg::Group*>(node);
    group->removeChildren(0, group->getNumChildren());

    Eigen::Vector3d res = grid.getVoxelResolution();

    osg::ref_ptr<PatchesGeode> occupied_voxels = new PatchesGeode(res.x(), res.y());
    occupied_voxels->setColor(osg::Vec4f(occupied_color.redF(), occupied_color.greenF(), occupied_color.blueF(), occupied_color.alphaF()));
    group->addChild( occupied_voxels );
    osg::ref_ptr<PatchesGeode> free_space_voxels = new PatchesGeode(res.x(), res.y());
    free_space_voxels->setColor(osg::Vec4f(free_space_color.redF(), free_space_color.greenF(), free_space_color.blueF(), free_space_color.alphaF()));
    group->addChild( free_space_voxels );

    maps::grid::Vector2ui num_cell = grid.getNumCells();
    const maps::grid::OccupancyConfiguration& config = grid.getConfig();
    for (size_t x = 0; x < num_cell.x(); x++)
    {
        for (size_t y = 0; y < num_cell.y(); y++)
        {
            const maps::grid::OccupancyGridMap::GridMapBase::CellType &tree = grid.at(x, y);
            maps::grid::Vector3d pos;
            if(grid.fromGrid(maps::grid::Index(x,y), pos))
            {
                occupied_voxels->setPosition(pos.x(), pos.y());
                free_space_voxels->setPosition(pos.x(), pos.y());
                std::vector< std::pair<int,int> > occ_cells;
                std::vector< std::pair<int,int> > free_cells;
                for(maps::grid::DiscreteTree<maps::grid::OccupancyGridMap::VoxelCellType>::const_iterator cell_it = tree.begin(); cell_it != tree.end(); cell_it++)
                {
                    if(show_occupied && cell_it->second.getLogOdds() >= config.occupied_logodds)
                    {
                        if(occ_cells.empty() || occ_cells.back().second + 1 != cell_it->first)
                            occ_cells.push_back(std::make_pair(cell_it->first, cell_it->first));
                        else
                            occ_cells.back().second++;
                    }
                    else if(show_freespace && cell_it->second.getLogOdds() < config.free_space_logodds)
                    {
                        if(free_cells.empty() || free_cells.back().second + 1 != cell_it->first)
                            free_cells.push_back(std::make_pair(cell_it->first, cell_it->first));
                        else
                            free_cells.back().second++;
                    }
                }

                for(const std::pair<int,int>& cell : occ_cells)
                    occupied_voxels->drawBox(tree.getCellCenter(cell.second) + res.z() * 0.5, res.z() * (float)((cell.second-cell.first)+1), osg::Vec3(0.f,0.f,1.f));
                for(const std::pair<int,int>& cell : free_cells)
                    free_space_voxels->drawBox(tree.getCellCenter(cell.second) + res.z() * 0.5, res.z() * (float)((cell.second-cell.first)+1), osg::Vec3(0.f,0.f,1.f));
            }
        }
    }
}

void OccupancyGridMapVisualization::updateDataIntern(const maps::grid::OccupancyGridMap& grid)
{
    this->grid = grid;
}