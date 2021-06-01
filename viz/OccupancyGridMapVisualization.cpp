//
// Copyright (c) 2015-2017, Deutsches Forschungszentrum für Künstliche Intelligenz GmbH.
// Copyright (c) 2015-2017, University of Bremen
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
#include "OccupancyGridMapVisualization.hpp"
#include "PatchesGeode.hpp"

using namespace vizkit3d;

OccupancyGridMapVisualization::OccupancyGridMapVisualization()
    : MapVisualization< maps::grid::OccupancyGridMap >()
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
    osg::ref_ptr<osg::Group> mainNode = MapVisualization::createMainNode()->asGroup();
    localNode = new osg::Group();

    mainNode->addChild(localNode.get());

    return mainNode;
}

void OccupancyGridMapVisualization::updateMainNode(osg::Node* node)
{
    // Apply local frame.
    setLocalFrame(grid.getLocalFrame());

    // Draw map extents.
    visualizeMapExtents(grid.calculateCellExtents(), grid.getResolution());

    localNode->removeChildren(0, localNode->getNumChildren());

    Eigen::Vector3d res = grid.getVoxelResolution();

    osg::ref_ptr<PatchesGeode> occupied_voxels = new PatchesGeode(res.x(), res.y());
    occupied_voxels->setColor(osg::Vec4f(occupied_color.redF(), occupied_color.greenF(), occupied_color.blueF(), occupied_color.alphaF()));
    localNode->addChild( occupied_voxels );
    osg::ref_ptr<PatchesGeode> free_space_voxels = new PatchesGeode(res.x(), res.y());
    free_space_voxels->setColor(osg::Vec4f(free_space_color.redF(), free_space_color.greenF(), free_space_color.blueF(), free_space_color.alphaF()));
    localNode->addChild( free_space_voxels );

    maps::grid::Vector2ui num_cell = grid.getNumCells();
    const maps::grid::OccupancyConfiguration& config = grid.getConfig();
    for (size_t x = 0; x < num_cell.x(); x++)
    {
        for (size_t y = 0; y < num_cell.y(); y++)
        {
            const maps::grid::OccupancyGridMap::GridMapBase::CellType &tree = grid.at(x, y);
            maps::grid::Index idx(x, y);
            if(grid.inGrid(idx))
            {
                // Calculate the position of the cell center.
                maps::grid::Vector2d pos = (idx.cast<double>() + maps::grid::Vector2d(0.5, 0.5)).array() * grid.getResolution().array();
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
