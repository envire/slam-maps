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
#ifndef __VIZKIT3D_MAP_VISUALIZATION_HPP_
#define __VIZKIT3D_MAP_VISUALIZATION_HPP_

#include <base/Eigen.hpp>

#include <boost/noncopyable.hpp>

#include <Eigen/Geometry>

#include <maps/grid/Index.hpp>

#include <vizkit3d/ExtentsRectangle.hpp>
#include <vizkit3d/Vizkit3DPlugin.hpp>


namespace vizkit3d
{
    /**
     * @brief: BaseClass for (grid based) map visualsizations.
     *         Provides functionality to apply the local frame of the map
     *         to the visualization as well as draw a rectangular border
     *         around the map extents.
     * @details: Derive from this class as follows:
     *
     *           class MyVisualization
     *           : public MapVisualization<FirstGridType>
     *           , public VizPluginAddType<SecondGridType> // Optional. Repeat to add more types.
     *           {
     *               Q_OBJECT  // Needed to enable QObject functionality.
     *
     *               Q_PROPERTY(bool showMapExtents READ areMapExtentsShown WRITE setShowMapExtents)  // Include this if you want to use visualizeMapExtents.
     *
     *               // Rest of class body
     *           };
     */
    template <typename GridType>
    class MapVisualization
        : public Vizkit3DPlugin<GridType>
        , boost::noncopyable
    {

    public:
        MapVisualization()
            : Vizkit3DPlugin<GridType>()
            , showMapExtents(false) {}

        virtual ~MapVisualization()
        {
        }

    protected:

        // You can override this function if needed. If you do, call this method
        // in your own function to create your main node.
        // WARNING: Be careful when removing or adding children, as this class
        // adds/removes children on its own! It is advisable to add an
        // osg::group as a member to your class which holds your own children.
        virtual osg::ref_ptr<osg::Node> createMainNode()
        {
            mainNode = new osg::PositionAttitudeTransform();
            mapExtentsGeode = new ExtentsRectangle(Eigen::Vector2d(), Eigen::Vector2d());
            return mainNode;
        }

        // Call this in your updateMainNode funtion, if you want the mapExtents displayed.
        // Add showMapExtents as a Q_Property as above if you want it to be togglable.
        void visualizeMapExtents(const maps::grid::CellExtents& mapExtents, const maps::grid::Vector2d& resolution) const
        {
            if (showMapExtents && !mapExtents.isEmpty())
            {
                // Add mapExtentsGeode if it it is not contained in mainNode.
                if (!mainNode->containsNode(mapExtentsGeode.get()))
                    mainNode->addChild(mapExtentsGeode.get());

                Eigen::Vector2d min = mapExtents.min().cast<double>();
                Eigen::Vector2d max = mapExtents.max().cast<double>();

                // Increase the max corners coordinates by 1 each since an ExtentsRectangle with
                // max = (5, 5) will be drawn excluding cell(5, 5).
                ++max.x();
                ++max.y();

                min = min.cwiseProduct(resolution);
                max = max.cwiseProduct(resolution);

                mapExtentsGeode->update(Eigen::Vector2d(min.x(), min.y()),
                                        Eigen::Vector2d(max.x(), max.y()));
            }
            else if (mainNode->containsNode(mapExtentsGeode.get()))
            {
                // Remove mapExtentsGeode if it is contained in mainNode and is not meant to be shown.
                mainNode->removeChild(mapExtentsGeode.get());
            }
        }

        // Call this in your updateMainNode funtion, if you want to apply a local frame.
        void setLocalFrame(const base::Transform3d& localFrame)
        {
            base::Transform3d inverseLocalFrame = localFrame.inverse();

            Eigen::Vector3d translation(inverseLocalFrame.translation());
            Eigen::Quaterniond rotation(inverseLocalFrame.rotation());

            setLocalFrame(osg::Vec3(translation.x(),
                                    translation.y(),
                                    translation.z()),
                          osg::Quat(rotation.x(),
                                    rotation.y(),
                                    rotation.z(),
                                    rotation.w()));
        }
        void setLocalFrame(const osg::Vec3& translation, const osg::Quat& rotation)
        {
            this->mainNode->setPosition(translation);
            this->mainNode->setAttitude(rotation);
        }


    private:
        osg::ref_ptr<osg::PositionAttitudeTransform> mainNode;
        osg::ref_ptr<ExtentsRectangle> mapExtentsGeode;

        bool showMapExtents;

    public slots:
        void setShowMapExtents(bool enabled)
        {
            showMapExtents = enabled;
            emit this->propertyChanged("showMapExtents");
            this->setDirty();
        }

        bool areMapExtentsShown() const
        {
            return showMapExtents;
        }
    };
}  // end namespace vizkit3d

#endif  // __VIZKIT3D_MAP_VISUALIZATION_HPP_
