/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifndef GAZEBO_PHYSICS_HEIGHTMAPSHAPE_HH_
#define GAZEBO_PHYSICS_HEIGHTMAPSHAPE_HH_

#include <string>
#include <vector>
#include </usr/include/ignition/transport4/ignition/transport/Node.hh>

#include </usr/include/ignition/math4/ignition/math/Vector2.hh>

#include "/home/ubuntu/wtr_upmachine_ws/src/assembly/include/gazebo/common/ImageHeightmap.hh"
#include "/home/ubuntu/wtr_upmachine_ws/src/assembly/include/gazebo/common/HeightmapData.hh"
#include "/home/ubuntu/wtr_upmachine_ws/src/assembly/include/gazebo/common/Dem.hh"
#include "/home/ubuntu/wtr_upmachine_ws/src/assembly/include/gazebo/transport/TransportTypes.hh"
#include "/home/ubuntu/wtr_upmachine_ws/src/assembly/include/gazebo/physics/PhysicsTypes.hh"
#include "/home/ubuntu/wtr_upmachine_ws/src/assembly/include/gazebo/physics/Shape.hh"
#include "/home/ubuntu/wtr_upmachine_ws/src/assembly/include/gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \class HeightmapShape HeightmapShape.hh physics/physics.hh
    /// \brief HeightmapShape collision shape builds a heightmap from
    /// an image.  The supplied image must be square with
    /// N*N+1 pixels per side, where N is an integer.
    class GZ_PHYSICS_VISIBLE HeightmapShape : public Shape
    {
      /// \brief Constructor.
      /// \param[in] _parent Parent Collision object.
      public: explicit HeightmapShape(CollisionPtr _parent);

      /// \brief Destructor.
      public: virtual ~HeightmapShape();

      /// \brief Load the heightmap.
      /// \param[in] _sdf SDF value to load from.
      public: virtual void Load(sdf::ElementPtr _sdf);

      /// \brief Initialize the heightmap.
      public: virtual void Init();

      /// \brief Set the scale of the heightmap shape.
      /// \param[in] _scale Scale to set the heightmap shape to.
      public: virtual void SetScale(const ignition::math::Vector3d &_scale);

      /// \brief Get the URI of the heightmap image.
      /// \return The heightmap image URI.
      public: std::string GetURI() const;

      /// \brief Get the size in meters.
      /// \return The size in meters.
      public: ignition::math::Vector3d Size() const;

      /// \brief Get the origin in world coordinate frame.
      /// \return The origin in world coordinate frame.
      public: ignition::math::Vector3d Pos() const;

      /// \brief Return the number of vertices, which equals the size of the
      /// image used to load the heightmap.
      /// \return ignition::math::Vector2i, result.X() == width,
      /// result.Y() == length/height.
      public: ignition::math::Vector2i VertexCount() const;

      /// \brief Get a height at a position.
      /// \param[in] _x X position.
      /// \param[in] _y Y position.
      /// \return The height at a the specified location.
      public: float GetHeight(int _x, int _y) const;

      /// \brief Fill a geometry message with this shape's data. Raw height
      /// data are not packed in this message to minimize packet size.
      /// \param[in] _msg Message to fill.
      /// \sa FillHeights
      public: void FillMsg(msgs::Geometry &_msg);

      /// \brief Fill a geometry message with this shape's height data.
      /// \param[in] _msg Message to fill.
      public: void FillHeights(msgs::Geometry &_msg) const;

      /// \brief Update the heightmap from a message.
      /// \param[in] _msg Message to update from.
      public: virtual void ProcessMsg(const msgs::Geometry &_msg);

      /// Documentation inherited
      public: virtual double ComputeVolume() const;

      /// \brief Get the maximum height.
      /// \return The maximum height.
      public: float GetMaxHeight() const;

      /// \brief Get the minimum height.
      /// \return The minimum height.
      public: float GetMinHeight() const;

      /// \brief Get the amount of subsampling.
      /// \return Amount of subsampling.
      public: int GetSubSampling() const;

      /// \brief Return an image representation of the heightmap.
      /// \return Image where white pixels represents the highest locations,
      /// and black pixels the lowest.
      public: common::Image GetImage() const;

      /// \brief Load a terrain file specified by _filename. The terrain file
      /// format might be an image or a DEM file. libgdal is required to enable
      /// DEM support. For a list of all raster formats supported you can type
      /// the command "gdalinfo --formats".
      /// \param[in] _filename The path to the terrain file.
      /// \return 0 when the operation succeeds to load a file or -1 when fails.
      private: int LoadTerrainFile(const std::string &_filename);

      /// \brief Handle request messages.
      /// \param[in] _msg The request message.
      private: void OnRequest(ConstRequestPtr &_msg);

      /// \brief Lookup table of heights.
      protected: std::vector<float> heights;

      /// \brief Image used to generate the heights.
      protected: common::ImageHeightmap img;

      /// \brief HeightmapData used to generate the heights.
      protected: common::HeightmapData *heightmapData;

      /// \brief Size of the height lookup table.
      protected: unsigned int vertSize;

      /// \brief True to flip the heights along the y direction.
      protected: bool flipY;

      /// \brief The amount of subsampling. Default is 2.
      protected: int subSampling;

      /// \brief Transportation node.
      private: transport::NodePtr node;

      /// \brief Subscriber to request messages.
      private: transport::SubscriberPtr requestSub;

      /// \brief Publisher for request response messages.
      private: transport::PublisherPtr responsePub;

      /// \brief File format of the heightmap
      private: std::string fileFormat;

      /// \brief Terrain size
      private: ignition::math::Vector3d heightmapSize;

      #ifdef HAVE_GDAL
      /// \brief DEM used to generate the heights.
      private: common::Dem dem;
      #endif

      // Place ignition::transport objects at the end of this file to
      // guarantee they are destructed first.

      /// \brief Ignition transportation node.
      private: ignition::transport::Node nodeIgn;

      /// \brief Igniiont publisher for request response messages.
      private: ignition::transport::Node::Publisher responsePubIgn;
    };
    /// \}
  }
}
#endif
