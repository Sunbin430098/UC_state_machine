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
#ifndef _RENDERING_MATERIAL_HH_
#define _RENDERING_MATERIAL_HH_

#include <string>

#include <ignition/math/Color.hh>

#include "/home/ubuntu/wtr_upmachine_ws/src/assembly/include/gazebo/common/Material.hh"
#include "/home/ubuntu/wtr_upmachine_ws/src/assembly/include/gazebo/util/system.hh"

/// \cond
namespace gazebo
{
  namespace rendering
  {
    /// \class Material Material.hh rendering/rendering.hh
    /// \brief An internal class used by Visuals to add materials to Ogre.
    class GZ_RENDERING_VISIBLE Material
    {
      /// \brief Create all the default materials
      public: static void CreateMaterials();

      /// \brief Update the Ogre materials from a Gazebo material.
      /// \param[in] _mat The Gazebo material to add to the Ogre system.
      public: static void Update(const gazebo::common::Material *_mat);

      /// \brief Get the color of the material.
      /// \param[in] _materialName Name of the material.
      /// \param[out] _ambient Ambient color of the material.
      /// \param[out] _diffuse Diffuse color of the material.
      /// \param[out] _specular Specular color of the material.
      /// \param[out] _emissive Emissive color of the material.
      /// \return True if the material found, false otherwise.
      /// \deprecated use ignition::math::Color
      public: static bool GetMaterialAsColor(const std::string &_materialName,
          common::Color &_ambient, common::Color &_diffuse,
          common::Color &_specular, common::Color &_emissive)
              GAZEBO_DEPRECATED(9.0);

      /// \brief Get the color of the material.
      /// \param[in] _materialName Name of the material.
      /// \param[out] _ambient Ambient color of the material.
      /// \param[out] _diffuse Diffuse color of the material.
      /// \param[out] _specular Specular color of the material.
      /// \param[out] _emissive Emissive color of the material.
      /// \return True if the material found, false otherwise.
      public: static bool MaterialAsColor(const std::string &_materialName,
          ignition::math::Color &_ambient, ignition::math::Color &_diffuse,
          ignition::math::Color &_specular, ignition::math::Color &_emissive);
    };
  }
}
/// \endcond
#endif
