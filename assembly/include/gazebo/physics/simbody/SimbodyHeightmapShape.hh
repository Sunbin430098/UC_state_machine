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

#ifndef _SIMBODY_HEIGHTMAPGEOM_HH_
#define _SIMBODY_HEIGHTMAPGEOM_HH_

#include <string>

#include "/home/ubuntu/wtr_upmachine_ws/src/assembly/include/gazebo/physics/HeightmapShape.hh"
#include "/home/ubuntu/wtr_upmachine_ws/src/assembly/include/gazebo/physics/simbody/SimbodyPhysics.hh"
#include "/home/ubuntu/wtr_upmachine_ws/src/assembly/include/gazebo/physics/Collision.hh"
#include "/home/ubuntu/wtr_upmachine_ws/src/assembly/include/gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_simbody Simbody Physics
    /// \{

    /// \brief Height map collision.
    class GZ_PHYSICS_VISIBLE SimbodyHeightmapShape : public HeightmapShape
    {
      /// \brief Constructor.
      public: explicit SimbodyHeightmapShape(CollisionPtr _parent);

      /// \brief Destructor.
      public: virtual ~SimbodyHeightmapShape();

      // Documentation inherited.
      public: virtual void Init();
    };
    /// \}
  }
}
#endif
