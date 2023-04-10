/* config.h. Generated by CMake for Gazebo. */
#ifndef _GAZEBO_CONFIG_H_
#define _GAZEBO_CONFIG_H_

/* Version number */
#define GAZEBO_MAJOR_VERSION 9
#define GAZEBO_MINOR_VERSION 0
#define GAZEBO_PATCH_VERSION 0

#define GAZEBO_VERSION "9.0"
#define GAZEBO_VERSION_FULL "9.0.0"

#define GAZEBO_VERSION_HEADER "Gazebo multi-robot simulator, version 9.0.0\nCopyright (C) 2012 Open Source Robotics Foundation.\nReleased under the Apache 2 License.\nhttp://gazebosim.org\n\n"

// Warning: the GAZEBO_BUILD_TYPE_* macros are unreliable and deprecated.
// They can change each time you install a new version of gazebo.
// They should not be used and will be removed in the future.
/* #undef GAZEBO_BUILD_TYPE_PROFILE */
/* #undef GAZEBO_BUILD_TYPE_DEBUG */
/* #undef GAZEBO_BUILD_TYPE_RELEASE */
/* #undef GAZEBO_BUILD_TYPE_COVERAGE */
#define HAVE_OPENAL 1
#define HAVE_FFMPEG 1
#define HAVE_AVDEVICE 1
/* #undef ENABLE_SHADOWS */
#define HAVE_BULLET 1
#define HAVE_SIMBODY 1
/* #undef HAVE_DART */
#define INCLUDE_RTSHADER 1
#define HAVE_GTS 1
/* #undef ENABLE_DIAGNOSTICS */
#define HAVE_GDAL 1
#define HAVE_USB 1
/* #undef HAVE_OCULUS */
#define HAVE_SPNAV 1
/* #undef HDF5_INSTRUMENT */
#define HAVE_GRAPHVIZ 1
#define HAVE_UUID 1
#define USE_EXTERNAL_TINYXML2 1
/* #undef HAVE_OSVR */
#define HAVE_IGNITION_FUEL_TOOLS 1

#ifdef GAZEBO_BUILD_TYPE_PROFILE
#include <gperftools/heap-checker.h>
#endif

// ifndef _GAZEBO_CONFIG_H_
#endif