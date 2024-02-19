// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/* This header must be included by all rclcpp headers which declare symbols
 * which are defined in the rclcpp library. When not building the rclcpp
 * library, i.e. when using the headers in other package's code, the contents
 * of this header change the visibility of certain symbols which the rclcpp
 * library cannot have, but the consuming code must have inorder to link.
 */

#ifndef CAR_GAZEBO__VISIBILITY_CONTROL_H_
#define CAR_GAZEBO__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define CAR_GAZEBO_EXPORT __attribute__((dllexport))
#define CAR_GAZEBO_IMPORT __attribute__((dllimport))
#else
#define CAR_GAZEBO_EXPORT __declspec(dllexport)
#define CAR_GAZEBO_IMPORT __declspec(dllimport)
#endif
#ifdef CAR_GAZEBO_BUILDING_DLL
#define CAR_GAZEBO_PUBLIC ROS2_CONTROL_DEMO_EXAMPLE_11_EXPORT
#else
#define CAR_GAZEBO_PUBLIC ROS2_CONTROL_DEMO_EXAMPLE_11_IMPORT
#endif
#define CAR_GAZEBO_PUBLIC_TYPE ROS2_CONTROL_DEMO_EXAMPLE_11_PUBLIC
#define CAR_GAZEBO_LOCAL
#else
#define CAR_GAZEBO_EXPORT __attribute__((visibility("default")))
#define CAR_GAZEBO_IMPORT
#if __GNUC__ >= 4
#define CAR_GAZEBO_PUBLIC __attribute__((visibility("default")))
#define CAR_GAZEBO_LOCAL __attribute__((visibility("hidden")))
#else
#define CAR_GAZEBO_PUBLIC
#define CAR_GAZEBO_LOCAL
#endif
#define CAR_GAZEBO_PUBLIC_TYPE
#endif

#endif // CAR_GAZEBO__VISIBILITY_CONTROL_H_
