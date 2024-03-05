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

#ifndef CAR_GAZEBO__CARLIKEBOT_SYSTEM_HPP_
#define CAR_GAZEBO__CARLIKEBOT_SYSTEM_HPP_

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <chrono>
#include <wiringSerial.h>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "car_gazebo/crc8.hpp"

#include <sys/socket.h>
#include <sys/un.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <unistd.h>
#define PORT 6666

#include "car_gazebo/visibility_control.h"

namespace car_gazebo
{
  struct Car
  {
    float traction_wheel_position;
    float traction_wheel_velocity;
    float steering_position;
  };
  struct JointValue
  {
    double position{0.0};
    double velocity{0.0};
    double effort{0.0};
  };

  struct Joint
  {
    explicit Joint(const std::string &name) : joint_name(name)
    {
      state = JointValue();
      command = JointValue();
    }

    Joint() = default;

    std::string joint_name;
    JointValue state;
    JointValue command;
  };
  class CarlikeBotSystemHardware : public hardware_interface::SystemInterface
  {
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(CarlikeBotSystemHardware)

    CAR_GAZEBO_PUBLIC
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo &info) override;

    CAR_GAZEBO_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    CAR_GAZEBO_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    CAR_GAZEBO_PUBLIC
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;

    CAR_GAZEBO_PUBLIC
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

    CAR_GAZEBO_PUBLIC
    hardware_interface::return_type read(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

    CAR_GAZEBO_PUBLIC
    hardware_interface::return_type write(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    // Parameters for the CarlikeBot simulation
    double hw_start_sec_;
    double hw_stop_sec_;

    // std::vector<std::tuple<std::string, double, double>>
    //     hw_interfaces_; // name of joint, state, command
    std::map<std::string, Joint> hw_interfaces_;

    Car car;

    int server_fd, new_socket;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);

    int sfd;
    enum ReadState {WAIT_START_BYTE, READ_REST_DATA};

    CRC_Hash crc{7};

  };

} // namespace car_gazebo

#endif // CAR_GAZEBO__CARLIKEBOT_SYSTEM_HPP_
