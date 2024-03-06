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

#include "car_gazebo/carlikebot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace car_gazebo
{
  hardware_interface::CallbackReturn CarlikeBotSystemHardware::on_init(
      const hardware_interface::HardwareInfo &info)
  {
    if (
        hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Check if the number of joints is correct based on the mode of operation
    if (info_.joints.size() != 2)
    {
      RCLCPP_ERROR(
          rclcpp::get_logger("CarlikeBotSystemHardware"),
          "CarlikeBotSystemHardware::on_init() - Failed to initialize, "
          "because the number of joints %ld is not 2.",
          info_.joints.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      bool joint_is_steering = joint.name.find("front") != std::string::npos;

      // Steering joints have a position command interface and a position state interface
      if (joint_is_steering)
      {
        RCLCPP_INFO(
            rclcpp::get_logger("CarlikeBotSystemHardware"), "Joint '%s' is a steering joint.",
            joint.name.c_str());

        if (joint.command_interfaces.size() != 1)
        {
          RCLCPP_FATAL(
              rclcpp::get_logger("CarlikeBotSystemHardware"),
              "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
              joint.command_interfaces.size());
          return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
          RCLCPP_FATAL(
              rclcpp::get_logger("CarlikeBotSystemHardware"),
              "Joint '%s' has %s command interface. '%s' expected.", joint.name.c_str(),
              joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
          return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() != 1)
        {
          RCLCPP_FATAL(
              rclcpp::get_logger("CarlikeBotSystemHardware"),
              "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
              joint.state_interfaces.size());
          return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
          RCLCPP_FATAL(
              rclcpp::get_logger("CarlikeBotSystemHardware"),
              "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
              joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
          return hardware_interface::CallbackReturn::ERROR;
        }
      }
      else
      {
        RCLCPP_INFO(
            rclcpp::get_logger("CarlikeBotSystemHardware"), "Joint '%s' is a drive joint.",
            joint.name.c_str());

        // Drive joints have a velocity command interface and a velocity state interface
        if (joint.command_interfaces.size() != 1)
        {
          RCLCPP_FATAL(
              rclcpp::get_logger("CarlikeBotSystemHardware"),
              "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
              joint.command_interfaces.size());
          return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
        {
          RCLCPP_FATAL(
              rclcpp::get_logger("CarlikeBotSystemHardware"),
              "Joint '%s' has %s command interface. '%s' expected.", joint.name.c_str(),
              joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
          return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() != 2)
        {
          RCLCPP_FATAL(
              rclcpp::get_logger("CarlikeBotSystemHardware"),
              "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
              joint.state_interfaces.size());
          return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
        {
          RCLCPP_FATAL(
              rclcpp::get_logger("CarlikeBotSystemHardware"),
              "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
              joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
          return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_POSITION)
        {
          RCLCPP_FATAL(
              rclcpp::get_logger("CarlikeBotSystemHardware"),
              "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
              joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_POSITION);
          return hardware_interface::CallbackReturn::ERROR;
        }
      }
    }

    // // BEGIN: This part here is for exemplary purposes - Please do not copy to your production
    // code
    hw_start_sec_ = std::stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
    hw_stop_sec_ = std::stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
    // // END: This part here is for exemplary purposes - Please do not copy to your production code

    hw_interfaces_["steering"] = Joint("virtual_front_wheel_joint");

    hw_interfaces_["traction"] = Joint("virtual_rear_wheel_joint");

    // 1. Create Socket
    // if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
    // {
    //   RCLCPP_ERROR(
    //       rclcpp::get_logger("CarlikeBotSystemHardware"), "socket failed!");
    //   return hardware_interface::CallbackReturn::ERROR;
    // }

    // // 2. Setsockopt
    // // helps in reuse of address and port. Prevents error such as: “address already in use”.
    // int error_state = setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt));
    // if (error_state == -1)
    // {
    //   perror("setsockopt");
    //   RCLCPP_ERROR(
    //       rclcpp::get_logger("CarlikeBotSystemHardware"), "setsockt opt failed %d!", error_state);
    //   return hardware_interface::CallbackReturn::ERROR;
    // }

    // // 3. Bind
    // // after socket creation, this binds the socket to the address and port number
    // // specified in addr(custom data structure).

    // address.sin_family = AF_INET;
    // address.sin_addr.s_addr = INADDR_ANY;
    // address.sin_port = htons(PORT);

    // // bind socket to address and port
    // int bind_state = bind(server_fd, (struct sockaddr *)&address, sizeof(address));
    // if (bind_state < 0)
    // {
    //   perror("bind");
    //   RCLCPP_ERROR(
    //       rclcpp::get_logger("CarlikeBotSystemHardware"), "binding failed %d!", bind_state);
    //   return hardware_interface::CallbackReturn::ERROR;
    // }

    // // 4. Listen
    // // waits for the client to approach the server to make a connection
    // if (listen(server_fd, 3) < 0)
    // {
    //   RCLCPP_ERROR(
    //       rclcpp::get_logger("CarlikeBotSystemHardware"), "listen failed!");
    //   return hardware_interface::CallbackReturn::ERROR;
    // }

    // // 5. Accept
    // // connection is established between client and server, and they are ready to transfer data.
    // if ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t *)&addrlen)) < 0)
    // {
    //   RCLCPP_ERROR(
    //       rclcpp::get_logger("CarlikeBotSystemHardware"), "accept failed!");
    //   return hardware_interface::CallbackReturn::ERROR;
    // }

    // // 6. Connection is established between server and client

    // RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Connection Established between server and client. Now Start read and write from both sides. ENJOY!!!!");

    if ((sfd = serialOpen("/dev/ttyS0", 9600)) < 0)
    {
      fprintf(stderr, "Unable to open serial device: %s\n", strerror(errno));
      RCLCPP_ERROR(
          rclcpp::get_logger("CarlikeBotSystemHardware"), "Error opening serial to stm...");
      return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Connection Established (stm uart). Now Start read and write from both sides. ENJOY!!!!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> CarlikeBotSystemHardware::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (auto &joint : hw_interfaces_)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          joint.second.joint_name, hardware_interface::HW_IF_POSITION, &joint.second.state.position));

      if (joint.first == "traction")
      {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            joint.second.joint_name, hardware_interface::HW_IF_VELOCITY, &joint.second.state.velocity));
      }
    }

    RCLCPP_INFO(
        rclcpp::get_logger("CarlikeBotSystemHardware"), "Exported %zu state interfaces.",
        state_interfaces.size());

    for (auto s : state_interfaces)
    {
      RCLCPP_INFO(
          rclcpp::get_logger("CarlikeBotSystemHardware"), "Exported state interface '%s'.",
          s.get_name().c_str());
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface>
  CarlikeBotSystemHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (auto &joint : hw_interfaces_)
    {
      if (joint.first == "steering")
      {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            joint.second.joint_name, hardware_interface::HW_IF_POSITION,
            &joint.second.command.position));
      }
      else if (joint.first == "traction")
      {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            joint.second.joint_name, hardware_interface::HW_IF_VELOCITY,
            &joint.second.command.velocity));
      }
    }

    RCLCPP_INFO(
        rclcpp::get_logger("CarlikeBotSystemHardware"), "Exported %zu command interfaces.",
        command_interfaces.size());

    for (auto i = 0u; i < command_interfaces.size(); i++)
    {
      RCLCPP_INFO(
          rclcpp::get_logger("CarlikeBotSystemHardware"), "Exported command interface '%s'.",
          command_interfaces[i].get_name().c_str());
    }

    return command_interfaces;
  }

  hardware_interface::CallbackReturn CarlikeBotSystemHardware::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Activating ...please wait...");

    for (auto i = 0; i < hw_start_sec_; i++)
    {
      rclcpp::sleep_for(std::chrono::seconds(1));
      RCLCPP_INFO(
          rclcpp::get_logger("CarlikeBotSystemHardware"), "%.1f seconds left...", hw_start_sec_ - i);
    }

    for (auto &joint : hw_interfaces_)
    {
      joint.second.state.position = 0.0;

      if (joint.first == "traction")
      {
        joint.second.state.velocity = 0.0;
        joint.second.command.velocity = 0.0;
      }

      else if (joint.first == "steering")
      {
        joint.second.command.position = 0.0;
      }
    }

    // connect serial with stm

    RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn CarlikeBotSystemHardware::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Deactivating ...please wait...");

    for (auto i = 0; i < hw_stop_sec_; i++)
    {
      rclcpp::sleep_for(std::chrono::seconds(1));
      RCLCPP_INFO(
          rclcpp::get_logger("CarlikeBotSystemHardware"), "%.1f seconds left...", hw_stop_sec_ - i);
    }
    // END: This part here is for exemplary purposes - Please do not copy to your production code

    // disconnect serial with stm
    // close(new_socket);
    // close(server_fd);

    serialClose(sfd);

    RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type CarlikeBotSystemHardware::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
  {
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code

    hw_interfaces_["steering"].state.position = hw_interfaces_["steering"].command.position;

    hw_interfaces_["traction"].state.velocity = hw_interfaces_["traction"].command.velocity;
    hw_interfaces_["traction"].state.position +=
        hw_interfaces_["traction"].state.velocity * period.seconds();

    // RCLCPP_INFO(
    //     rclcpp::get_logger("CarlikeBotSystemHardware"), "Got position state: %.2f for joint '%s'.",
    //     hw_interfaces_["steering"].command.position, hw_interfaces_["steering"].joint_name.c_str());

    // RCLCPP_INFO(
    //     rclcpp::get_logger("CarlikeBotSystemHardware"), "Got velocity state: %.2f for joint '%s'.",
    //     hw_interfaces_["traction"].command.velocity, hw_interfaces_["traction"].joint_name.c_str());

    // END: This part here is for exemplary purposes - Please do not copy to your production code

    // rear_wheel_joint
    // read encoder values of both two rear wheels-------------------------position
    // read velocity values of both two rear wheels------------------------velocity

    // front_wheel_joint
    // read servo position only position-----------------------------------position

    /*-----------------------------------read data from stm to pi to ros2 to python to cpp plugin-----------------------------------*/

    const int DATA_SIZE = 15;
    const uint8_t START_BYTE = 0xA5;

    uint8_t recv_data[DATA_SIZE];

    while (1)
    {

      if (serialDataAvail(sfd) < DATA_SIZE)
      {
        break;
      }

      ReadState state = WAIT_START_BYTE;
      uint8_t ch = serialGetchar(sfd);

      if (ch == START_BYTE)
      {
        state = READ_REST_DATA;
        for (int i = 1; i < DATA_SIZE; i++)
        {
          recv_data[i] = (uint8_t)serialGetchar(sfd);
        }
      }
    }

    uint8_t hash = crc.get_Hash(recv_data + 1, DATA_SIZE - 3);

    if (hash != recv_data[DATA_SIZE - 2])
    {
      RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Error: hash failed, "
                                                                  "got %d, expected %d",
                  hash, recv_data[DATA_SIZE - 2]);
      return hardware_interface::return_type::OK;
    }

    // always clear the buffer, before receiving data for frame matching
    // memset(received_state, 0, sizeof(received_state));
    // ssize_t valread = ::read(new_socket, &received_state, sizeof(received_state));
    // if (valread < 0)
    // {
    //   RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Error receiving data");
    //   return hardware_interface::return_type::ERROR;
    // }

    // copy received data to car(rear_wheel_count, rear_wheel_position, front_wheel_steering)
    memcpy(&car, (recv_data) + 1, sizeof(car));

    // feedback if no car odometry, you can uncomment this line, your car starts to move in rviz as according to your feedback
    // car.traction_wheel_position += 100.0;
    // car.traction_wheel_velocity = 5.0;
    // car.steering_position = 0.01;

    // update rear_wheel_joint position
    hw_interfaces_["traction"].state.position = car.traction_wheel_position; //-----------------both wheel encoder values
    // update rear_wheel_joint velocity
    hw_interfaces_["traction"].state.velocity = car.traction_wheel_velocity; //-----------------both wheel velocity values
    // update front_wheel_joint position
    hw_interfaces_["steering"].state.position = car.steering_position; //-----------------------front servo steering values

    RCLCPP_INFO(
        rclcpp::get_logger("CarlikeBotSystemHardware"), "Got steering position state: %f for joint '%s', time='%f'.",
        hw_interfaces_["steering"].state.position, hw_interfaces_["steering"].joint_name.c_str(), period.seconds());

    RCLCPP_INFO(
        rclcpp::get_logger("CarlikeBotSystemHardware"), "Got traction velocity state: %.4f for joint '%s'.",
        hw_interfaces_["traction"].state.velocity, hw_interfaces_["traction"].joint_name.c_str());

    // clear the buffer
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type car_gazebo ::CarlikeBotSystemHardware::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    /*-----------------------------------write data from topic /bicycle_steering_controller/reference to cpp plugin to python to ros2 to pi to stm-----------------------------------*/

    // rear_wheel_joint
    // set velocities for both two rear wheel------------------------------velocity
    // front_wheel_joint
    // set servo position for front steering-------------------------------position

    RCLCPP_INFO(
        rclcpp::get_logger("CarlikeBotSystemHardware"), "Got position command: %.2f for joint '%s'.",
        hw_interfaces_["steering"].command.position, hw_interfaces_["steering"].joint_name.c_str());

    RCLCPP_INFO(
        rclcpp::get_logger("CarlikeBotSystemHardware"), "Got velocity command: %.2f for joint '%s'.",
        hw_interfaces_["traction"].command.velocity, hw_interfaces_["traction"].joint_name.c_str());

    // clear the buffer before sending
    // memset(command, 0, sizeof(command));
    float command[] = {
        (float)hw_interfaces_["traction"].command.velocity,
        (float)hw_interfaces_["steering"].command.position};

    uint8_t send_data[10];
    send_data[0] = 0xA5;
    memcpy(send_data + 1, command, sizeof(command));
    send_data[9] = crc.get_Hash(send_data + 1, 8);
    for (int i = 0; i < 10; i++)
    {
      serialPutchar(sfd, send_data[i]);
    }

    // server sends data, client must listen data, done using recv function inside pose_receiver node
    // always send all data, from start to end for frame matching
    // ssize_t bytes_sent = send(new_socket, &command, sizeof(command), 0);
    if (0 /*|| bytes_sent < 0*/)
    {
      RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Error sending data");
      return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
  }
} // namespace car_gazebo

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    car_gazebo::CarlikeBotSystemHardware, hardware_interface::SystemInterface)
