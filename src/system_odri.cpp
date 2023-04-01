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

#include "system_odri.hpp"

#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <system_interface_odri.hpp>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

/*Connection to ODRI for read sensors and write commands*/
#include "odri_control_interface/imu.hpp"
#include "odri_control_interface/utils.hpp"

using namespace odri_control_interface;
using namespace Eigen;
using namespace semantic_components;

#include <iostream>
#include <stdexcept>

namespace ros2_control_odri {

/* Code issue from demo_odri_actuator_control.cpp (ODRI)*/

Eigen::Vector6d desired_joint_position = Eigen::Vector6d::Zero();
Eigen::Vector6d desired_torque = Eigen::Vector6d::Zero();

hardware_interface::return_type SystemOdriHardware::read_default_cmd_state_value(
    std::string &default_joint_cs) {
  // Hardware parameters provides a string
  if (info_.hardware_parameters.find(default_joint_cs) ==
      info_.hardware_parameters.end()) {
    RCLCPP_FATAL(
        rclcpp::get_logger("SystemOdriHardware"),
        "%s not in the parameter list of ros2_control_odri/SystemOdriHardware!",
        default_joint_cs.c_str());
    return hardware_interface::return_type::ERROR;
  }
  std::string str_des_start_pos = info_.hardware_parameters[default_joint_cs];

  typedef std::map<std::string, PosVelEffortGains> map_pveg;
  map_pveg hw_cs;
  if (default_joint_cs == "default_joint_cmd") {
    hw_cs = hw_commands_;
  } else if (default_joint_cs == "default_joint_state") {
    hw_cs = hw_states_;
  } else
    return hardware_interface::return_type::ERROR;

  // Read the parameter through a stream of strings.
  std::istringstream iss_def_cmd_val;
  iss_def_cmd_val.str(str_des_start_pos);

  while (!iss_def_cmd_val.eof()) {
    // Find joint name.
    std::string joint_name;
    RCLCPP_INFO_STREAM(
        rclcpp::get_logger("SystemOdriHardware"),
        " Current value of iss_def_cmd_val:" << iss_def_cmd_val.str().c_str());
    iss_def_cmd_val >> joint_name;

    // Find the associate joint
    bool found_joint = false;
    for (const hardware_interface::ComponentInfo &joint : info_.joints) {
      if (joint.name == joint_name) {
        auto handle_dbl_and_msg = [](std::istringstream &iss_def_cmd_val,
                                     std::string &joint_name, double &adbl,
                                     std::string &msg) {
          if (!iss_def_cmd_val.eof())
            iss_def_cmd_val >> adbl;
          else {
            RCLCPP_FATAL(rclcpp::get_logger("SystemOdriHardware"),
                         "default_joint_cmd '%s' no '%s'.", joint_name.c_str(),
                         msg.c_str());
            return hardware_interface::return_type::ERROR;
          }
          return hardware_interface::return_type::OK;
        };

        std::string amsg("position");
        if (handle_dbl_and_msg(iss_def_cmd_val, joint_name,
                               hw_cs.at(joint_name).position,
                               amsg) == hardware_interface::return_type::ERROR)
          return hardware_interface::return_type::ERROR;

        amsg = "velocity";
        if (handle_dbl_and_msg(iss_def_cmd_val, joint_name,
                               hw_cs.at(joint_name).velocity,
                               amsg) == hardware_interface::return_type::ERROR)
          return hardware_interface::return_type::ERROR;

        amsg = "effort";
        if (handle_dbl_and_msg(iss_def_cmd_val, joint_name,
                               hw_cs.at(joint_name).effort,
                               amsg) == hardware_interface::return_type::ERROR)
          return hardware_interface::return_type::ERROR;

        amsg = "Kp";
        if (handle_dbl_and_msg(iss_def_cmd_val, joint_name,
                               hw_cs.at(joint_name).Kp,
                               amsg) == hardware_interface::return_type::ERROR)
          return hardware_interface::return_type::ERROR;

        amsg = "Kd";
        if (handle_dbl_and_msg(iss_def_cmd_val, joint_name,
                               hw_cs.at(joint_name).Kd,
                               amsg) == hardware_interface::return_type::ERROR)
          return hardware_interface::return_type::ERROR;

        found_joint = true;
        break;  // Found the joint break the loop
      }
    }

    if (!found_joint) {
      RCLCPP_FATAL(rclcpp::get_logger("SystemOdriHardware"),
                   "Joint '%s' not found in '%s'.", joint_name.c_str(),
                   default_joint_cs.c_str());
      return hardware_interface::return_type::ERROR;
    }
  }
  if (default_joint_cs == "default_joint_cmd")
    hw_commands_ = hw_cs;
  else if (default_joint_cs == "default_joint_state")
    hw_states_ = hw_cs;

  return hardware_interface::return_type::OK;
}

/// Reading desired position
hardware_interface::return_type SystemOdriHardware::read_desired_starting_position() {
  std::vector<double> vec_des_start_pos;

  if (info_.hardware_parameters.find("desired_starting_position") ==
      info_.hardware_parameters.end()) {
    RCLCPP_FATAL(rclcpp::get_logger("SystemOdriHardware"),
                 "desired_starting_positon not in the parameter list of "
                 "ros2_control_odri/SystemOdriHardware!");
    return hardware_interface::return_type::ERROR;
  }

  // Hardware parameters provides a string
  std::string str_des_start_pos =
      info_.hardware_parameters["desired_starting_position"];

  // Read the parameter through a stream of strings.
  std::istringstream iss_des_start_pos;
  iss_des_start_pos.str(str_des_start_pos);

  RCLCPP_INFO_STREAM(
      rclcpp::get_logger("SystemOdriHardware"),
      " Current desired_start_position size:" << eig_des_start_pos_.size());

  // From istringstream to std::vector
  while (!iss_des_start_pos.eof()) {
    double apos;
    iss_des_start_pos >> apos;
    vec_des_start_pos.push_back(apos);
  }
  eig_des_start_pos_.resize(vec_des_start_pos.size());

  RCLCPP_INFO_STREAM(
      rclcpp::get_logger("SystemOdriHardware"),
      " Current desired_start_position size:" << eig_des_start_pos_.size());

  // From std::vector to VectorXd
  int idx_dsp = 0;
  for (auto apos : vec_des_start_pos) eig_des_start_pos_[idx_dsp++] = apos;

  return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn SystemOdriHardware::on_init(
    const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // For each sensor.
  for (const hardware_interface::ComponentInfo &sensor : info_.sensors) {
    imu_states_[sensor.name] = {std::numeric_limits<double>::quiet_NaN(),
                                std::numeric_limits<double>::quiet_NaN(),
                                std::numeric_limits<double>::quiet_NaN(),
                                std::numeric_limits<double>::quiet_NaN(),
                                std::numeric_limits<double>::quiet_NaN(),
                                std::numeric_limits<double>::quiet_NaN(),
                                std::numeric_limits<double>::quiet_NaN(),
                                std::numeric_limits<double>::quiet_NaN(),
                                std::numeric_limits<double>::quiet_NaN(),
                                std::numeric_limits<double>::quiet_NaN(),
                                std::numeric_limits<double>::quiet_NaN(),
                                std::numeric_limits<double>::quiet_NaN(),
                                std::numeric_limits<double>::quiet_NaN(),
                                std::numeric_limits<double>::quiet_NaN(),
                                std::numeric_limits<double>::quiet_NaN(),
                                std::numeric_limits<double>::quiet_NaN()};
  }
  // For each joint.
  for (const hardware_interface::ComponentInfo &joint : info_.joints) {
    // Initialize state of the joint by default to NaN
    // it allows to see which joints are not properly initialized
    // from the real hardware
    hw_states_[joint.name] = {std::numeric_limits<double>::quiet_NaN(),
                              std::numeric_limits<double>::quiet_NaN(),
                              std::numeric_limits<double>::quiet_NaN(),
                              std::numeric_limits<double>::quiet_NaN(),
                              std::numeric_limits<double>::quiet_NaN()};
    hw_commands_[joint.name] = {std::numeric_limits<double>::quiet_NaN(),
                                std::numeric_limits<double>::quiet_NaN(),
                                std::numeric_limits<double>::quiet_NaN(),
                                std::numeric_limits<double>::quiet_NaN(),
                                std::numeric_limits<double>::quiet_NaN()};
    control_mode_[joint.name] = control_mode_t::NO_VALID_MODE;

    // SystemOdri has exactly 5 doubles for the state and
    // 5 doubles for the command interface on each joint
    if (joint.command_interfaces.size() != odri_list_of_cmd_inter.size()) {
      RCLCPP_FATAL(
          rclcpp::get_logger("SystemOdriHardware"),
          "Joint '%s' has %lu command interfaces found.",  // 5 expected.",
          joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // For each command interface of the joint
    for (const auto &a_joint_cmd_inter : joint.command_interfaces) {
      // Check if the command interface is inside the list
      if (odri_list_of_cmd_inter.find(a_joint_cmd_inter.name) ==
          odri_list_of_cmd_inter.end()) {
        // If not then generate an error message
        RCLCPP_FATAL(rclcpp::get_logger("SystemOdriHardware"),
                     "Joint '%s' have %s command interfaces found. One of the "
                     "following values is expected",
                     joint.name.c_str(), a_joint_cmd_inter.name.c_str());
        for (const auto &a_cmd_inter : odri_list_of_cmd_inter) {
          RCLCPP_FATAL(rclcpp::get_logger("SystemOdriHardware"),
                       "'%s' expected.", a_cmd_inter.c_str());
        }
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    // Check if the state interface list is of the right size
    if (joint.state_interfaces.size() != odri_list_of_state_inter.size()) {
      RCLCPP_FATAL(rclcpp::get_logger("SystemOdriHardware"),
                   "Joint '%s' has %lu state interface.",  // 5 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // For each state interface of the joint
    for (const auto &a_joint_state_inter : joint.state_interfaces) {
      std::string joint_state_inter_name = a_joint_state_inter.name;

      // Check if the state interface is inside the list
      if (odri_list_of_state_inter.find(joint_state_inter_name) ==
          odri_list_of_state_inter.end()) {
        RCLCPP_FATAL(rclcpp::get_logger("SystemOdriHardware"),
                     "Joint '%s' have %s state interface. One of the following "
                     "was expected: ",
                     joint.name.c_str(), a_joint_state_inter.name.c_str());

        for (const auto &a_state_inter : odri_list_of_state_inter) {
          RCLCPP_FATAL(rclcpp::get_logger("SystemOdriHardware"),
                       "'%s' expected.", a_state_inter.c_str());
        }
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

void SystemOdriHardware::display_robot_state() {
  for (const hardware_interface::ComponentInfo &joint : info_.joints) {
    std::cout << "joint " << joint.name << " "
              << hw_commands_[joint.name].position << " "
              << hw_commands_[joint.name].velocity << " "
              << hw_commands_[joint.name].effort << " "
              << hw_commands_[joint.name].Kp << " "
              << hw_commands_[joint.name].Kd << std::endl;
  }
  std::cout << " **************************" << std::endl;
}

hardware_interface::return_type SystemOdriHardware::prepare_command_mode_switch(
    const std::vector<std::string> &start_interfaces,
    const std::vector<std::string> &stop_interfaces) {
  // Initialize new modes.
  for (const hardware_interface::ComponentInfo &joint : info_.joints) {
    new_modes_[joint.name] = control_mode_t::NO_VALID_MODE;
  }

  /// Check that the key interfaces are coherent
  for (auto &key : start_interfaces) {
    /// For each joint
    for (const hardware_interface::ComponentInfo &joint : info_.joints) {
      if (key == joint.name + "/" + hardware_interface::HW_IF_POSITION) {
        new_modes_[joint.name] = control_mode_t::POSITION;
      }

      if (key == joint.name + "/" + hardware_interface::HW_IF_VELOCITY) {
        new_modes_[joint.name] = control_mode_t::VELOCITY;
      }

      if (key == joint.name + "/" + hardware_interface::HW_IF_EFFORT) {
        new_modes_[joint.name] = control_mode_t::EFFORT;
      }

      if (key == joint.name + "/" + ros2_control_odri::HW_IF_GAIN_KP) {
        new_modes_[joint.name] = control_mode_t::POS_VEL_EFF_GAINS;
      }
      if (key == joint.name + "/" + ros2_control_odri::HW_IF_GAIN_KD) {
        new_modes_[joint.name] = control_mode_t::POS_VEL_EFF_GAINS;
      }
    }
  }
  // Stop motion on all relevant joints that are stopping
  for (std::string key : stop_interfaces) {
    for (const hardware_interface::ComponentInfo &joint : info_.joints) {
      if (key.find(joint.name) != std::string::npos) {
        hw_commands_[joint.name].velocity = 0.0;
        hw_commands_[joint.name].effort = 0.0;
        control_mode_[joint.name] =
            control_mode_t::NO_VALID_MODE;  // Revert to undefined
      }
    }
  }
  // Set the new command modes
  for (const hardware_interface::ComponentInfo &joint : info_.joints) {
    if ((control_mode_[joint.name] == control_mode_t::NO_VALID_MODE) &&
        (new_modes_[joint.name] == control_mode_t::NO_VALID_MODE)) {
      // Something else is using the joint! Abort!
      RCLCPP_ERROR(rclcpp::get_logger("SystemOdriHardware"),
                   "Joint '%s' has no valid control mode %d %d",
                   joint.name.c_str(), control_mode_[joint.name],
                   new_modes_[joint.name]);
      return hardware_interface::return_type::ERROR;
    }
    control_mode_[joint.name] = new_modes_[joint.name];
  }

  std::cout << "in prepare_command_mode_switch" << std::endl;
  display_robot_state();
  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface>
SystemOdriHardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (const hardware_interface::ComponentInfo &joint : info_.joints) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint.name, hardware_interface::HW_IF_POSITION,
        &hw_states_[joint.name].position));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint.name, hardware_interface::HW_IF_VELOCITY,
        &hw_states_[joint.name].velocity));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint.name, hardware_interface::HW_IF_EFFORT,
        &hw_states_[joint.name].effort));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint.name, HW_IF_GAIN_KP, &hw_states_[joint.name].Kp));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint.name, HW_IF_GAIN_KD, &hw_states_[joint.name].Kd));
  }

  for (const hardware_interface::ComponentInfo &sensor : info_.sensors) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        sensor.name, "gyroscope_x", &imu_states_[sensor.name].gyro_x));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        sensor.name, "gyroscope_y", &imu_states_[sensor.name].gyro_y));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        sensor.name, "gyroscope_z", &imu_states_[sensor.name].gyro_z));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        sensor.name, "accelerometer_x", &imu_states_[sensor.name].accelero_x));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        sensor.name, "accelerometer_y", &imu_states_[sensor.name].accelero_y));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        sensor.name, "accelerometer_z", &imu_states_[sensor.name].accelero_z));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        sensor.name, "linear_acceleration_x",
        &imu_states_[sensor.name].line_acc_x));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        sensor.name, "linear_acceleration_y",
        &imu_states_[sensor.name].line_acc_y));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        sensor.name, "linear_acceleration_z",
        &imu_states_[sensor.name].line_acc_z));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        sensor.name, "attitude_euler_x", &imu_states_[sensor.name].euler_x));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        sensor.name, "attitude_euler_y", &imu_states_[sensor.name].euler_y));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        sensor.name, "attitude_euler_z", &imu_states_[sensor.name].euler_z));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(sensor.name, "attitude_quaternion_x",
                                           &imu_states_[sensor.name].quater_x));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(sensor.name, "attitude_quaternion_y",
                                           &imu_states_[sensor.name].quater_y));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(sensor.name, "attitude_quaternion_z",
                                           &imu_states_[sensor.name].quater_z));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(sensor.name, "attitude_quaternion_w",
                                           &imu_states_[sensor.name].quater_w));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
SystemOdriHardware::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (const hardware_interface::ComponentInfo &joint : info_.joints) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint.name, hardware_interface::HW_IF_POSITION,
        &hw_commands_[joint.name].position));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint.name, hardware_interface::HW_IF_VELOCITY,
        &hw_commands_[joint.name].velocity));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint.name, hardware_interface::HW_IF_EFFORT,
        &hw_commands_[joint.name].effort));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint.name, HW_IF_GAIN_KP, &hw_commands_[joint.name].Kp));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint.name, HW_IF_GAIN_KD, &hw_commands_[joint.name].Kd));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn SystemOdriHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  //// Read Parameters ////

  /// Read odri_config_yaml
  // Initialize Robot
  robot_ = RobotFromYamlFile(info_.hardware_parameters["odri_config_yaml"]);

  /// Read hardware parameter "desired_starting_position"
  if (read_desired_starting_position() == hardware_interface::return_type::ERROR)
    return hardware_interface::CallbackReturn::ERROR;

  /// Read hardware parameter "default_joint_cmd"
  std::string default_joint_cs("default_joint_cmd");
  if (read_default_cmd_state_value(default_joint_cs) == hardware_interface::return_type::ERROR)
    return hardware_interface::CallbackReturn::ERROR;

  /// Read hardware parameter "default_joint_state"
  default_joint_cs = "default_joint_state";
  if (read_default_cmd_state_value(default_joint_cs) == hardware_interface::return_type::ERROR)
    return hardware_interface::CallbackReturn::ERROR;

  /// Initialize the robot to the desired starting position.
  robot_->Initialize(eig_des_start_pos_);

  /// Build the map from name to index.
  for (const hardware_interface::ComponentInfo &joint : info_.joints) {
    // First set the key
    joint_name_to_array_index_[joint.name] = 0;
  }

  // Then build the index.
  uint idx = 0;
  for (auto it = joint_name_to_array_index_.begin();
       it != joint_name_to_array_index_.end(); ++it) {
    joint_name_to_array_index_[it->first] = idx++;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SystemOdriHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
  // Stop the MasterBoard
  main_board_ptr_->MasterBoardInterface::Stop();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type SystemOdriHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  // Data acquisition (with ODRI)
  robot_->ParseSensorData();

  auto sensor_positions = robot_->joints->GetPositions();
  auto sensor_velocities = robot_->joints->GetVelocities();
  auto measured_torques = robot_->joints->GetMeasuredTorques();

  auto imu_gyroscope = robot_->imu->GetGyroscope();
  auto imu_accelero = robot_->imu->GetAccelerometer();
  auto imu_linear_acc = robot_->imu->GetLinearAcceleration();
  auto imu_euler = robot_->imu->GetAttitudeEuler();
  auto imu_quater = robot_->imu->GetAttitudeQuaternion();

  // Assignment of sensor data to ros2_control variables (URDF)
  for (const hardware_interface::ComponentInfo &joint : info_.joints) {
    hw_states_[joint.name].position =
        sensor_positions[joint_name_to_array_index_[joint.name]];
    hw_states_[joint.name].velocity =
        sensor_velocities[joint_name_to_array_index_[joint.name]];
    hw_states_[joint.name].effort =
        measured_torques[joint_name_to_array_index_[joint.name]];
    hw_states_[joint.name].Kp = hw_commands_[joint.name].Kp;
    hw_states_[joint.name].Kd = hw_commands_[joint.name].Kd;
  }

  // Assignment of IMU data (URDF)
  // Modif with for loop possible to optimize the code
  imu_states_["IMU"].gyro_x = imu_gyroscope[0];
  imu_states_["IMU"].gyro_z = imu_gyroscope[1];
  imu_states_["IMU"].gyro_y = imu_gyroscope[2];

  imu_states_["IMU"].accelero_x = imu_accelero[0];
  imu_states_["IMU"].accelero_y = imu_accelero[1];
  imu_states_["IMU"].accelero_z = imu_accelero[2];

  imu_states_["IMU"].line_acc_x = imu_linear_acc[0];
  imu_states_["IMU"].line_acc_y = imu_linear_acc[1];
  imu_states_["IMU"].line_acc_z = imu_linear_acc[2];

  imu_states_["IMU"].euler_x = imu_euler[0];
  imu_states_["IMU"].euler_y = imu_euler[1];
  imu_states_["IMU"].euler_z = imu_euler[2];

  imu_states_["IMU"].quater_x = imu_quater[0];
  imu_states_["IMU"].quater_y = imu_quater[1];
  imu_states_["IMU"].quater_z = imu_quater[2];
  imu_states_["IMU"].quater_w = imu_quater[3];

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SystemOdriHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  Eigen::Vector6d positions;
  Eigen::Vector6d velocities;
  Eigen::Vector6d torques;

  Eigen::Vector6d gain_KP;
  Eigen::Vector6d gain_KD;

  for (const hardware_interface::ComponentInfo &joint : info_.joints) {
    if ((control_mode_[joint.name] == control_mode_t::POS_VEL_EFF_GAINS) ||
        (control_mode_[joint.name] == control_mode_t::POSITION)) {
      positions[joint_name_to_array_index_[joint.name]] =
          hw_commands_[joint.name].position;
      velocities[joint_name_to_array_index_[joint.name]] =
          hw_commands_[joint.name].velocity;
      torques[joint_name_to_array_index_[joint.name]] =
          hw_commands_[joint.name].effort;
      gain_KP[joint_name_to_array_index_[joint.name]] =
          hw_commands_[joint.name].Kp;
      gain_KD[joint_name_to_array_index_[joint.name]] =
          hw_commands_[joint.name].Kd;
    }
  }

  static unsigned int my_perso_counter2 = 0;
  if (my_perso_counter2 % 1000 == 0) {
    std::cout << "positions:" << positions.transpose() << std::endl;
    std::cout << "velocities:" << velocities.transpose() << std::endl;
    std::cout << "torques: " << torques.transpose() << std::endl;
    std::cout << "gain_KP: " << gain_KP.transpose() << std::endl;
    std::cout << "gain_KD: " << gain_KD.transpose() << std::endl;
    std::cout << " " << std::endl;
  }
  ++my_perso_counter2;

  robot_->joints->SetDesiredPositions(positions);
  robot_->joints->SetDesiredVelocities(velocities);
  robot_->joints->SetTorques(torques);
  robot_->joints->SetPositionGains(gain_KP);
  robot_->joints->SetVelocityGains(gain_KD);

  robot_->SendCommandAndWaitEndOfCycle(0.00);

  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_odri

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ros2_control_odri::SystemOdriHardware,
                       hardware_interface::SystemInterface)
