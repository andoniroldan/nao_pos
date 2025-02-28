// Copyright 2024 Antonio Bono
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

#ifndef NAO_POS_SERVER__NAO_POS_ACTION_SERVER_HPP_
#define NAO_POS_SERVER__NAO_POS_ACTION_SERVER_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/node.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nao_lola_command_msgs/msg/joint_positions.hpp"
#include "nao_lola_command_msgs/msg/joint_stiffnesses.hpp"
#include "nao_lola_command_msgs/msg/joint_indexes.hpp"
#include "nao_lola_sensor_msgs/msg/joint_positions.hpp"

#include "nao_pos_interfaces/action/pos_play.hpp"
#include "nao_pos_server/key_frame.hpp"

#include "std_msgs/msg/string.hpp"

namespace nao_pos_action_server_ns
{

class NaoPosActionServer : public rclcpp::Node
{
public:
  explicit NaoPosActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});
  virtual ~NaoPosActionServer();

private:
  std::string getFullFilePath(std::string& filename);
  std::vector<std::string> readLines(std::ifstream& ifstream);
  void calculateEffectorJoints(nao_lola_sensor_msgs::msg::JointPositions& sensor_joints);
  const KeyFrame& findPreviousKeyFrame(int time_ms);
  const KeyFrame& findNextKeyFrame(int time_ms);
  bool posFinished(int time_ms);
  void readPosFile(std::string& filePath);
  float findElem(const std::vector<uint8_t>& indexes, const std::vector<float>& data, uint8_t joint);

  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID& uuid,
                                         std::shared_ptr<const nao_pos_interfaces::action::PosPlay::Goal> goal);
  rclcpp_action::CancelResponse
  handleCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<nao_pos_interfaces::action::PosPlay>> goal_handle);
  void handleAccepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<nao_pos_interfaces::action::PosPlay>> goal_handle);

  rclcpp::Subscription<nao_lola_sensor_msgs::msg::JointPositions>::SharedPtr sub_joint_states_;
  rclcpp::Publisher<nao_lola_command_msgs::msg::JointPositions>::SharedPtr pub_joint_positions_;
  rclcpp::Publisher<nao_lola_command_msgs::msg::JointStiffnesses>::SharedPtr pub_joint_stiffnesses_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_action_status_;

  rclcpp_action::Server<nao_pos_interfaces::action::PosPlay>::SharedPtr action_server_;

  bool file_successfully_read_ = false;
  std::vector<KeyFrame> key_frames_;
  std::atomic<bool> pos_in_action_;
  bool firstTickSinceActionStarted_ = true;
  std::unique_ptr<KeyFrame> key_frame_start_;
  rclcpp::Time initial_time_;
  std::vector<uint8_t> selected_joints_;

  std::shared_ptr<rclcpp_action::ServerGoalHandle<nao_pos_interfaces::action::PosPlay>> goal_handle_;

  std::mutex mutex_;
};

}  // namespace nao_pos_action_server_ns

#endif  // NAO_POS_SERVER__NAO_POS_SERVER_HPP_
