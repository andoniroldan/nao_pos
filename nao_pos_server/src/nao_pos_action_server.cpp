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

#include "nao_pos_server/nao_pos_action_server.hpp"

#include <algorithm>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <fstream>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "boost/filesystem.hpp"
#include "indexes.hpp"
#include "parser.hpp"
#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"

namespace fs = boost::filesystem;

namespace nao_pos_action_server_ns
{

NaoPosActionServer::NaoPosActionServer(const rclcpp::NodeOptions & options)
: rclcpp::Node{"nao_pos_action_server_node", options}, pos_in_action_(false)
{
  pub_joint_positions_ = create_publisher<nao_lola_command_msgs::msg::JointPositions>(
    "/effectors/joint_positions", rclcpp::SensorDataQoS());
  pub_joint_stiffnesses_ = create_publisher<nao_lola_command_msgs::msg::JointStiffnesses>(
    "/effectors/joint_stiffnesses", rclcpp::SensorDataQoS());

  sub_joint_states_ = create_subscription<nao_lola_sensor_msgs::msg::JointPositions>(
    "/sensors/joint_positions", rclcpp::SensorDataQoS(),
    [this](nao_lola_sensor_msgs::msg::JointPositions::SharedPtr sensor_joints) {
      if (pos_in_action_) {
        calculateEffectorJoints(*sensor_joints);
      }
    });

  action_server_ = rclcpp_action::create_server<nao_pos_interfaces::action::PosPlay>(
    this, "nao_pos_action",
    std::bind(&NaoPosActionServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&NaoPosActionServer::handleCancel, this, std::placeholders::_1),
    std::bind(&NaoPosActionServer::handleAccepted, this, std::placeholders::_1));

  pub_action_status_ = create_publisher<std_msgs::msg::String>(
    "/nao_pos_action/status", 10);

  RCLCPP_INFO(this->get_logger(), "nao_pos_action_server_node initialized");
}

NaoPosActionServer::~NaoPosActionServer() {}

void NaoPosActionServer::readPosFile(std::string & filePath)
{
  std::ifstream ifstream(filePath);
  if (ifstream.is_open()) {
    RCLCPP_DEBUG(this->get_logger(), ("Pos file succesfully loaded from " + filePath).c_str());
    file_successfully_read_ = true;
    auto lines = readLines(ifstream);
    auto parseResult = parser::parse(lines);
    file_successfully_read_ = parseResult.successful;
    key_frames_ = parseResult.keyFrames;
  } else {
    RCLCPP_ERROR(this->get_logger(), ("Could not open file:  " + filePath).c_str());
    file_successfully_read_ = false;
  }
}

std::string NaoPosActionServer::getFullFilePath(std::string & filename)
{
  std::string file = "pos/" + filename;
  std::string package_share_directory =
    ament_index_cpp::get_package_share_directory("nao_pos_server");

  fs::path dir_path(package_share_directory);
  fs::path file_path(file);
  fs::path full_path = dir_path / file_path;
  return full_path.string();
}

std::vector<std::string> NaoPosActionServer::readLines(std::ifstream & ifstream)
{
  std::vector<std::string> ret;

  while (!ifstream.eof()) {
    std::string line;
    std::getline(ifstream, line);
    ret.push_back(line);
  }

  return ret;
}

float NaoPosActionServer::findElem(
  const std::vector<uint8_t> & indexes, const std::vector<float> & data, uint8_t joint)
{
  for (uint8_t a = 0; a < indexes.size(); a++) {
    if (indexes.at(a) == joint) {
      return data.at(a);
    }
  }
  return NAN;
}

void NaoPosActionServer::calculateEffectorJoints(
  nao_lola_sensor_msgs::msg::JointPositions & sensor_joints)
{
  //std::lock_guard<std::mutex> lock(mutex_);

  if (goal_handle_->is_canceling()) {
    auto result = std::make_shared<nao_pos_interfaces::action::PosPlay::Result>();
    result->success = false;
    goal_handle_->canceled(result);
    RCLCPP_DEBUG(this->get_logger(), "pos action goal canceled");
    return;
  }

  int time_ms = (rclcpp::Node::now() - initial_time_).nanoseconds() / 1e6;

  if (posFinished(time_ms)) {
    // We've finished the motion, set to DONE
    pos_in_action_ = false;
    auto result = std::make_shared<nao_pos_interfaces::action::PosPlay::Result>();
    result->success = true;
    goal_handle_->succeed(result);

    std_msgs::msg::String msg;
    msg.data = goal_handle_->get_goal()->action_name + " succeeded";
    pub_action_status_->publish(msg);

    RCLCPP_INFO(this->get_logger(), "Action finished: %s", goal_handle_->get_goal()->action_name.c_str());
    return;
  }

  if (firstTickSinceActionStarted_) {
    nao_lola_command_msgs::msg::JointPositions command;
    command.indexes = indexes::indexes;
    command.positions =
      std::vector<float>(sensor_joints.positions.begin(), sensor_joints.positions.end());
    key_frame_start_ =
      std::make_unique<KeyFrame>(0, command, nao_lola_command_msgs::msg::JointStiffnesses{});
  }

  const auto & previousKeyFrame = findPreviousKeyFrame(time_ms);
  const auto & nextKeyFrame = findNextKeyFrame(time_ms);

  if (firstTickSinceActionStarted_) {
    for (auto i : nextKeyFrame.positions.indexes) {
      selected_joints_.push_back(i);
    }
    firstTickSinceActionStarted_ = false;

    RCLCPP_DEBUG(this->get_logger(), "first tick false");
    /*std::string s1="";
    for (unsigned c = 0; c < selected_joints_.size(); c++) {
        s1.append( std::to_string(selected_joints_.at(c))+", " );
    }
    RCLCPP_INFO(this->get_logger(), ("selected_joints_: "+s1).c_str() );*/
  }

  float timeFromPreviousKeyFrame = time_ms - previousKeyFrame.t_ms;
  float timeToNextKeyFrame = nextKeyFrame.t_ms - time_ms;
  float duration = timeFromPreviousKeyFrame + timeToNextKeyFrame;

  RCLCPP_DEBUG(
    this->get_logger(), ("timeFromPreviousKeyFrame, timeToNextKeyFrame, duration: " +
                         std::to_string(timeFromPreviousKeyFrame) + ", " +
                         std::to_string(timeToNextKeyFrame) + ", " + std::to_string(duration))
                          .c_str());

  float alpha = timeToNextKeyFrame / duration;  // normalized coefficent k of convex combination
  float beta = timeFromPreviousKeyFrame / duration;  // normalized 1-k

  RCLCPP_DEBUG(
    this->get_logger(),
    ("alpha, beta: " + std::to_string(alpha) + ", " + std::to_string(beta)).c_str());

  nao_lola_command_msgs::msg::JointPositions effector_joints;
  nao_lola_command_msgs::msg::JointStiffnesses effector_joints_stiff;

  float nextPos = NAN, previousPos = NAN, nextStiff = NAN;
  float tmp = NAN;

  for (uint8_t i : selected_joints_) {
    nextPos = this->findElem(nextKeyFrame.positions.indexes, nextKeyFrame.positions.positions, i);
    nextStiff =
      this->findElem(nextKeyFrame.stiffnesses.indexes, nextKeyFrame.stiffnesses.stiffnesses, i);
    previousPos =
      this->findElem(previousKeyFrame.positions.indexes, previousKeyFrame.positions.positions, i);

    tmp = previousPos * alpha + nextPos * beta;
    if (tmp != NAN) {
      effector_joints.indexes.push_back(i);
      effector_joints.positions.push_back(tmp);
    } else {
      RCLCPP_ERROR(this->get_logger(), "NAN position");
      return;
    }
    if (nextStiff != NAN) {
      effector_joints_stiff.indexes.push_back(i);
      effector_joints_stiff.stiffnesses.push_back(nextStiff);
    } else {
      RCLCPP_ERROR(this->get_logger(), "NAN stiffness");
      return;
    }

    nextPos = NAN, previousPos = NAN, nextStiff = NAN;
  }

  pub_joint_positions_->publish(effector_joints);
  pub_joint_stiffnesses_->publish(effector_joints_stiff);
  RCLCPP_DEBUG(
    this->get_logger(), "published to /effectors/joint_positions and /effectors/joint_stiffnesses");
}

const KeyFrame & NaoPosActionServer::findPreviousKeyFrame(int time_ms)
{
  for (auto it = key_frames_.rbegin(); it != key_frames_.rend(); ++it) {
    const auto & keyFrame = *it;
    int keyFrameDeadline = keyFrame.t_ms;
    if (time_ms >= keyFrameDeadline) {
      return keyFrame;
    }
  }

  return *key_frame_start_;
}

const KeyFrame & NaoPosActionServer::findNextKeyFrame(int time_ms)
{
  for (const auto & keyFrame : key_frames_) {
    int keyFrameDeadline = keyFrame.t_ms;
    if (time_ms < keyFrameDeadline) {
      return keyFrame;
    }
  }

  RCLCPP_ERROR(this->get_logger(), "findKeyFrame: Should never reach here");
  return key_frames_.back();
}

bool NaoPosActionServer::posFinished(int time_ms)
{
  if (key_frames_.size() == 0) {
    return true;
  }

  const auto lastKeyFrame = key_frames_.back();
  int lastKeyFrameTime = lastKeyFrame.t_ms;
  if (time_ms >= lastKeyFrameTime) {
    return true;
  }

  return false;
}

rclcpp_action::GoalResponse NaoPosActionServer::handleGoal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const nao_pos_interfaces::action::PosPlay::Goal> goal)
{
  std::lock_guard<std::mutex> lock(mutex_);

  RCLCPP_INFO(get_logger(), ("Received goal request for:  " + goal->action_name).c_str());
  (void)uuid;
  (void)goal;

  if (!pos_in_action_) {
    std::string filename = goal->action_name + ".pos";
    std::string path = getFullFilePath(filename);
    readPosFile(path);
    RCLCPP_INFO(get_logger(), ("found pos file:  " + path).c_str());
    if (file_successfully_read_) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
  }

  return rclcpp_action::GoalResponse::REJECT;
}

rclcpp_action::CancelResponse NaoPosActionServer::handleCancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<nao_pos_interfaces::action::PosPlay>>
    goal_handle)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RCLCPP_INFO(get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  pos_in_action_ = false;
  goal_handle_.reset();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void NaoPosActionServer::handleAccepted(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<nao_pos_interfaces::action::PosPlay>>
    goal_handle)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RCLCPP_INFO(this->get_logger(), "Starting Pos Action");
  std_msgs::msg::String status_msg;
  status_msg.data = "Action started: " + goal_handle->get_goal()->action_name;
  pub_action_status_->publish(status_msg);
  initial_time_ = rclcpp::Node::now();
  pos_in_action_ = true;
  firstTickSinceActionStarted_ = true;
  selected_joints_.clear();  // std::vector<uint8_t>
  goal_handle_ = goal_handle;
}

}  // namespace nao_pos_action_server_ns

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nao_pos_action_server_ns::NaoPosActionServer)
