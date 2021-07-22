/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <mutex>
#include <chrono>
#include <functional>

#include <rclcpp/rclcpp.hpp>

#include <rmf_traffic_ros2/schedule/MirrorManager.hpp>
#include <rmf_traffic_ros2/StandardNames.hpp>
#include <rmf_traffic_ros2/Time.hpp>
#include <rmf_traffic_ros2/Trajectory.hpp>

#include <rmf_traffic/schedule/Query.hpp>

#include "ScheduleDataNode.hpp"

namespace rmf_massrobotics {
namespace websocket_bridge {

//==============================================================================
class ScheduleDataNode::Implementation
{
public:

  struct Data
  {
    rmf_traffic_ros2::schedule::MirrorManager mirror;

    Data(rmf_traffic_ros2::schedule::MirrorManager mirror_)
    : mirror(std::move(mirror_))
    {
      // Do nothing
    }
  };

  Implementation()
  {}

  Implementation(const Implementation&)
  {}

  std::shared_ptr<rclcpp::Node> node;

  void timer_callback_fn();

  void start(Data data, uint32_t period_msec);

  uint16_t port;

  mutable std::mutex mutex;

private:
  std::unique_ptr<Data> _data;

  rclcpp::TimerBase::SharedPtr _timer;
};

//==============================================================================
void ScheduleDataNode::Implementation::timer_callback_fn()
{
  if (!node)
    return;

  // hardcoded values for now
  const std::string map_name = "office";
  const rmf_traffic::Time time_now =
    rmf_traffic_ros2::convert(node->get_clock()->now());
  const rmf_traffic::Time end_time = time_now + std::chrono::milliseconds(1000);

  // make a query starting now, get a view
  auto query =
    rmf_traffic::schedule::make_query({map_name}, &time_now, &end_time);
  //   rmf_traffic::schedule::make_query({map_name}, &time_now, nullptr);
  const auto view = _data->mirror.viewer().query(query);

  // handle every element in the view
  // for (const auto& elem : view)
  // {
  //    
  // }

  RCLCPP_INFO(
    node->get_logger(),
    "Found %d elements.",
    static_cast<int>(view.size()));
}

//==============================================================================
void ScheduleDataNode::Implementation::start(
  ScheduleDataNode::Implementation::Data data,
  uint32_t period_msec)
{
  _data = std::make_unique<Data>(std::move(data));
  _data->mirror.update();

  std::chrono::milliseconds period(period_msec);
  _timer = node->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period),
    std::bind(&ScheduleDataNode::Implementation::timer_callback_fn, this));
}

//==============================================================================
std::shared_ptr<ScheduleDataNode> ScheduleDataNode::make(
  const std::string& node_name,
  uint16_t port,
  uint32_t period_msec,
  rmf_traffic::Duration wait_time)
{
  const auto start_time = std::chrono::steady_clock::now();
  std::shared_ptr<rclcpp::Node> node(new rclcpp::Node(node_name));
  std::shared_ptr<ScheduleDataNode> schedule_data(new ScheduleDataNode());

  // Creating a mirror manager that queries over all
  // Spacetime in the database schedule
  auto mirror_mgr_future = rmf_traffic_ros2::schedule::make_mirror(
    *node, rmf_traffic::schedule::query_all(), &schedule_data->_pimpl->mutex);

  const auto stop_time = start_time + wait_time;
  while (rclcpp::ok() && std::chrono::steady_clock::now() < stop_time)
  {
    rclcpp::spin_some(node);
    using namespace std::chrono_literals;

    if (mirror_mgr_future.wait_for(0s) == std::future_status::ready)
    {
      // TODO(AA): Starting websocket sending stuff here too
      schedule_data->_pimpl->port = port;
      schedule_data->_pimpl->node = std::move(node);

      schedule_data->_pimpl->start(
        Implementation::Data{mirror_mgr_future.get()}, period_msec);
      return schedule_data;
    }
  }

  RCLCPP_ERROR(
    node->get_logger(),
    "Mirror was not initialized in enough time [%ss]!",
    std::to_string(rmf_traffic::time::to_seconds(wait_time)).c_str());
  return nullptr;
}

//==============================================================================
ScheduleDataNode::ScheduleDataNode()
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation{}))
{}

//==============================================================================
const std::shared_ptr<rclcpp::Node>& ScheduleDataNode::node() const
{
  return _pimpl->node;
}

//==============================================================================
} // namespace websocket_bridge
} // namespace rmf_massrobotics
