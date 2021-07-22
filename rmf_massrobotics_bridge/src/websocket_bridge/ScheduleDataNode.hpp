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

#ifndef SRC__WEBSOCKET_BRIDGE__SCHEDULEDATANODE_HPP
#define SRC__WEBSOCKET_BRIDGE__SCHEDULEDATANODE_HPP

#include <rclcpp/node.hpp>

#include <rmf_traffic/Time.hpp>
#include <rmf_utils/impl_ptr.hpp>

#include <memory>
#include <string>

namespace rmf_massrobotics {
namespace websocket_bridge {

class ScheduleDataNode : public std::enable_shared_from_this<ScheduleDataNode>
{
public:
  /// Builder function which returns a pointer to ScheduleDataNode when
  /// the Mirror Manager is readied.
  /// A nullptr is returned if initialization fails.
  ///
  /// \param[in] node_name
  ///   The name of the node
  ///
  /// \param[in] port
  ///   The port number of the websocket server
  ///
  /// \param[in] period_msec 
  ///   The period duration between updates to websocket server in milliseconds 
  ///
  /// \param[in] wait_time
  ///   The waiting duration to discover the rmf_traffic_schedule node
  static std::shared_ptr<ScheduleDataNode> make(
    const std::string& node_name,
    uint16_t port,
    uint32_t period_msec,
    rmf_traffic::Duration wait_time = std::chrono::seconds(10));

  ///
  const std::shared_ptr<rclcpp::Node>& node() const;

  class Implementation;
private:
  ScheduleDataNode();
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace websocket_bridge
} // namespace rmf_massrobotics 

#endif // SRC__WEBSOCKET_BRIDGE__SCHEDULEDATANODE_HPP
