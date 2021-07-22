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

#include <chrono>
#include <memory>
#include <iomanip>
#include <unordered_set>
#include <rclcpp/rclcpp.hpp>

#include <websocketpp/client.hpp>
#include <websocketpp/common/thread.hpp>
#include <websocketpp/common/memory.hpp>
#include <websocketpp/config/asio_no_tls_client.hpp>

#include <rmf_traffic/Motion.hpp>
#include <rmf_traffic_ros2/Time.hpp>
#include <rmf_visualization_schedule/CommonData.hpp>
#include <rmf_visualization_schedule/ScheduleDataNode.hpp>

#include <std_msgs/msg/string.hpp>

#include "../json.hpp"

//==============================================================================
bool get_arg(
  const std::vector<std::string>& args,
  const std::string& key,
  std::string& value,
  const std::string& desc,
  const bool mandatory = true)
{
  const auto key_arg = std::find(args.begin(), args.end(), key);
  if (key_arg == args.end())
  {
    if (mandatory)
    {
      std::cerr << "You must specify a " << desc <<" using the " << key
                << " argument!" << std::endl;
    }
    return false;
  }
  else if (key_arg+1 == args.end())
  {
    std::cerr << "The " << key << " argument must be followed by a " << desc
              << "!" << std::endl;
    return false;
  }

  value = *(key_arg+1);
  return true;
}  

//==============================================================================
class WebsocketClient
{
public:

  using Client = websocketpp::client<websocketpp::config::asio_client>;
  using Json = nlohmann::json;

private:
  std::shared_ptr<rmf_visualization_schedule::ScheduleDataNode> _schedule_node;
  rclcpp::TimerBase::SharedPtr _timer;
  
  uint16_t _port = 0;
  std::string _map_name = "L1";
  std::unordered_set<std::string> _robot_names;

  const Json _j_envelope = {{"x", {}}, {"y", {}}};
  const Json _j_identity =
  {
    {"manufacturerName", {}},
    {"robotModel", {}},
    {"robotSerialNumb", {}},
    {"baseRobotEnvelope", {}},
    {"uuid", {}},
    {"timestamp", {}}
  };
  const Json _j_angle =
  {
    {"x", {}},
    {"y", {}},
    {"z", {}},
    {"w", {}}
  };
  const Json _j_velocity =
  {
    {"linear", {}},
    {"angular", {}}
  };
  const Json _j_location =
  {
    {"x", {}},
    {"y", {}},
    {"z", {}},
    {"angle", {}},
    {"planarDatum", {}}
  };
  const Json _j_waypoint =
  {
    {"timestamp", {}},
    {"x", {}},
    {"y", {}},
    {"angle", {}},
    {"planarDatumUUID", {}}
  };
  const Json _j_state =
  {
    {"uuid", {}},
    {"timestamp", {}},
    {"operationalState", {}},
    {"batteryPercentage", {}},
    {"velocity", {}},
    {"location", {}},
    {"path", {}},
    {"destinations", {}}
  };

  bool connected = false;
  Client m_endpoint;
  websocketpp::lib::shared_ptr<websocketpp::lib::thread> m_thread;
  websocketpp::connection_hdl m_hdl;

  Json _quat(double yaw)
  {
    using namespace Eigen;
    Quaterniond q;
    q = AngleAxisd(0.0 , Vector3d::UnitX())
        * AngleAxisd(0.0 , Vector3d::UnitY())
        * AngleAxisd(yaw, Vector3d::UnitZ());
    
    Json j_angle = _j_angle;
    j_angle["x"] = q.x();
    j_angle["y"] = q.y();
    j_angle["z"] = q.z();
    j_angle["w"] = q.w();
    return j_angle;
  }

  std::string _time_stamp(rmf_traffic::Time time)
  {
    std::string sec =
      std::to_string(
        static_cast<int32_t>(
          std::chrono::duration_cast<std::chrono::seconds>(
            time.time_since_epoch()).count()));
    std::string nanosec = 
      std::to_string(
        static_cast<uint32_t>(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
            time.time_since_epoch()).count() % 1000000000));
    std::string stamp = sec + " " + nanosec;
    return stamp;
  }

  void _on_new_robot(const std::string& robot_name)
  {
    if (!connected)
      return;

    RCLCPP_INFO(
      _schedule_node->get_logger(),
      "Found new robot: %s",
      robot_name.c_str());

    // TODO(AA): Parameterize everything.
    Json j_envelope = _j_envelope;
    j_envelope["x"] = 1.0;
    j_envelope["y"] = 1.0;

    Json j_identity = _j_identity;
    j_identity["manufacturerName"] = "Open Robotics";
    j_identity["robotModel"] = "TinyRobot";
    j_identity["robotSerialNumb"] = robot_name;
    j_identity["baseRobotEnvelope"] = j_envelope;
    // j_identity["baseRobotEnvelope"].push_back(j_envelope);
    j_identity["uuid"] = robot_name;
    j_identity["timestamp"] =
      _time_stamp(
        rmf_traffic_ros2::convert(_schedule_node->get_clock()->now()));
  
    websocketpp::lib::error_code ec;
    m_endpoint.send(m_hdl, j_identity.dump(), websocketpp::frame::opcode::text, ec);
  } 

  void _timer_callback_fn()
  {
    if (!connected)
      return;

    using namespace rmf_visualization_schedule;

    RequestParam req;
    req.map_name = _map_name;
    req.start_time =
      rmf_traffic_ros2::convert(_schedule_node->get_clock()->now());
    req.finish_time = req.start_time + std::chrono::milliseconds(600000);

    auto elems = _schedule_node->get_elements(req);
    RCLCPP_INFO(
      _schedule_node->get_logger(),
      "Found %d elements.",
      static_cast<int>(elems.size()));

    // Handle json stuff
    try
    {
      for (const auto& elem : elems)
      {
        std::string robot_name = elem.description.name();
        auto insertion = _robot_names.insert(robot_name);
        if (insertion.second)
        {
          _on_new_robot(robot_name);
        }

        // create json state
        const auto& trajectory = elem.route.trajectory();
        const auto start_time = std::max(
          *trajectory.start_time(), req.start_time);
        const auto end_time = std::min(
          *trajectory.finish_time(), req.finish_time);

        // TODO(AA): not hard code state
        Json j_state = _j_state;
        j_state["uuid"] = elem.description.name();
        j_state["timestamp"] = _time_stamp(start_time); 
        j_state["operationalState"] = "navigating";
        j_state["batteryPercentage"] = 1.0;
        // j_state["velocity"] = 
        // j_state["location"] =
        // j_state["path"] =

        auto add_waypoint = [&](rmf_traffic::Time finish_time,
            Eigen::Vector3d finish_position)
        {
          auto j_wp = _j_waypoint;
          j_wp["timestamp"] = _time_stamp(finish_time);
          j_wp["x"] = finish_position[0];
          j_wp["y"] = finish_position[1];
          j_wp["angle"] = _quat(finish_position[2]);
          j_wp["planarDatumUUID"] = "4B8302DA-21AD-401F-AF45-1DFD956B80B5";
          j_state["path"].push_back(j_wp);
        };

        auto it = trajectory.find(start_time);
        assert(it != trajectory.end());
        assert(trajectory.find(end_time) != trajectory.end());

        // Add the trimmed start
        auto begin = it; --begin;
        auto end = it; ++end;
        auto motion = rmf_traffic::Motion::compute_cubic_splines(begin, end);

        const Eigen::Vector3d start_vel = motion->compute_velocity(start_time);
        const Eigen::Vector3d start_pos = motion->compute_position(start_time);
      
        Json j_loc = _j_location;
        j_loc["x"] = start_pos[0];
        j_loc["y"] = start_pos[1];
        j_loc["z"] = 0.0;
        j_loc["angle"] = _quat(start_pos[2]);
        j_loc["planarDatum"] = "4B8302DA-21AD-401F-AF45-1DFD956B80B5";
        j_state["location"] = j_loc;

        add_waypoint(it->time(), it->position());

        // Add the waypoints in between
        for (; it < trajectory.find(end_time); it++)
        {
          add_waypoint(it->time(), it->position());
        }
       
        // Add the trimmed end
        assert(it != trajectory.end());
        begin = it; --begin;
        end = it; ++end;
        motion = rmf_traffic::Motion::compute_cubic_splines(begin, end);
        add_waypoint(end_time, motion->compute_position(end_time)); 

        websocketpp::lib::error_code ec;
        m_endpoint.send(m_hdl, j_state.dump(), websocketpp::frame::opcode::text, ec);
        RCLCPP_INFO(
          _schedule_node->get_logger(),
          "Sending state once.");
      }
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(
        _schedule_node->get_logger(),
        "Error: %s",
        std::to_string(*e.what()).c_str());
      return;
    }
  }

  void on_open(Client* c, websocketpp::connection_hdl hdl)
  {
    RCLCPP_INFO(_schedule_node->get_logger(), "On open...");
    connected = true;
  }

  void on_fail(Client* c, websocketpp::connection_hdl hdl)
  {
    RCLCPP_INFO(_schedule_node->get_logger(), "On fail...");
  }

  void on_close(Client* c, websocketpp::connection_hdl hdl)
  {
    RCLCPP_INFO(_schedule_node->get_logger(), "On close...");
    connected = false;
  }

  void on_message(websocketpp::connection_hdl, Client::message_ptr msg)
  {
    RCLCPP_INFO(_schedule_node->get_logger(), "On message...");
  }

public:

  WebsocketClient(
    std::shared_ptr<rmf_visualization_schedule::ScheduleDataNode> schedule_node,
    const std::string& map_name,
    uint16_t port,
    uint32_t period_msec)
  : _schedule_node(std::move(schedule_node)),
    _map_name(map_name),
    _port(port)
  {
    // Initialize all the websocket stuff 
    // std::string uri = "ws://localhost:" + std::to_string(_port);
    std::string uri = "ws://localhost:3000";
    RCLCPP_INFO(
      _schedule_node->get_logger(),
      "Connecting to uri: %s",
      uri.c_str());

    m_endpoint.clear_access_channels(websocketpp::log::alevel::all);
    m_endpoint.clear_error_channels(websocketpp::log::elevel::all);
    m_endpoint.init_asio();
    m_endpoint.start_perpetual();
    m_thread = websocketpp::lib::make_shared<websocketpp::lib::thread>(&Client::run, &m_endpoint);

    websocketpp::lib::error_code ec;
    Client::connection_ptr con = m_endpoint.get_connection(uri, ec);
    if (ec)
    {
      std::cout << "> Connect initialization error: " << ec.message() << std::endl;
      return;
    }
    
    m_hdl = con->get_handle();
    con->set_open_handler(
      websocketpp::lib::bind(
        &WebsocketClient::on_open, this, &m_endpoint, websocketpp::lib::placeholders::_1));
    con->set_fail_handler(
      websocketpp::lib::bind(
        &WebsocketClient::on_fail, this, &m_endpoint, websocketpp::lib::placeholders::_1));
    con->set_close_handler(
      websocketpp::lib::bind(
        &WebsocketClient::on_close, this, &m_endpoint, websocketpp::lib::placeholders::_1));
    con->set_message_handler(
      websocketpp::lib::bind(
        &WebsocketClient::on_message, this,
        websocketpp::lib::placeholders::_1,
        websocketpp::lib::placeholders::_2));
    m_endpoint.connect(con);

    std::chrono::milliseconds period(period_msec);
    _timer =
      _schedule_node->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&WebsocketClient::_timer_callback_fn, this));
  }

  ~WebsocketClient()
  {
    m_endpoint.stop_perpetual();
    if (connected)
    {
      std::cout << "Closing connection in destructor..." << std::endl;
      websocketpp::lib::error_code ec;
      m_endpoint.close(m_hdl, websocketpp::close::status::going_away, "", ec);
      if (ec)
      {
        std::cout << "Error closing connection: " << ec.message() << std::endl;
      }
    }
    m_thread->join();
  }
};

//==============================================================================
int main(int argc, char** argv)
{
  const std::vector<std::string> args =
    rclcpp::init_and_remove_ros_arguments(argc, argv);

  std::string node_name = "websocket_bridge_schedule_data_node";
  get_arg(args, "-n", node_name, "node name", false);

  std::string map_name = "L1";
  get_arg(args, "-m", map_name, "map_name", false);

  std::string port_string;
  get_arg(args, "-p", port_string, "port", false);
  const uint16_t port = port_string.empty() ? 8006 : std::stoul(
    port_string, nullptr, 0);

  std::string period_msec_string;
  get_arg(args, "-r", period_msec_string, "period_msec", false);
  uint32_t period_msec = period_msec_string.empty() ?
    1000 : std::stod(period_msec_string);

  const auto schedule_data_node =
    rmf_visualization_schedule::ScheduleDataNode::make(node_name);
  if (!schedule_data_node)
  {
    std::cerr << "Failed to initialize the schedule_data_node" << std::endl;
    return 1;
  }
  
  std::shared_ptr<WebsocketClient> websocket_client(new WebsocketClient(
    schedule_data_node,
    map_name,
    port,
    period_msec));

  RCLCPP_INFO(
    schedule_data_node->get_logger(),
    "Node [%s] started...",
    node_name.c_str());

  rclcpp::spin(schedule_data_node);

  RCLCPP_INFO(
    schedule_data_node->get_logger(),
    "Node [%s] closing down",
    node_name.c_str());

  rclcpp::shutdown();
}
