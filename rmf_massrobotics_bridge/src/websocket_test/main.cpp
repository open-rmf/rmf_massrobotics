#include <chrono>
#include <string>
#include <memory>
#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include <websocketpp/client.hpp>
#include <websocketpp/common/thread.hpp>
#include <websocketpp/common/memory.hpp>
#include <websocketpp/config/asio_no_tls_client.hpp>

#include "../json.hpp"

class WebNode : public rclcpp::Node
{
public:
  using Client = websocketpp::client<websocketpp::config::asio_client>;
  using Json = nlohmann::json;

private:

  bool connected = false;
  rclcpp::TimerBase::SharedPtr _timer;
  Client m_endpoint;
  websocketpp::lib::shared_ptr<websocketpp::lib::thread> m_thread;
  websocketpp::connection_hdl m_hdl;

  void on_open(Client* c, websocketpp::connection_hdl hdl)
  {
    RCLCPP_INFO(get_logger(), "On open...");
    connected = true;
  }

  void on_fail(Client* c, websocketpp::connection_hdl hdl)
  {
    RCLCPP_INFO(get_logger(), "On fail...");
  }

  void on_close(Client* c, websocketpp::connection_hdl hdl)
  {
    RCLCPP_INFO(get_logger(), "On close...");
    connected = false;
  }

  void on_message(websocketpp::connection_hdl, Client::message_ptr msg)
  {
    RCLCPP_INFO(get_logger(), "On message...");
  }

  void _timer_callback_fn()
  {
    RCLCPP_INFO(get_logger(), "timer callback...");

    const Json _j_identity =
    {
      {"manufacturerName", {}},
      {"robotModel", {}},
      {"robotSerialNumb", {}},
      {"baseRobotEnvelope", {}},
      {"uuid", {}},
      {"timestamp", {}}
    };
    Json id = _j_identity;
    id["manufacturerName"] = "China";

    websocketpp::lib::error_code ec;
    m_endpoint.send(m_hdl, id.dump(), websocketpp::frame::opcode::text, ec);
  }

public:

  WebNode()
  : Node("web_node")
  {
    m_endpoint.clear_access_channels(websocketpp::log::alevel::all);
    m_endpoint.clear_error_channels(websocketpp::log::alevel::all);
    m_endpoint.init_asio();
    m_endpoint.start_perpetual();
    
    m_thread = websocketpp::lib::make_shared<websocketpp::lib::thread>(
      &Client::run, &m_endpoint);

    // connect
    const std::string uri = "ws://localhost:3000";
    websocketpp::lib::error_code ec;
    Client::connection_ptr con = m_endpoint.get_connection(uri, ec);
    m_hdl = con->get_handle();
    con->set_open_handler(
      websocketpp::lib::bind(
        &WebNode::on_open, this, &m_endpoint,
        websocketpp::lib::placeholders::_1));
    con->set_fail_handler(
      websocketpp::lib::bind(
        &WebNode::on_fail, this, &m_endpoint,
        websocketpp::lib::placeholders::_1));
    con->set_close_handler(
      websocketpp::lib::bind(
        &WebNode::on_close, this, &m_endpoint,
        websocketpp::lib::placeholders::_1));
    con->set_message_handler(
      websocketpp::lib::bind(
        &WebNode::on_message, this,
        websocketpp::lib::placeholders::_1,
        websocketpp::lib::placeholders::_2));
    m_endpoint.connect(con);

    std::chrono::milliseconds period(1000);
    _timer =
      create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&WebNode::_timer_callback_fn, this));
  }

  ~WebNode()
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

int main(int argc, char** argv)
{
  const std::vector<std::string> args =
    rclcpp::init_and_remove_ros_arguments(argc, argv);

  std::shared_ptr<WebNode> node(new WebNode());
  
  RCLCPP_INFO(node->get_logger(), "starting...");
  rclcpp::spin(node);
  RCLCPP_INFO(node->get_logger(), "closing...");  
  rclcpp::shutdown();
}
