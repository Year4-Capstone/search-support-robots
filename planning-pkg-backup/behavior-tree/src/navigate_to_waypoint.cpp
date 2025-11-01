#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <iostream>

class NavigateToWaypoint : public BT::SyncActionNode
{
public:
  // NOTE the extra constructor arg: a shared ROS node
  NavigateToWaypoint(const std::string& name,
                     const BT::NodeConfig& config,
                     const rclcpp::Node::SharedPtr& shared_node)
    : BT::SyncActionNode(name, config),
      node_(shared_node)
  {
    std::cout << "[NavigateToWaypoint] Constructed with shared node" << std::endl;
    if (!node_) {
      throw std::runtime_error("NavigateToWaypoint: shared node is null");
    }
    pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    std::cout << "[NavigateToWaypoint] Publisher created" << std::endl;
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    std::cout << "[NavigateToWaypoint] Tick start" << std::endl;
    if (!pub_) {
      std::cerr << "[NavigateToWaypoint] ERROR: pub_ null" << std::endl;
      return BT::NodeStatus::FAILURE;
    }

    geometry_msgs::msg::Twist msg;
    msg.linear.x = 2.0;
    msg.angular.z = 1.0;
    std::cout << "[NavigateToWaypoint] Publishing Twist: lin.x=" << msg.linear.x
              << " ang.z=" << msg.angular.z << std::endl;

    pub_->publish(msg);
    rclcpp::spin_some(node_);

    std::cout << "[NavigateToWaypoint] Tick end" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
};

// IMPORTANT: remove BT_REGISTER_NODES here to avoid plugin/double registration.
// We register explicitly in run_tree.cpp via a builder.
