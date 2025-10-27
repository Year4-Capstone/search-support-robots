#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <chrono>

// Include node classes
#include "check_system.cpp"
#include "navigate_to_waypoint.cpp"
#include "detect_person.cpp"
#include "approach_person.cpp"
#include "deliver_payload.cpp"
#include "replan_search_area.cpp"
#include "report_to_base.cpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  BT::BehaviorTreeFactory factory;

  factory.registerNodeType<CheckSystemOK>("CheckSystemOK");
  factory.registerNodeType<NavigateToNextWaypoint>("NavigateToNextWaypoint");
  factory.registerNodeType<DetectPerson>("DetectPerson");
  factory.registerNodeType<ApproachPerson>("ApproachPerson");
  factory.registerNodeType<DeliverPayload>("DeliverPayload");
  factory.registerNodeType<ReplanSearchArea>("ReplanSearchArea");
  factory.registerNodeType<ReportToBase>("ReportToBase");

  std::string pkgpath = ament_index_cpp::get_package_share_directory("behavior-tree");
  std::string xml_file = pkgpath + "/behavior_trees/search_and_support.xml";

  auto tree = factory.createTreeFromFile(xml_file);
  BT::Groot2Publisher publisher(tree, 5555);

  while (rclcpp::ok())
  {
    tree.tickOnce();  // single cycle each loop
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  }

  rclcpp::shutdown();
  return 0;
}
