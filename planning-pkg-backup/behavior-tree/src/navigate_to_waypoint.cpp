class NavigateToNextWaypoint : public BT::SyncActionNode {
public:
  NavigateToNextWaypoint(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config) {}
  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override {
    std::cout << "[NavigateToNextWaypoint] Moving to next waypoint...\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    return BT::NodeStatus::SUCCESS;
  }
};
