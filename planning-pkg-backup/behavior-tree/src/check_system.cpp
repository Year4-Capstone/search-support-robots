class CheckSystemOK : public BT::SyncActionNode {
public:
  CheckSystemOK(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config) {}
  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override {
    std::cout << "[CheckSystemOK] All systems nominal.\n";
    return BT::NodeStatus::SUCCESS;
  }
};
