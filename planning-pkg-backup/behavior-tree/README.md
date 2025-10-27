git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git
cd ~/Projects/search-support-robots/ros-packages
colcon build

colcon build --allow-overriding behaviortree_cpp
source install/setup.bash
ros2 run behavior-tree run_tree
