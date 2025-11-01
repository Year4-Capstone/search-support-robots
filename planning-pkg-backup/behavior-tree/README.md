git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git
cd ~/Projects/search-support-robots/ros-packages
colcon build

colcon build --allow-overriding behaviortree_cpp
colcon build --symlink-install --allow-overriding behaviortree_cpp --packages-select behavior-tree 
colcon build --symlink-install --cmake-clean-cache --packages-select behavior-tree--cmake-clean-cache
olcon build --symlink-install --cmake-clean-cache --packages-select behavior-tree --cmake-args -DCMAKE_BUILD_TYPE=Debug
source install/setup.bash
ros2 run behavior-tree run_tree
