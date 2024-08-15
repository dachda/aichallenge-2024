# for local test
colcon build --packages-select path_to_trajectory
source install/setup.bash
ros2 run path_to_trajectory path_to_trajectory_node