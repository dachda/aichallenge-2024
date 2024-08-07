# rosdep install -i --from-path . --rosdistro foxy -y
colcon build --packages-select object_subscriber
source install/setup.bash
ros2 run object_subscriber object_subscriber_node