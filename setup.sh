clear
echo "Installing dependencies"
sleep 10
apt install -y ros-foxy-rmw-cyclonedds-cpp llvm-dev libclang-dev terminator
mkdir /etc/cyclone
cp cyclonedds.xml /etc/cyclone/cyclonedds.xml

clear
echo "Building AutowareAuto"
sleep 10
cd ~/AutowareAuto
colcon build

clear
echo "Building zenoh-plugin-dds"
sleep 10
cd ~/zenoh-plugin-dds
cargo build --release

clear
echo "Building DO-12 workspace"
sleep 10
cd ~/d0_12_ws
colcon build

clear
echo "Adding aliases to your .bashrc"
sleep 10
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
echo "export CYCLONEDDS_URI=file:///etc/cyclone/cyclonedds.xml" >> ~/.bashrc
echo "export CYCLONE_INCLUDE=/opt/ros/foxy/include" >> ~/.bashrc
echo "export CYCLONE_LIB=/opt/ros/foxy/lib/x86_64-linux-gnu" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=2" >> ~/.bashrc
echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/" >> ~/.bashrc
echo "source "$HOME/.cargo/env"" >> ~/.bashrc
echo "alias term_tel='terminator -l 111'" >> ~/.bashrc
echo "alias term_joy='terminator -l 222'" >> ~/.bashrc
echo "alias heartbt_tel='cd do_12_ws; source install/setup.bash; ros2 run heartbeat_node echo heart_exec_TX'" >> ~/.bashrc
echo "alias ui_pub='cd do_12_ws; source install/setup.bash; ros2 run userinput ui_publisher'" >> ~/.bashrc
echo "alias rf_pub='cd do_12_ws; source install/setup.bash; ros2 run userinput ui_raceflag'" >> ~/.bashrc
echo "alias joy_pub='cd AutowareAuto; source install/setup.bash; ros2 launch joystick_vehicle_interface_nodes joystick_vehicle_interface_node.launch.py'" >> ~/.bashrc
echo "alias int_node='cd ros2_testing; source install/setup.bash; ros2 run heartbeat_node intermediate_node'" >> ~/.bashrc
echo "alias vizsrc='source Desktop/ros2_ws/install/setup.bash'" >> ~/.bashrc
echo "alias asvis='ros2 run visualizer AS_vis'" >> ~/.bashrc
echo "alias vdvis='ros2 run visualizer VD_vis'" >> ~/.bashrc
echo "alias ptvis='ros2 run visualizer PT_vis'" >> ~/.bashrc
echo "alias str_tel='ros2 topic echo /joystick/steering_cmd'" >> ~/.bashrc
echo "alias acc_tel='ros2 topic echo /joystick/accelerator_cmd'" >> ~/.bashrc
echo "alias brk_tel='ros2 topic echo /joystick/brake_cmd'" >> ~/.bashrc
echo "alias gear_tel='source AutowareAuto/install/setup.bash;ros2 topic echo /vehicle/state_command'" >> ~/.bashrc
echo "alias emer_tel='ros2 topic echo /vehicle/emergency_stop'" >> ~/.bashrc
echo "alias rate_tel='ros2 topic hz /counter'" >> ~/.bashrc
echo "alias joystat_tel='ros2 topic echo /vehicle/joy_control_enable'" >> ~/.bashrc
echo "alias ct_tel='ros2 topic echo /raptor_dbw_interface/ct_report'" >> ~/.bashrc

source ~/.bashrc

clear
echo "All done!"