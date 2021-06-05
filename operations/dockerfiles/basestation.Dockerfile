FROM ros:foxy-ros-core

# https://discourse.ros.org/t/ros-gpg-key-expiration-incident/20669
RUN apt update || echo hi
RUN apt install -y curl
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# Install colcon
RUN apt update \
    && apt install -y *colcon*
    # unbelievable that these aren't also installed...
RUN apt install -y gcc g++ build-essential git
RUN apt install -y ros-foxy-rmw-cyclonedds-cpp llvm-dev libclang-dev terminator

# https://github.com/eclipse-zenoh/zenoh-plugin-dds
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
RUN apt install -y llvm-dev libclang-dev
RUN git clone https://github.com/eclipse-zenoh/zenoh-plugin-dds
RUN cd zenoh-plugin-dds && git checkout 2f2b5ef55623f513a5cce04bf2a655f00c32ea8e && /root/.cargo/bin/cargo build --release
RUN cp zenoh-plugin-dds/target/release/dzd /usr/bin

# ADD Executables / config
RUN mkdir /etc/cyclone
ADD operations/config/cyclonedds.xml /etc/cyclone/cyclonedds.xml
ADD operations/scripts/adlink_fm.sh /usr/local/bin
ADD operations/scripts/bst_fm.sh /usr/local/bin
ADD operations/scripts/remote_spoof_spawn.sh /usr/local/bin
ADD operations/scripts/remote_spoof_kill.sh /usr/local/bin

#AUTOWARE MSGS
RUN apt update \
    && apt install -y ros-foxy-joy* iputils* ros-foxy-autoware-auto-msgs tmux tmuxp nano

# Setup Bashrc
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
RUN echo "export CYCLONEDDS_URI=file:///etc/cyclone/cyclonedds.xml" >> ~/.bashrc
RUN echo "export CYCLONE_INCLUDE=/opt/ros/foxy/include" >> ~/.bashrc
RUN echo "export CYCLONE_LIB=/opt/ros/foxy/lib/x86_64-linux-gnu" >> ~/.bashrc
RUN echo "export ROS_DOMAIN_ID=2" >> ~/.bashrc
RUN echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/" >> ~/.bashrc
# RUN echo "source "$HOME/.cargo/env"" >> ~/.bashrc
# RUN echo "alias term_tel='terminator -l 111'" >> ~/.bashrc
# RUN echo "alias term_joy='terminator -l 222'" >> ~/.bashrc
RUN echo "source /workspace/install/setup.bash" >> ~/.bashrc
RUN echo "alias e_shutdown='/workspace/install/shutdown.sh" >> ~/.bashrc
# RUN echo "alias heartbt_tel='ros2 run heartbeat_node heart_exec_TX'" >> ~/.bashrc
# RUN echo "alias ui_pub='ros2 run userinput ui_publisher'" >> ~/.bashrc
# RUN echo "alias rf_pub='ros2 run userinput ui_raceflag'" >> ~/.bashrc
# RUN echo "alias int_node='ros2 run intermediate_node intermediate_node'" >> ~/.bashrc
# RUN echo "alias joy_pub='ros2 launch joystick_vehicle_interface_nodes joystick_vehicle_interface_node.launch.py'" >> ~/.bashrc
# RUN echo "alias vizsrc='source Desktop/ros2_ws/install/setup.bash'" >> ~/.bashrc
# RUN echo "alias asvis='ros2 run visualizer AS_vis'" >> ~/.bashrc
# RUN echo "alias vdvis='ros2 run visualizer VD_vis'" >> ~/.bashrc
# RUN echo "alias ptvis='ros2 run visualizer PT_vis'" >> ~/.bashrc
# RUN echo "alias str_tel='ros2 topic echo /joystick/steering_cmd'" >> ~/.bashrc
# RUN echo "alias acc_tel='ros2 topic echo /joystick/accelerator_cmd'" >> ~/.bashrc
# RUN echo "alias brk_tel='ros2 topic echo /joystick/brake_cmd'" >> ~/.bashrc
# RUN echo "alias gear_tel='ros2 topic echo /vehicle/state_command'" >> ~/.bashrc
# RUN echo "alias emer_tel='ros2 topic echo /vehicle/emergency_stop'" >> ~/.bashrc
# RUN echo "alias rate_tel='ros2 topic hz /counter'" >> ~/.bashrc
# RUN echo "alias joystat_tel='ros2 topic echo /vehicle/joy_control_enable'" >> ~/.bashrc
# RUN echo "alias ss_tel='ros2 topic echo /raptor_dbw_interface/misc_report_do deep_orange_msgs/msg/MiscReport'" >> ~/.bashrc
# RUN echo "alias ct_tel='ros2 topic echo /raptor_dbw_interface/ct_report deep_orange_msgs/msg/CtReport'" >> ~/.bashrc
