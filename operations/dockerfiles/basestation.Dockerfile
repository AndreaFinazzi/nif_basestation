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
RUN git clone --branch IAC https://github.com/eclipse-zenoh/zenoh-plugin-dds
RUN cd zenoh-plugin-dds && /root/.cargo/bin/cargo build --release
RUN cp zenoh-plugin-dds/target/release/zenoh-bridge-dds /usr/bin

# ADD Executables / config
RUN mkdir /etc/cyclone
ADD operations/config/cyclonedds.xml /etc/cyclone/cyclonedds.xml

#AUTOWARE MSGS
RUN apt update \
    && apt install -y ros-foxy-joy* iputils* ros-foxy-autoware-auto-msgs tmux tmuxp nano

# message dependencies 
RUN git clone --branch merge_joystick https://gitlab.com/IACBaseSoftware/deep_orange_msgs.git /workspace/src/deep_orange_msgs
RUN git clone https://gitlab.com/IACBaseSoftware/raptor-dbw-ros2.git /workspace/src/raptor-dbw-ros2

# Setup Bashrc
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
RUN echo "export CYCLONEDDS_URI=file:///etc/cyclone/cyclonedds.xml" >> ~/.bashrc
RUN echo "export CYCLONE_INCLUDE=/opt/ros/foxy/include" >> ~/.bashrc
RUN echo "export CYCLONE_LIB=/opt/ros/foxy/lib/x86_64-linux-gnu" >> ~/.bashrc
RUN echo "export ROS_DOMAIN_ID=2" >> ~/.bashrc
RUN echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/" >> ~/.bashrc
RUN echo "source /workspace/install/setup.bash" >> ~/.bashrc
RUN echo "alias e_shutdown='/workspace/install/shutdown.sh'" >> ~/.bashrc
