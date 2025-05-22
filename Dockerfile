FROM osrf/ros:humble-desktop

ARG DEBIAN_FRONTEND=noninteractive

# Install debian packages
RUN apt-get update && apt-get install -y \
        nano \
        sudo \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-imu-tools \
    && rm -rf /var/lib/apt/lists/*

# Create a non-root user
RUN useradd -ms /bin/bash devuser \
   && echo "devuser ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
   && chown -R devuser:devuser /home/devuser/

# Create ros workspace
RUN mkdir -p /home/devuser/ros2_ws/src/ /home/devuser/livox_ws/src/ /home/devuser/.config \
    && chown -R devuser:devuser /home/devuser/ros2_ws \
    && chown -R devuser:devuser /home/devuser/livox_ws \
    && chown -R devuser:devuser /home/devuser/.config

# Switch to non-root user
USER devuser
ENV USER devuser
WORKDIR /home/devuser/livox_ws/src

RUN echo "source /opt/ros/humble/setup.bash" >> /home/devuser/.bashrc \
    && echo "source /home/devuser/livox_ws/install/setup.bash" >> /home/devuser/.bashrc

# Install Livox SDK and Livox ROS2 wrapper
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git \
    && cd Livox-SDK2 && mkdir build && cd build \
    && cmake .. && make -j && sudo make install \
    && cd ../../ && rm -r Livox-SDK2

# Install Livox ROS2 wrapper
RUN git clone https://github.com/Livox-SDK/livox_ros_driver2.git \
    && cd livox_ros_driver2 \
    && bash -c "source /opt/ros/humble/setup.bash && ./build.sh humble"
RUN echo "export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH}:/usr/local/lib" >> /home/devuser/.bashrc

# Copy convienience scripts
COPY launch/run.sh launch/record_MID360_launch.py launch/record_HAP_launch.py ./

