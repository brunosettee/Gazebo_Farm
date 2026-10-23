FROM ros:noetic-perception-focal as base_dev_image

# Install system and ROS packages
RUN apt-get update && apt-get install -y \
    build-essential \
    python3-pip \
    libeigen3-dev \
    libpcap-dev \
    libignition-math4-dev \
    libgoogle-glog-dev \
    git \
    cmake \
    unzip \
    ros-${ROS_DISTRO}-roslint \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-rviz \
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-gazebo-ros-control \
    ros-${ROS_DISTRO}-ddynamic-reconfigure \
    ros-${ROS_DISTRO}-realsense2-camera \
    ros-${ROS_DISTRO}-realsense2-description \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-joint-state-controller \
    ros-${ROS_DISTRO}-effort-controllers   \
    ros-${ROS_DISTRO}-position-controllers \
    ros-${ROS_DISTRO}-velocity-controllers \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-octomap-msgs \
    ros-${ROS_DISTRO}-mavros \
    ros-${ROS_DISTRO}-octomap-ros \
    ros-${ROS_DISTRO}-octomap \
    ros-${ROS_DISTRO}-mavros-msgs \
    ros-${ROS_DISTRO}-rqt-gui-py \
    ros-${ROS_DISTRO}-rqt-gui \
    ros-${ROS_DISTRO}-hector-gazebo-plugins \
    ros-${ROS_DISTRO}-hector-mapping \
    ros-${ROS_DISTRO}-hector-trajectory-server \
    ros-${ROS_DISTRO}-hector-geotiff && \
    rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install --no-cache-dir --upgrade pip && \
    pip3 install --no-cache-dir laspy==2.5.4 open3d rasterio numpy pycollada==0.7.2 Pillow PyGLM==2.7.1 glfw==2.5.3 PyOpenGL==3.1.6 PyOpenGL_accelerate==3.1.6 && \
    pip3 install --no-cache-dir argparse matplotlib tqdm scipy scikit-learn

# Install LAStools
RUN git clone https://github.com/LAStools/LAStools.git /opt/lastools && \
    cd /opt/lastools/LASzip && mkdir build && cd build && \
    cmake .. && make && make install

# Add LAStools to PATH
ENV PATH="/opt/lastools:/opt/lastools/bin:${PATH}"

# Bootstrap catkin workspace
ENV CATKIN_DIR=/catkin_ws
RUN mkdir -p $CATKIN_DIR/src/

# Setup map directories for map_0
RUN mkdir -p $CATKIN_DIR/src/maps_file/las_files && \
    mkdir -p $CATKIN_DIR/src/maps_file/images_tif_files && \
    mkdir -p $CATKIN_DIR/src/maps_file/mesh_files/map_0/images-file && \
    mkdir -p $CATKIN_DIR/src/maps_file/mesh_files/map_0/info-file && \
    mkdir -p $CATKIN_DIR/src/maps_file/mesh_files/map_0/semantic-segmentation && \
    mkdir -p $CATKIN_DIR/src/maps_file/mesh_files/map_0/semantic-segmentation-colored && \
    mkdir -p $CATKIN_DIR/src/maps_file/mesh_files/map_0/camera_info_file && \
    mkdir -p $CATKIN_DIR/src/maps_file/mesh_files/map_0/bounding_box && \
    mkdir -p $CATKIN_DIR/src/maps_file/mesh_files/map_0/depth-segmentation && \
    mkdir -p $CATKIN_DIR/src/maps_file/mesh_files/map_0/Models && \
    mkdir -p $CATKIN_DIR/src/maps_file/mesh_files/map_0/depth-segmentation-npz

RUN /bin/bash -c "echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /root/.bashrc" \
    && /bin/bash -c "echo 'source /catkin_ws/devel/setup.bash' >> /root/.bashrc"   \
    && /bin/bash -c "echo 'export GAZEBO_RESOURCE_PATH=\$GAZEBO_RESOURCE_PATH:/catkin_ws/src/cessna_simulation/worlds' >> /root/.bashrc" \
    && /bin/bash -c "echo 'export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/catkin_ws/src/cessna_simulation/models' >> /root/.bashrc"

WORKDIR $CATKIN_DIR
COPY ./src/ $CATKIN_DIR/src/

# Fix hector packages that are missing package.xml files by installing missing dependencies
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-geographic-msgs \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-eigen-conversions && \
    rm -rf /var/lib/apt/lists/*

RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y --skip-keys hector_pose_estimation && \
    catkin_make"
