FROM ros:humble-ros-base-jammy

# Set up timezone
ENV TZ=Etc/UTC
RUN echo $TZ > /etc/timezone && \
  ln -fs /usr/share/zoneinfo/$TZ /etc/localtime

# Install required utilities
RUN apt update \
&& apt install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    gnupg2 \
    nano \
    python3-dbg \
    python3-setuptools \
    python3-vcstool \
    ruby \
    sudo \
&& rm -rf /var/lib/apt/lists/* \
&& apt clean -qq

# Set up locale
RUN sudo apt update && sudo apt install locales \
&& sudo locale-gen en_US en_US.UTF-8 \
&& sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ARG ROSDIST=humble

# Install required packages
RUN apt update && apt upgrade -y
RUN apt install -y --no-install-recommends \
    subversion g++ cmake xterm libfltk1.3-dev freeglut3-dev libpng-dev libjpeg-dev libxft-dev libxinerama-dev libtiff5-dev \
    protobuf-compiler libb64-dev ros-humble-diagnostic-updater lsb-release wget gnupg 

# Clone and set up AllSeaingVehicle2 and moos-ivp
RUN git clone https://github.com/ArcturusNavigation/AllSeaingVehicle2.git
RUN svn co https://oceanai.mit.edu/svn/moos-ivp-aro/trunk/ moos-ivp
RUN git clone https://github.com/ArcturusNavigation/moos-ivp-arcturus.git
RUN export PATH=/moos-ivp/bin:$PATH && \
    export PATH=/moos-ivp/scripts:$PATH && \
    export PATH=/moos-ivp-arcturus/bin:$PATH \
    export PATH=/moos-ivp-arcturus/scripts:$PATH
RUN cd moos-ivp && ./build-moos.sh && ./build-ivp.sh
RUN cd ../moos-ivp-arcturus && ./build.sh
RUN cd ../AllSeaingVehicle2
RUN git submodule update --init --recursive src/robot_localization
RUN rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
RUN protoc --cpp_out=./src/protobuf_client_ros2/include/protobuf_client ./src/protobuf_client_ros2/include/protobuf_client/gateway.proto
RUN colcon build --merge-install

# Copy over script to Docker container
COPY ./run_my_system.bash /

# Use your ros_entrypoint
COPY ./ros_entrypoint.sh /
