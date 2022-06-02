FROM ghcr.io/andreafinazzi/nif:master

COPY apt-packages /tmp/
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get -y dist-upgrade && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y gettext-base && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
        $(cat /tmp/apt-packages | cut -d# -f1 | envsubst) \
    && rm -rf /var/lib/apt/lists/* /tmp/apt-packages \
    && apt-get clean

# Python deps
COPY requirements /tmp/
RUN pip install -r /tmp/requirements && \
    rm -rf /tmp/requirements

RUN apt-get update && \ 
    DEBIAN_FRONTEND=noninteractive apt-get install -y ros-$ROS_DISTRO-performance-test-fixture ros-$ROS_DISTRO-mimick-vendor 

RUN mkdir -p /opt/autoware_perf
RUN cd /opt/autoware_perf && \
    git clone -b galactic_add_tp https://gitlab.com/reishikou/ros2_tracing.git && \
    git clone -b galactic_add_tp https://gitlab.com/reishikou/tracetools_analysis.git && \
    git clone -b galactic_add_tp https://github.com/reishikou/rclcpp.git && \
    git clone -b galactic_add_tp https://github.com/reishikou/rcl.git
    
RUN cd /opt/autoware_perf && \
    . /opt/ros/$ROS_DISTRO/setup.sh && \
    rosdep update && \
    rosdep install -ir -y --from-path /opt/autoware_perf --rosdistro $ROS_DISTRO --os=ubuntu:focal
    
RUN cd /opt/autoware_perf && \
    . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get -y dist-upgrade && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y linux-headers-$(uname -r) && \
    && rm -rf /var/lib/apt/lists/* /tmp/apt-packages \
    && apt-get clean
