# FROM ros:foxy AS stage-deps
FROM  registry.gitlab.com/iacbasesoftware/autowareauto/amd64/ade-foxy:iac-dev
ARG GH_TOKEN=${GH_TOKEN}
ARG GL_TOKEN=${GL_TOKEN}

RUN apt-get update \
  && apt-get upgrade -y \
  && apt-get install -y ssh \
      build-essential \
      gcc \
      g++ \
      gdb \
      clang \
      cmake \
      rsync \
      tar \
      python \
      wget \
  && apt-get clean

# COPY --from=stage-deps /opt/ros-deps /tmp/ros-deps

# RUN chmod +x /tmp/ros-deps && \    
#     /bin/bash /tmp/ros-deps && \
#     rm -rf /var/lib/apt/lists/* /tmp/ros-deps

# Clone and install dependencies
RUN apt update && apt install -y python3-vcstool
RUN cd /opt && \
    git clone --recurse-submodules https://${GH_TOKEN}@github.com/AndreaFinazzi/nif.git && \
    cd nif && echo $GL_TOKEN > .secret.gitlab && echo $GH_TOKEN > .secret.github && \
    vcs import < nif.$ROS_DISTRO.repos && \
    rosdep update && \
    pip install -r central-requirements.txt && \
    rosdep install -ir -y --from-path src/ --rosdistro $ROS_DISTRO
    # rosdep install --as-root "apt:false pip:false" --simulate --reinstall --ignore-src -y --from-paths src | sort >> /opt/ros-deps

# GTSAM lib install
RUN cd /opt && git clone https://github.com/borglab/gtsam.git && \
    cd /opt/gtsam && \
    mkdir build && \
    cd build && \
    cmake -DGTSAM_INSTALL_GEOGRAPHICLIB=ON -DGTSAM_WITH_EIGEN_MKL=OFF .. && \
    make install

RUN rm -rf /opt/nif
  
RUN ( \
    echo 'LogLevel DEBUG2'; \
    echo 'PermitRootLogin no'; \
    echo 'PasswordAuthentication yes'; \
    echo 'Subsystem sftp /usr/lib/openssh/sftp-server'; \
  ) > /etc/ssh/sshd_config_test_clion \
  && mkdir /run/sshd


RUN apt-get update \
  && apt-get install -y \
      ros-foxy-rqt \
      ros-foxy-rqt-* \
      ros-foxy-tf2-tools \
      tmux tmuxp terminator nano \
      ros-${ROS_DISTRO}-plotjuggler \
      ros-${ROS_DISTRO}-plotjuggler-ros \
  && apt-get clean

COPY ./entrypoint /ade_entrypoint
