# FROM ros:foxy AS stage-deps
FROM  registry.gitlab.com/iacbasesoftware/autowareauto/amd64/ade-foxy:iac-dev
ARG GH_TOKEN

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

RUN apt update && apt install -y python3-vcstool
RUN cd /opt && \
    git clone https://${GH_TOKEN}@github.com/AndreaFinazzi/nif.git && \
    cd nif && \
    vcs import < nif.$ROS_DISTRO.repos && \
    rosdep update && \
    pip install -r central-requirements.txt && \
    rosdep install -ir -y --from-path src/ --rosdistro $ROS_DISTRO
    # rosdep install --as-root "apt:false pip:false" --simulate --reinstall --ignore-src -y --from-paths src | sort >> /opt/ros-deps
RUN rm -rf /opt/nif
  
RUN ( \
    echo 'LogLevel DEBUG2'; \
    echo 'PermitRootLogin no'; \
    echo 'PasswordAuthentication yes'; \
    echo 'Subsystem sftp /usr/lib/openssh/sftp-server'; \
  ) > /etc/ssh/sshd_config_test_clion \
  && mkdir /run/sshd

# Shouldn't be needed, libboost-all-dev is installed by rosdep
# ADD --chown=$USER:$GROUP ./boost_1_76_0 /opt/boost_1_76_0
# RUN cd /opt/boost_1_76_0 && \
#     ./bootstrap.sh && \
#     ./b2 install

ENV CLION_VERSION=2021.1.3

ADD --chown="1000:1000" ./clion-$CLION_VERSION /opt/clion-$CLION_VERSION
COPY ./entrypoint /ade_entrypoint

#ADD https://download.jetbrains.com/cpp/CLion-2021.1.3.tar.gz /opt
#RUN sudo tar xvzf /opt/CLion-2021.1.3.tar.gz -C /opt/
#RUN rm -rf /opt/CLion-2021.1.3.tar.gz && \
#    rm -rf /opt/clion-2021.1.3/jre64 && \
#    rm -rf /var/lib/apt/lists/* && \
#    rm -rf /tmp/*