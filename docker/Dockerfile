# ARG FROM_IMAGE=ros:galactic
ARG ROS_DISTRO=galactic

# FROM ros:${ROS_DISTRO} AS stage-deps
FROM ros:$ROS_DISTRO

ARG GH_TOKEN=${GH_TOKEN}
ARG GL_TOKEN=${GL_TOKEN}

## Install from packages list
COPY apt-packages /tmp/
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get -y dist-upgrade && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y gettext-base && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
        $(cat /tmp/apt-packages | cut -d# -f1 | envsubst) \
    && rm -rf /var/lib/apt/lists/* /tmp/apt-packages \
    && apt-get clean

RUN ( \
    echo 'LogLevel DEBUG2'; \
    echo 'PermitRootLogin no'; \
    echo 'PasswordAuthentication yes'; \
    echo 'Subsystem sftp /usr/lib/openssh/sftp-server'; \
  ) > /etc/ssh/sshd_config_test_clion \
  && mkdir /run/sshd

# Clone and install dependencies
RUN cd /opt && echo $GH_TOKEN && echo $GL_TOKEN && \
    git clone --recurse-submodules https://$GH_TOKEN@github.com/AndreaFinazzi/nif.git && \
    cd nif && echo $GL_TOKEN > .secret.gitlab && echo $GH_TOKEN > .secret.github && \
    ROS_DISTRO=$ROS_DISTRO ./nifupdate

# GTSAM lib install
RUN cd /opt && git clone https://github.com/borglab/gtsam.git && \
    cd /opt/gtsam && \
    mkdir build && \
    cd build && \
    cmake -DGTSAM_INSTALL_GEOGRAPHICLIB=ON -DGTSAM_WITH_EIGEN_MKL=OFF .. && \
    make install

# Install Allied Vision Vimba SDK
RUN wget https://autoware-auto.s3.us-east-2.amazonaws.com/iac/Vimba_v4.2_Linux.tgz \
    && tar xvf Vimba_v4.2_Linux.tgz \
    && rm Vimba_v4.2_Linux.tgz \
    && mv Vimba_4_2/ /opt \
    && /opt/Vimba_4_2/VimbaGigETL/Install.sh

# Install VSCode
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y software-properties-common apt-transport-https wget && \
    wget -q https://packages.microsoft.com/keys/microsoft.asc -O- | sudo apt-key add - && \
    add-apt-repository "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main" && \
    apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y code


COPY env.sh /etc/profile.d/ade_env.sh

# SUDOERS settings
RUN echo 'ALL ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
RUN echo 'Defaults env_keep += "DEBUG ROS_DISTRO"' >> /etc/sudoers

RUN rm -rf /opt/nif
COPY ./entrypoint /ade_entrypoint

ENTRYPOINT ["/ade_entrypoint"]

CMD ["/bin/sh", "-c", "trap 'exit 147' TERM; tail -f /dev/null & wait ${!}"]

