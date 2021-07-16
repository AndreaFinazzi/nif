FROM  registry.gitlab.com/iacbasesoftware/autowareauto/amd64/ade-foxy:iac-dev

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
      ros-foxy-raptor-dbw-msgs \
      ros-foxy-autoware-auto-msgs \
      ros-foxy-astuff-sensor-msgs \
  && apt-get clean

RUN ( \
    echo 'LogLevel DEBUG2'; \
    echo 'PermitRootLogin no'; \
    echo 'PasswordAuthentication yes'; \
    echo 'Subsystem sftp /usr/lib/openssh/sftp-server'; \
  ) > /etc/ssh/sshd_config_test_clion \
  && mkdir /run/sshd

COPY ./ade_entrypoint /

ADD https://download.jetbrains.com/cpp/CLion-2021.1.3.tar.gz /opt
RUN sudo tar xvzf /opt/CLion-2021.1.3.tar.gz -C /opt/
RUN rm -rf /opt/CLion-2021.1.3.tar.gz && \
    rm -rf /opt/clion-2021.1.3/jre64 && \    
    rm -rf /var/lib/apt/lists/* && \
    rm -rf /tmp/*

