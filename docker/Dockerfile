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
  && apt-get clean

RUN ( \
    echo 'LogLevel DEBUG2'; \
    echo 'PermitRootLogin no'; \
    echo 'PasswordAuthentication yes'; \
    echo 'Subsystem sftp /usr/lib/openssh/sftp-server'; \
  ) > /etc/ssh/sshd_config_test_clion \
  && mkdir /run/sshd

COPY ./ade_entrypoint /
