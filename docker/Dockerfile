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
  && apt-get clean

RUN ( \
    echo 'LogLevel DEBUG2'; \
    echo 'PermitRootLogin no'; \
    echo 'PasswordAuthentication yes'; \
    echo 'Subsystem sftp /usr/lib/openssh/sftp-server'; \
  ) > /etc/ssh/sshd_config_test_clion \
  && mkdir /run/sshd

COPY ./setclion.sh /home/$USER/$ADE_NAME/setclion.sh
COPY ./ade_entrypoint /
