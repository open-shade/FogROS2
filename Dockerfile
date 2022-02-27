ARG FROM_IMAGE=ros:rolling
FROM $FROM_IMAGE

RUN apt update && sudo apt install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-pip \
  python3-vcstool \
  wget \
  emacs-nox \
  unzip \
  wireguard \
  iproute2 \
  curl \
  net-tools \
  libeigen3-dev \
  libassimp-dev \
  libccd-dev \
  ssh && \
  rm -rf /var/lib/apt/lists/* 
  # libfcl-dev

# install some pip packages needed for testing
RUN python3 -m pip install --no-cache-dir -U \
  argcomplete \
  flake8 \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest \
  pytest-cov \
  pytest-runner \
  setuptools

RUN curl "https://awscli.amazonaws.com/awscli-exe-linux-x86_64.zip" -o "awscliv2.zip"
RUN unzip awscliv2.zip && rm awscliv2.zip
RUN ./aws/install
RUN pip3 install boto3 paramiko scp wgconfig

# Create FogROS2 worspace and build it
RUN mkdir -p /home/root/fog_ws/src
WORKDIR /home/root/fog_ws/src
COPY .  /home/root/fog_ws/src/fogros2
COPY ./fogros2/configs/cyclonedds.xml /home/root/fog_ws

ARG USERNAME=root

# FCL
RUN cd ~ && mkdir fcl && \
  curl -L https://github.com/flexible-collision-library/fcl/archive/refs/tags/v0.6.0.tar.gz | tar -xz -C fcl --strip-components=1 && \
  mkdir fcl/build && cd fcl/build && cmake -DCMAKE_BUILD_TYPE=Release .. && \
  make -j4 && sudo make install
#$(nproc)

# vscode remote env
RUN mkdir -p /home/$USERNAME/.vscode-server/extensions \
        /home/$USERNAME/.vscode-server-insiders/extensions \
    && chown -R $USERNAME \
        /home/$USERNAME/.vscode-server \
        /home/$USERNAME/.vscode-server-insiders

# Need to comment for building a new docker image
WORKDIR /home/root/fog_ws
RUN . /opt/ros/rolling/setup.sh && \
      colcon build --merge-install

CMD ["bash"]