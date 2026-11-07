# syntax=docker/dockerfile:1
# ROS 2 Jazzy with Gazebo Harmonic (through ros_gz), on ROS's default middleware, with the r3v_sim
# package and the r3v_viewer node. Rendering (the gpu_lidar sensor, the optional GUI, the viewer)
# runs on Mesa's software rasterizers, there is no GPU.
FROM ros:jazzy-ros-base AS base

RUN apt-get update \
  && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-gz \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-rosbridge-server \
    ros-jazzy-rviz2 \
    libgl1-mesa-dri libegl-mesa0 mesa-vulkan-drivers libvulkan1 ffmpeg \
    xvfb x11vnc novnc websockify openbox \
  && rm -rf /var/lib/apt/lists/*

ENV LIBGL_ALWAYS_SOFTWARE=1

FROM base AS builder

RUN apt-get update \
  && apt-get install -y --no-install-recommends curl ca-certificates build-essential \
  && rm -rf /var/lib/apt/lists/*
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs \
  | sh -s -- -y --profile minimal --default-toolchain 1.99.0
ENV PATH=/root/.cargo/bin:$PATH

WORKDIR /ws
COPY --from=renderer --exclude=target --exclude=.git . r3v-renderer/
COPY --from=voxelizer --exclude=target --exclude=.git . r3v-voxelizer/
COPY rust-toolchain.toml ./
COPY src/ src/
RUN --mount=type=cache,target=/root/.cargo/registry --mount=type=cache,target=/build \
  . /opt/ros/jazzy/setup.sh \
  && CARGO_TARGET_DIR=/build/cargo colcon build --base-paths src --event-handlers console_direct-

FROM base

WORKDIR /ws
COPY --from=builder /ws/install /ws/install
RUN echo "source /ws/install/setup.bash" >> /root/.bashrc

COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "r3v_sim", "sim.launch.py"]
