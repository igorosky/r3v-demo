# r3v-demo

A ROS 2 Jazzy / Gazebo Harmonic simulation of a mobile robot with a 3D lidar and moving
obstacles. It builds a voxel map with r3v-voxelizer, renders it with r3v-renderer and streams
video through Janus to r3v-web. Docker provides the full setup with software rendering; no
GPU is required.

`src/sim/` contains the simulation; `src/viewer/` contains the rendering and streaming node.

## Run with Docker

Requires Docker with Compose. Clone with `--recurse-submodules`, or initialize the submodules
in an existing checkout. From this directory:

```bash
git submodule update --init --recursive
docker compose up -d --build
```

This starts the robot patrol, renderer, rosbridge, Janus and the web viewer. The build
uses the checked-out r3v-renderer, r3v-voxelizer and r3v-web submodules, pinned to commits by
this repository. Run `git submodule update --init --recursive` again after pulling updates.

Open http://localhost:5173/examples/janus/. Drag to orbit, right-drag to pan and scroll to zoom.

For development, edit the submodules directly and rebuild. To use another local checkout,
set `R3V_VOXELIZER`, `R3V_RENDERER` or `R3V_WEB` to its path, e.g.:

```bash
R3V_VOXELIZER=../r3v-voxelizer docker compose up -d --build
```

You can combine these variables. Repeat the command after code changes to rebuild and restart
the affected services; local changes do not need to be committed.

Optional: `GUI=1 docker compose up -d` also starts Gazebo's GUI and RViz at
http://localhost:6080/vnc.html.

To stop all services, run from the repository root:

```bash
docker compose down
```

## Run locally

Requires [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
and [Rust via rustup](https://rustup.rs/), plus Docker with Compose for Janus. Run these
commands from the repository root:

```bash
git submodule update --init --recursive
source /opt/ros/jazzy/setup.bash
sudo apt update
sudo apt install -y build-essential python3-colcon-common-extensions \
  ros-jazzy-ros-gz ros-jazzy-robot-state-publisher ros-jazzy-rosbridge-server \
  ros-jazzy-rviz2 libgl1-mesa-dri libegl-mesa0 mesa-vulkan-drivers libvulkan1 ffmpeg

colcon build --base-paths src
source install/setup.bash
docker compose up -d janus
ros2 launch r3v_sim sim.launch.py
```

`colcon build` builds the Rust libraries from the submodules through CMake and installs them
with the viewer. Rustup selects the toolchain from `rust-toolchain.toml`. Repeat `colcon build`
after editing the Rust or C++ code.

This runs Gazebo, the robot patrol, rosbridge and the viewer. The viewer uses local Janus
by default; Compose supplies its internal address automatically. Add `gui:=true` for Gazebo's
GUI and RViz, or `viewer:=false` to run without streaming.

With Node.js 24 installed, start the frontend in another terminal, from the repository root:

```bash
cd r3v-web
npm ci
npm run dev
```

Open http://localhost:5173/examples/janus/. Stop the foreground processes with `Ctrl+C` and
stop Janus with `docker compose down`.

## Voxelizer backend

The default is `single_threaded`. Choose CPU parallelism or GPU compute at startup:

```bash
ros2 launch r3v_sim sim.launch.py voxelizer_backend:=multi_threaded voxelizer_threads:=4
ros2 launch r3v_sim sim.launch.py voxelizer_backend:=wgpu
```

`voxelizer_threads:=0` selects CPU worker threads automatically. GPU compute needs a compatible
adapter and drivers; `voxelizer_wgpu_bucket_count` controls its hashmap size (default 131072).
The corresponding ROS parameters are `voxelizer.backend`, `voxelizer.threads` and
`voxelizer.wgpu_bucket_count`, matching r3v-ros. Rebuild once after updating to enable all backends.

Known issue in the pinned voxelizer: `wgpu` readback crashes on the tested ARM64 setups
(Apple Metal and Linux Mesa/llvmpipe). Use a CPU backend on these setups until the voxelizer
is fixed; GPU operation has not been verified on other hardware.

For CPU parallelism in Docker:

```bash
VOXELIZER_BACKEND=multi_threaded VOXELIZER_THREADS=4 docker compose up -d --build
```
