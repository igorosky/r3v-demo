#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdint>
#include <limits>
#include <map>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/empty.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "r3v-renderer.hpp"
#include "r3v-voxelizer.hpp"

namespace rr = r3v::renderer;
namespace rv = r3v::voxelizer;

static_assert(sizeof(rv::PointI) == sizeof(rr::R3vPointI));

namespace {

constexpr float kFovY = M_PI / 4;
constexpr float kMinDistance = 0.5f;
constexpr float kMaxDistance = 100.0f;
constexpr auto kStatsInterval = std::chrono::seconds(5);

void check(rr::R3vStatus status, const std::string &what) {
  if (status != rr::R3vStatus::R3V_OK) {
    throw std::runtime_error(what + ": " + rr::r3v_renderer_last_error());
  }
}

tf2::Transform from_msg(const geometry_msgs::msg::Transform &transform) {
  const auto &q = transform.rotation;
  const auto &t = transform.translation;
  return tf2::Transform(tf2::Quaternion(q.x, q.y, q.z, q.w), tf2::Vector3(t.x, t.y, t.z));
}

tf2::Transform from_twist(const rr::R3vTwist &twist) {
  const auto &q = twist.rotational;
  const auto &t = twist.linear;
  return tf2::Transform(tf2::Quaternion(q.x, q.y, q.z, q.w), tf2::Vector3(t.x, t.y, t.z));
}

rr::R3vTwist to_twist(const tf2::Transform &transform) {
  const auto &t = transform.getOrigin();
  const auto q = transform.getRotation();
  return {{float(t.x()), float(t.y()), float(t.z())},
          {float(q.x()), float(q.y()), float(q.z()), float(q.w())}};
}

rr::R3vOrbitCamera initial_camera() {
  return rr::r3v_orbit_camera_with_clip_planes(
      rr::r3v_orbit_camera_new({0.0f, 0.5f, 0.0f}, 17.0f, 30.0f, 40.0f), 0.05f, 500.0f);
}

}  // namespace

class Viewer : public rclcpp::Node {
 public:
  Viewer()
      : Node("r3v_viewer"),
        tf_buffer_(get_clock()),
        tf_listener_(tf_buffer_),
        to_renderer_(tf2::Quaternion(tf2::Vector3(1, 0, 0), -M_PI / 2)),
        camera_(initial_camera()) {
    urdf_ = declare_parameter<std::string>("urdf");
    fixed_frame_ = declare_parameter("fixed_frame", std::string("odom"));
    base_frame_ = declare_parameter("base_frame", std::string("base_link"));
    width_ = declare_parameter("width", 1280);
    height_ = declare_parameter("height", 720);
    fps_ = declare_parameter("fps", 30);
    rtp_url_ = declare_parameter("rtp_url", std::string("rtp://127.0.0.1:5004?pkt_size=1200"));
    bitrate_ = declare_parameter("bitrate", std::string("4M"));
    float voxel_size = declare_parameter("voxel_size", 0.1);

    voxelizer_ = make_voxelizer(voxel_size);
    tesselator_ = rr::r3v_cpu_tesselator_create();
    check(rr::r3v_cpu_tesselator_configure(tesselator_, {voxel_size, false}),
          "Configuring the tesselator");

    clouds_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "lidar/points", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) { on_cloud(*cloud); });
    joints_ = create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10, [this](sensor_msgs::msg::JointState::ConstSharedPtr state) {
          std::lock_guard lock(state_mutex_);
          for (size_t i = 0; i < std::min(state->name.size(), state->position.size()); ++i) {
            joint_positions_[state->name[i]] = state->position[i];
          }
        });
    camera_moves_ = create_subscription<geometry_msgs::msg::Twist>(
        "r3v/camera/move", 10,
        [this](geometry_msgs::msg::Twist::ConstSharedPtr move) { on_camera_move(*move); });
    camera_resets_ = create_subscription<std_msgs::msg::Empty>(
        "r3v/camera/reset", 10, [this](std_msgs::msg::Empty::ConstSharedPtr) {
          std::lock_guard lock(state_mutex_);
          camera_ = initial_camera();
        });

    render_thread_ = std::thread([this] {
      try {
        render_loop();
      } catch (const std::exception &error) {
        RCLCPP_FATAL(get_logger(), "%s", error.what());
        rclcpp::shutdown();
      }
    });
  }

  ~Viewer() override {
    running_ = false;
    if (render_thread_.joinable()) {
      render_thread_.join();
    }
    rr::r3v_cpu_tesselator_drop(tesselator_);
    rv::drop_voxelizer(voxelizer_);
  }

 private:
  struct Visual {
    std::string link;
    size_t index;
    uint64_t instance;
  };

  void *make_voxelizer(float voxel_size) {
    const auto backend = declare_parameter("voxelizer.backend", std::string("single_threaded"));
    const auto threads = declare_parameter<int64_t>("voxelizer.threads", 0);
    const auto buckets = declare_parameter<int64_t>("voxelizer.wgpu_bucket_count", 131072);
    if (backend != "single_threaded" && backend != "multi_threaded" && backend != "wgpu") {
      throw std::invalid_argument(
          "voxelizer.backend must be single_threaded, multi_threaded or wgpu");
    }
    if (threads < 0 || uint64_t(threads) > std::numeric_limits<uintptr_t>::max()) {
      throw std::invalid_argument("voxelizer.threads must be non-negative and fit in uintptr_t");
    }
    if (buckets <= 0 || uint64_t(buckets) > std::numeric_limits<uint32_t>::max()) {
      throw std::invalid_argument("voxelizer.wgpu_bucket_count must be positive and fit in uint32_t");
    }

    const float intensity = declare_parameter("intensity_multiplier", 1.0);
    const uint32_t hit = declare_parameter("hit_multiplier", 1);
    const uint32_t target = declare_parameter("target_multiplier", 2);
    const uint32_t destroy = declare_parameter("destroy_threshold", 0);
    const uint32_t construct = declare_parameter("construct_threshold", 150);
    const uint32_t maximum = declare_parameter("max_value", 1000);
    void *voxelizer = nullptr;
    if (backend == "single_threaded") {
      voxelizer = rv::create_single_threaded_voxelizer(
          voxel_size, intensity, hit, target, destroy, construct, maximum, true);
    } else if (backend == "multi_threaded") {
      voxelizer = rv::create_multi_threaded_voxelizer(
          voxel_size, intensity, hit, target, destroy, construct, maximum, true, uintptr_t(threads));
    } else {
      voxelizer = rv::create_wgpu_voxelizer(
          voxel_size, intensity, hit, target, destroy, construct, maximum, true, uint32_t(buckets));
    }
    if (!voxelizer) {
      throw std::runtime_error(
          "Creating the " + backend + " voxelizer failed" +
          (backend == "wgpu"
               ? " (no compatible compute adapter, or voxelizer.wgpu_bucket_count exceeds its limits)"
               : ""));
    }
    RCLCPP_INFO(get_logger(), "Voxelizer backend: %s", backend.c_str());
    return voxelizer;
  }

  void on_cloud(const sensor_msgs::msg::PointCloud2 &cloud) {
    auto start = std::chrono::steady_clock::now();
    tf2::Transform sensor;
    try {
      sensor = to_renderer_ * from_msg(tf_buffer_
                                           .lookupTransform(fixed_frame_, cloud.header.frame_id,
                                                            rclcpp::Time(cloud.header.stamp),
                                                            rclcpp::Duration::from_seconds(0.5))
                                           .transform);
    } catch (const tf2::TransformException &error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Dropping a cloud: %s", error.what());
      return;
    }

    bool has_intensity = std::any_of(cloud.fields.begin(), cloud.fields.end(),
                                     [](const auto &field) { return field.name == "intensity"; });
    std::optional<sensor_msgs::PointCloud2ConstIterator<float>> intensity;
    if (has_intensity) {
      intensity.emplace(cloud, "intensity");
    }
    std::vector<rv::PointF> points;
    points.reserve(size_t(cloud.width) * cloud.height);
    sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
    for (; x != x.end(); ++x, ++y, ++z) {
      float i = 1.0f;
      if (intensity) {
        i = **intensity;
        ++*intensity;
      }
      if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {
        continue;
      }
      tf2::Vector3 point = sensor * tf2::Vector3(*x, *y, *z);
      points.push_back({float(point.x()), float(point.y()), float(point.z()), i});
    }

    const auto &origin = sensor.getOrigin();
    rv::VoxelsUpdate update = rv::add_points(
        voxelizer_, {float(origin.x()), float(origin.y()), float(origin.z()), 0.0f}, points.data(),
        points.size());
    rr::R3vStatus status;
    {
      std::lock_guard lock(tesselator_mutex_);
      status = rr::r3v_cpu_tesselator_update_input_geometry(
          tesselator_, reinterpret_cast<const rr::R3vPointI *>(update.data), update.len);
    }
    rv::drop_update(update);
    if (status != rr::R3vStatus::R3V_OK) {
      RCLCPP_ERROR(get_logger(), "Updating the voxels: %s", rr::r3v_renderer_last_error());
    }

    stats_clouds_ += 1;
    stats_points_ += points.size();
    stats_voxelize_us_ += std::chrono::duration_cast<std::chrono::microseconds>(
                              std::chrono::steady_clock::now() - start)
                              .count();
  }

  void on_camera_move(const geometry_msgs::msg::Twist &move) {
    std::lock_guard lock(state_mutex_);
    float yaw = move.angular.z;
    float pitch = move.angular.y;
    if (yaw != 0.0f || pitch != 0.0f) {
      rr::r3v_orbit_camera_rotate(&camera_, -yaw, pitch);
    }
    if (move.linear.x != 0.0 || move.linear.y != 0.0) {
      float visible_height = 2.0f * camera_.distance * std::tan(kFovY / 2.0f);
      rr::r3v_orbit_camera_pan(&camera_, -move.linear.x * visible_height,
                               move.linear.y * visible_height);
    }
    if (move.linear.z != 0.0) {
      float distance = std::clamp(camera_.distance * std::exp(float(move.linear.z)), kMinDistance,
                                  kMaxDistance);
      rr::r3v_orbit_camera_zoom(&camera_, distance - camera_.distance);
    }
  }

  void load_robot(rr::R3vWgpuRenderer *renderer) {
    description_ = rr::r3v_urdf_read_file(urdf_.c_str());
    if (!description_) {
      throw std::runtime_error("Reading " + urdf_ + ": " + rr::r3v_renderer_last_error());
    }
    robot_ = rr::r3v_urdf_robot_from_description(description_);
    if (!robot_) {
      throw std::runtime_error(std::string("Loading the robot: ") + rr::r3v_renderer_last_error());
    }
    for (size_t l = 0; l < rr::r3v_urdf_description_link_count(description_); ++l) {
      rr::R3vUrdfLink link;
      check(rr::r3v_urdf_description_link(description_, l, &link), "Reading a link");
      for (size_t v = 0; v < link.visual_count; ++v) {
        rr::R3vUrdfVisual visual;
        check(rr::r3v_urdf_description_visual(description_, l, v, &visual), "Reading a visual");
        if (visual.geometry.kind != rr::R3vUrdfGeometryKind::R3V_URDF_GEOMETRY_MESH) {
          continue;
        }
        rr::R3vMesh *mesh = rr::r3v_stl_read_file(visual.geometry.mesh_filename);
        if (!mesh) {
          throw std::runtime_error(std::string("Reading ") + visual.geometry.mesh_filename + ": " +
                                   rr::r3v_renderer_last_error());
        }
        if (visual.geometry.has_mesh_scale) {
          const auto &scale = visual.geometry.mesh_scale;
          rr::R3vTriangle *triangles = rr::r3v_mesh_triangles(mesh);
          for (size_t t = 0; t < rr::r3v_mesh_triangle_count(mesh); ++t) {
            for (float *vertex : {triangles[t].v0, triangles[t].v1, triangles[t].v2}) {
              vertex[0] *= scale.x;
              vertex[1] *= scale.y;
              vertex[2] *= scale.z;
            }
            if (scale.x * scale.y * scale.z < 0.0f) {
              std::swap(triangles[t].v1, triangles[t].v2);
            }
          }
        }
        uint64_t instance;
        rr::R3vStatus status = rr::r3v_wgpu_renderer_upload_asset(renderer, mesh, 1, &instance);
        rr::r3v_mesh_drop(mesh);
        check(status, "Uploading a mesh");
        if (visual.has_material && visual.material.has_color) {
          const float *rgba = visual.material.rgba;
          check(rr::r3v_wgpu_renderer_set_asset_color(renderer, instance, {rgba[0], rgba[1], rgba[2]}),
                "Coloring a mesh");
        }
        visuals_.push_back({link.name, v, instance});
      }
    }
    RCLCPP_INFO(get_logger(), "Loaded robot '%s': %zu visuals",
                rr::r3v_urdf_description_name(description_), visuals_.size());
  }

  void place_robot(rr::R3vWgpuRenderer *renderer, const std::map<std::string, float> &joints) {
    geometry_msgs::msg::TransformStamped base;
    try {
      base = tf_buffer_.lookupTransform(fixed_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException &) {
      return;
    }
    std::vector<const char *> names;
    std::vector<float> positions;
    for (const auto &[name, position] : joints) {
      names.push_back(name.c_str());
      positions.push_back(position);
    }
    rr::R3vRobotTransforms *transforms =
        rr::r3v_urdf_robot_feed_sensor_input(robot_, names.data(), positions.data(), names.size());
    if (!transforms) {
      throw std::runtime_error(std::string("Moving the robot: ") + rr::r3v_renderer_last_error());
    }
    tf2::Transform world = to_renderer_ * from_msg(base.transform);
    for (const auto &visual : visuals_) {
      rr::R3vTwist twist;
      if (rr::r3v_robot_transforms_visual(transforms, visual.link.c_str(), visual.index, &twist) ==
          rr::R3vStatus::R3V_OK) {
        rr::r3v_wgpu_renderer_set_asset_transform(renderer, visual.instance,
                                                  to_twist(world * from_twist(twist)));
      }
    }
    rr::r3v_robot_transforms_drop(transforms);
  }

  std::string ffmpeg_command() const {
    return "ffmpeg -hide_banner -loglevel warning -use_wallclock_as_timestamps 1"
           " -f rawvideo -pix_fmt bgra -video_size " +
           std::to_string(width_) + "x" + std::to_string(height_) +
           " -framerate 1000 -i - -an -c:v libx264 -preset ultrafast -tune zerolatency"
           " -profile:v baseline -pix_fmt yuv420p -crf 23 -maxrate " +
           bitrate_ + " -bufsize 1M -vf fps=" + std::to_string(fps_) +
           " -force_key_frames 'expr:gte(t,n_forced)' -x264-params repeat-headers=1"
           " -f rtp -payload_type 96 '" +
           rtp_url_ + "'";
  }

  void render_loop() {
    rr::R3vWgpuRenderer *renderer = rr::r3v_wgpu_renderer_create();
    if (!renderer) {
      throw std::runtime_error(std::string("No GPU adapter: ") + rr::r3v_renderer_last_error());
    }
    check(rr::r3v_wgpu_renderer_configure(renderer, {width_, height_, {0.12f, 0.13f, 0.16f}, 30.0f,
                                                     rr::r3v_terrain_coloring_default()}),
          "Configuring the renderer");
    load_robot(renderer);
    FILE *output = popen(ffmpeg_command().c_str(), "w");
    if (!output) {
      throw std::runtime_error("Starting ffmpeg failed");
    }
    RCLCPP_INFO(get_logger(), "Streaming %ux%u at %d fps to %s", width_, height_, fps_,
                rtp_url_.c_str());

    std::vector<uint8_t> pixels(size_t(width_) * height_ * 4);
    auto period = std::chrono::nanoseconds(1'000'000'000 / std::max(fps_, 1));
    auto next_frame = std::chrono::steady_clock::now();
    auto stats_since = next_frame;
    uint64_t frames = 0;
    uint64_t render_us = 0;
    while (running_ && rclcpp::ok()) {
      auto start = std::chrono::steady_clock::now();
      rr::R3vOrbitCamera camera;
      std::map<std::string, float> joints;
      {
        std::lock_guard lock(state_mutex_);
        camera = camera_;
        joints = joint_positions_;
      }
      place_robot(renderer, joints);
      rr::R3vCameraState state;
      check(rr::r3v_orbit_camera_get_state(camera, float(width_) / height_, &state),
            "Getting the camera state");
      rr::R3vGeometryDelta *delta = nullptr;
      {
        std::lock_guard lock(tesselator_mutex_);
        check(rr::r3v_cpu_tesselator_get_output_geometry_delta(tesselator_, &state, &delta),
              "Tesselating");
      }
      if (delta) {
        check(rr::r3v_wgpu_renderer_update_geometry(renderer, delta), "Updating the geometry");
      }
      check(rr::r3v_wgpu_renderer_render(renderer, &state, pixels.data(), pixels.size()),
            "Rendering");
      if (std::fwrite(pixels.data(), 1, pixels.size(), output) != pixels.size() ||
          std::fflush(output) != 0) {
        throw std::runtime_error("ffmpeg stopped taking frames");
      }

      auto now = std::chrono::steady_clock::now();
      frames += 1;
      render_us += std::chrono::duration_cast<std::chrono::microseconds>(now - start).count();
      if (now - stats_since >= kStatsInterval) {
        double seconds = std::chrono::duration<double>(now - stats_since).count();
        uint64_t clouds = stats_clouds_.exchange(0);
        RCLCPP_INFO(get_logger(),
                    "%.1f fps (%.1f ms a frame), %.1f clouds/s of %.0f points, voxelized in %.1f ms",
                    frames / seconds, render_us / 1000.0 / std::max<uint64_t>(frames, 1),
                    clouds / seconds,
                    double(stats_points_.exchange(0)) / std::max<uint64_t>(clouds, 1),
                    stats_voxelize_us_.exchange(0) / 1000.0 / std::max<uint64_t>(clouds, 1));
        stats_since = now;
        frames = 0;
        render_us = 0;
      }
      next_frame = std::max(next_frame + period, now);
      std::this_thread::sleep_until(next_frame);
    }

    pclose(output);
    rr::r3v_urdf_robot_drop(robot_);
    rr::r3v_urdf_description_drop(description_);
    rr::r3v_wgpu_renderer_drop(renderer);
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2::Transform to_renderer_;

  std::string urdf_;
  std::string fixed_frame_;
  std::string base_frame_;
  uint32_t width_;
  uint32_t height_;
  int fps_;
  std::string rtp_url_;
  std::string bitrate_;

  void *voxelizer_ = nullptr;
  rr::R3vCpuTesselator *tesselator_ = nullptr;
  std::mutex tesselator_mutex_;

  std::mutex state_mutex_;
  rr::R3vOrbitCamera camera_;
  std::map<std::string, float> joint_positions_;

  rr::R3vUrdfDescription *description_ = nullptr;
  rr::R3vUrdfRobot *robot_ = nullptr;
  std::vector<Visual> visuals_;

  std::atomic<uint64_t> stats_clouds_ = 0;
  std::atomic<uint64_t> stats_points_ = 0;
  std::atomic<uint64_t> stats_voxelize_us_ = 0;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr clouds_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr camera_moves_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr camera_resets_;

  std::atomic<bool> running_ = true;
  std::thread render_thread_;
};

int main(int argc, char **argv) {
  std::signal(SIGPIPE, SIG_IGN);
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<Viewer>());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("r3v_viewer"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
}
