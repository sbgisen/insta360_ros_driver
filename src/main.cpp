#include <camera/camera.h>
#include <camera/device_discovery.h>
#include <camera/photography_settings.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "stream_compose.hpp"

// Options for combining the two lens streams (stream_index 0 / 1) into one
// side-by-side H.264 stream.
struct ComposeOptions
{
  bool enable = true;
  stream_compose::CombinerConfig combiner;
  double detect_timeout_sec = 3.0;
};

class TestStreamDelegate : public ins_camera::StreamDelegate
{
private:
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  // LIDAR-075: camera latency measurement instrumentation. Disabled by
  // default; when disabled none of this touches the published messages.
  bool enable_latency_debug_log_;
  std::shared_ptr<ins_camera::Camera> cam_;
  std::ofstream latency_log_;
  std::atomic<int64_t> frame_seq_{0};
  std::atomic<double> last_exposure_timestamp_{-1.0};
  std::atomic<int64_t> exposure_event_count_{0};

  // Per stream_index receive statistics, logged once per second. SDK 5.7K+
  // previews deliver two video streams (stream_index 0 and 1); this makes it
  // visible how many packets/bytes each one carries.
  struct StreamStats
  {
    int64_t count = 0;
    size_t bytes = 0;
    size_t min_size = 0;
    size_t max_size = 0;
    int64_t last_timestamp = 0;
  };
  static constexpr int kMaxLoggedStreams = 4;
  std::mutex stats_mutex_;
  StreamStats stats_[kMaxLoggedStreams];
  std::chrono::steady_clock::time_point last_stats_log_ = std::chrono::steady_clock::now();

  // Dual-lens composition. The mode is decided once: dual as soon as the first
  // stream_index 1 packet arrives, single (plain passthrough of stream 0, as
  // before) if none shows up within detect_timeout_sec. It never switches back,
  // so the topic never mixes 2880x2880 and side-by-side frames.
  enum class Mode
  {
    kDetect,
    kDual,
    kSingle
  };
  struct Job
  {
    int stream_index;
    int64_t timestamp;
    rclcpp::Time stamp;
    std::vector<uint8_t> data;
  };
  static constexpr size_t kMaxQueuedJobs = 40;
  ComposeOptions compose_;
  std::atomic<Mode> mode_{Mode::kDetect};
  std::chrono::steady_clock::time_point first_packet_time_;
  bool first_packet_seen_ = false;
  std::unique_ptr<stream_compose::DualStreamCombiner> combiner_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::deque<Job> queue_;
  std::atomic<bool> stop_worker_{false};
  std::atomic<int64_t> dropped_jobs_{0};
  std::thread worker_;

  void PublishPacket(const uint8_t * data, size_t size, int64_t timestamp, const rclcpp::Time & stamp)
  {
    auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
    msg->header.stamp = stamp;
    msg->header.frame_id = "camera_frame";
    // The subscriber will need to know this to select the correct decoder.
    msg->format = "h264";
    msg->data.assign(data, data + size);
    compressed_pub_->publish(std::move(msg));

    if (enable_latency_debug_log_ && latency_log_.is_open()) {
      const int64_t seq = frame_seq_.fetch_add(1);
      // NOTE: GetCameraMediaTime() を live stream 中（OnVideoData コールバック内）から
      // 呼んではならない。実機検証 (LIDAR-079) で、1回でも呼ぶとカメラの内部コマンド
      // チャンネルがブロックされ video stream が完全停止することが100%再現で確認された。
      // media_time 列は既存 CSV パーサとの互換性のため残すが、値は常に -1.0 固定とする。
      // 詳細: queue/projects/lidar-colorization/reports/lidar079_camera_latency_results.md
      const double media_time = -1.0;
      latency_log_ << seq << ',' << timestamp << ',' << std::fixed << std::setprecision(6)
                   << last_exposure_timestamp_.load() << ',' << media_time << ',' << stamp.seconds() << '\n';
      latency_log_.flush();
    }
  }

  // Runs on its own thread: decoding/encoding must never block the SDK
  // callback (a blocked callback has stalled the stream before).
  void WorkerLoop()
  {
    // header.stamp of a pair is the arrival time of its lens 0 packet, taken in
    // the SDK callback, so decode/encode latency does not shift it.
    std::deque<std::pair<int64_t, rclcpp::Time>> stamps;
    std::vector<uint8_t> out;
    while (true) {
      Job job;
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_cv_.wait(lock, [this] { return stop_worker_ || !queue_.empty(); });
        if (stop_worker_) {
          return;
        }
        job = std::move(queue_.front());
        queue_.pop_front();
      }
      if (mode_ == Mode::kSingle) {
        continue;
      }
      if (job.stream_index == 0) {
        stamps.emplace_back(job.timestamp, job.stamp);
        if (stamps.size() > 32) {
          stamps.pop_front();
        }
      }
      int64_t out_ts = 0;
      if (!combiner_->PushPacket(job.stream_index, job.data.data(), job.data.size(), job.timestamp, &out, &out_ts)) {
        continue;
      }
      rclcpp::Time stamp = node_->get_clock()->now();
      for (const auto & kv : stamps) {
        if (kv.first == out_ts) {
          stamp = kv.second;
        }
      }
      PublishPacket(out.data(), out.size(), out_ts, stamp);
    }
  }

  void EnqueueJob(const uint8_t * data, size_t size, int64_t timestamp, int stream_index, const rclcpp::Time & stamp)
  {
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      if (queue_.size() >= kMaxQueuedJobs) {
        queue_.pop_front();
        const int64_t n = ++dropped_jobs_;
        if (n == 1 || n % 100 == 0) {
          RCLCPP_WARN(node_->get_logger(), "Composer queue full; dropped %ld packets so far.", static_cast<long>(n));
        }
      }
      queue_.push_back(Job{stream_index, timestamp, stamp, std::vector<uint8_t>(data, data + size)});
    }
    queue_cv_.notify_one();
  }

  void RecordStreamStats(size_t size, int64_t timestamp, int stream_index)
  {
    if (stream_index < 0 || stream_index >= kMaxLoggedStreams) {
      return;
    }
    std::lock_guard<std::mutex> lock(stats_mutex_);
    StreamStats & s = stats_[stream_index];
    s.min_size = s.count == 0 ? size : std::min(s.min_size, size);
    s.max_size = std::max(s.max_size, size);
    s.bytes += size;
    s.last_timestamp = timestamp;
    ++s.count;

    const auto now = std::chrono::steady_clock::now();
    if (now - last_stats_log_ < std::chrono::seconds(1)) {
      return;
    }
    last_stats_log_ = now;
    for (int i = 0; i < kMaxLoggedStreams; ++i) {
      if (stats_[i].count > 0) {
        RCLCPP_INFO(
          node_->get_logger(), "stream_index=%d count=%ld bytes=%zu size_min=%zu size_max=%zu last_timestamp=%ld", i,
          static_cast<long>(stats_[i].count), stats_[i].bytes, stats_[i].min_size, stats_[i].max_size,
          static_cast<long>(stats_[i].last_timestamp));
      }
    }
  }

public:
  TestStreamDelegate(
    const std::shared_ptr<rclcpp::Node> & node, bool enable_latency_debug_log,
    const std::string & latency_debug_log_path, const std::shared_ptr<ins_camera::Camera> & cam,
    const ComposeOptions & compose)
  : node_(node), enable_latency_debug_log_(enable_latency_debug_log), cam_(cam), compose_(compose)
  {
    // Publisher for the compressed H.264 video stream
    compressed_pub_ =
      node_->create_publisher<sensor_msgs::msg::CompressedImage>("/dual_fisheye/image/compressed", rclcpp::QoS(10));

    // Publisher for IMU data (remains the same)
    imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", rclcpp::SensorDataQoS());
    RCLCPP_INFO(node_->get_logger(), "Publisher for compressed images and IMU created.");

    if (compose_.enable) {
      combiner_ = std::make_unique<stream_compose::DualStreamCombiner>(compose_.combiner);
      if (combiner_->encoder_ok()) {
        worker_ = std::thread([this] { WorkerLoop(); });
        RCLCPP_INFO(
          node_->get_logger(), "Side-by-side composition enabled (eye_size=%d swap_lenses=%d pair_tolerance=%ld).",
          compose_.combiner.eye_size, compose_.combiner.swap_lenses,
          static_cast<long>(compose_.combiner.pair_tolerance));
      } else {
        RCLCPP_ERROR(node_->get_logger(), "H.264 encoder (libx264) unavailable; publishing stream 0 as is.");
        compose_.enable = false;
      }
    }

    if (enable_latency_debug_log_) {
      latency_log_.open(latency_debug_log_path, std::ios::out | std::ios::trunc);
      if (latency_log_.is_open()) {
        latency_log_ << "frame_seq,sdk_timestamp,exposure_timestamp,media_time,now_stamp_sec\n";
        latency_log_.flush();
        RCLCPP_INFO(node_->get_logger(), "Latency debug log enabled: %s", latency_debug_log_path.c_str());
      } else {
        RCLCPP_ERROR(node_->get_logger(), "Failed to open latency debug log at %s", latency_debug_log_path.c_str());
      }
    }
  }

  virtual ~TestStreamDelegate()
  {
    stop_worker_ = true;
    queue_cv_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
    if (latency_log_.is_open()) {
      latency_log_.close();
    }
  }

  void OnAudioData(const uint8_t * data, size_t size, int64_t timestamp) override {}

  void OnVideoData(const uint8_t * data, size_t size, int64_t timestamp, uint8_t streamType, int stream_index) override
  {
    RecordStreamStats(size, timestamp, stream_index);

    if (!compose_.enable) {
      PublishStream0(data, size, timestamp, stream_index);
      return;
    }

    const auto steady_now = std::chrono::steady_clock::now();
    if (!first_packet_seen_) {
      first_packet_seen_ = true;
      first_packet_time_ = steady_now;
    }
    Mode mode = mode_.load();
    if (mode == Mode::kDetect) {
      if (stream_index == 1) {
        mode_ = mode = Mode::kDual;
        RCLCPP_INFO(node_->get_logger(), "Two video streams detected; publishing side-by-side frames.");
      } else if (std::chrono::duration<double>(steady_now - first_packet_time_).count() > compose_.detect_timeout_sec) {
        mode_ = mode = Mode::kSingle;
        RCLCPP_WARN(
          node_->get_logger(),
          "No stream_index 1 within %.1f s; falling back to publishing stream 0 only (single lens, as before).",
          compose_.detect_timeout_sec);
      }
    }

    if (mode == Mode::kSingle) {
      PublishStream0(data, size, timestamp, stream_index);
    } else if (size > 0 && (stream_index == 0 || stream_index == 1)) {
      // kDetect: stream 0 is already decoded so the composer is warm when stream 1 shows up.
      EnqueueJob(data, size, timestamp, stream_index, node_->get_clock()->now());
    }
  }

  void PublishStream0(const uint8_t * data, size_t size, int64_t timestamp, int stream_index)
  {
    // Plain passthrough of the main video stream (index 0)
    if (stream_index == 0 && size > 0 && compressed_pub_) {
      PublishPacket(data, size, timestamp, node_->get_clock()->now());
    }
  }

  void OnGyroData(const std::vector<ins_camera::GyroData> & data) override
  {
    for (const auto & gyro : data) {
      auto msg = std::make_unique<sensor_msgs::msg::Imu>();
      msg->header.stamp = node_->get_clock()->now();
      msg->header.frame_id = "imu_frame";
      msg->angular_velocity.x = gyro.gx;
      msg->angular_velocity.y = gyro.gy;
      msg->angular_velocity.z = gyro.gz;

      msg->linear_acceleration.x = gyro.ax * 9.80665;
      msg->linear_acceleration.y = gyro.ay * 9.80665;
      msg->linear_acceleration.z = gyro.az * 9.80665;

      msg->orientation.x = 0.0;
      msg->orientation.y = 0.0;
      msg->orientation.z = 0.0;
      msg->orientation.w = 1.0;               // Neutral orientation
      msg->orientation_covariance[0] = -1.0;  // No orientation data available

      for (int i = 0; i < 9; i++) {
        msg->angular_velocity_covariance[i] = 0;
        msg->linear_acceleration_covariance[i] = 0;
      }
      imu_pub_->publish(std::move(msg));
    }
  }

  void OnExposureData(const ins_camera::ExposureData & data) override
  {
    if (enable_latency_debug_log_) {
      last_exposure_timestamp_.store(data.timestamp);
      const int64_t count = exposure_event_count_.fetch_add(1);
      if (count == 0) {
        RCLCPP_INFO(
          node_->get_logger(), "OnExposureData first call observed: timestamp=%f exposure_time=%f", data.timestamp,
          data.exposure_time);
      }
    }
  }
};

namespace
{
// Resolves the `video_resolution` ROS parameter string to the SDK enum.
// Falls back to the default (RES_3840_1920P20, matching the previous
// hardcoded behavior) and logs a WARN when the string is not recognized.
ins_camera::VideoResolution ResolveVideoResolution(const std::string & value, const rclcpp::Logger & logger)
{
  if (value == "1920x960@30") {
    return ins_camera::VideoResolution::RES_1920_960P30;
  } else if (value == "2560x1280@30") {
    return ins_camera::VideoResolution::RES_2560_1280P30;
  } else if (value == "3840x1920@20") {
    return ins_camera::VideoResolution::RES_3840_1920P20;
  } else if (value == "3840x1920@30") {
    return ins_camera::VideoResolution::RES_3840_1920P30;
  }
  RCLCPP_WARN(logger, "Unknown video_resolution '%s'. Falling back to default 3840x1920@20.", value.c_str());
  return ins_camera::VideoResolution::RES_3840_1920P20;
}
}  // namespace

class CameraWrapper
{
private:
  std::shared_ptr<ins_camera::Camera> cam;
  std::shared_ptr<rclcpp::Node> node_;
  std::atomic<bool> streaming_{false};

  // StopLiveStreaming()/Close() perform a request/response handshake with
  // the camera over USB. journalctl evidence from a live reboot (2026-07-06
  // 12:15:31-41) shows this handshake can stall for 15+ seconds when the
  // camera doesn't answer promptly: ros2 launch's default shutdown
  // escalation (SIGTERM 5s after SIGINT, SIGKILL 10s after that) fired
  // and killed the process with pid still inside this call, exit code -9.
  // A SIGKILL skips every destructor, so the camera never receives the
  // stop/close handshake at all -- which lines up with the camera later
  // ignoring BLE wake-up (and even the official app) until it's forcibly
  // power-cycled. Bounding the wait here means the worst case is "we gave
  // up and exited anyway", which cannot be worse for the camera's state
  // than a SIGKILL, while letting the process reliably exit inside the
  // escalation window instead of gambling on it.
  static constexpr auto kStopTimeout = std::chrono::seconds(4);

public:
  CameraWrapper(const std::shared_ptr<rclcpp::Node> & node) : node_(node) {}

  ~CameraWrapper() { stop(); }

  // Stops the live stream and closes the camera. Safe to call multiple
  // times (e.g. once from rclcpp::on_shutdown() and once from the
  // destructor) since cam is reset to nullptr after the first call.
  void stop()
  {
    if (!cam) {
      return;
    }
    auto local_cam = std::move(cam);
    const bool was_streaming = streaming_.exchange(false);

    // Run the actual SDK calls on a detached thread with a bounded wait
    // (rather than std::async, whose future would block on destruction
    // until the task finishes, defeating the timeout) so a stuck camera
    // can no longer take the whole process down with it via SIGKILL.
    auto done = std::make_shared<std::atomic<bool>>(false);
    std::thread([local_cam, was_streaming, done]() {
      if (was_streaming) {
        local_cam->StopLiveStreaming();
      }
      // ShutdownCamera() explicitly tells the camera to power itself
      // off, which is what puts it into the BLE-wake-listening standby
      // state the same way the power button / official app does.
      // Close() alone only tears down our local connection object and
      // does not appear to trigger this -- leaving the camera
      // connected-but-abandoned from its own point of view, which
      // matches the observed symptom (a camera left in that state
      // answers neither BLE wake-up nor the official app until it is
      // forcibly power-cycled).
      local_cam->ShutdownCamera();
      local_cam->Close();
      done->store(true);
    }).detach();

    const auto deadline = std::chrono::steady_clock::now() + kStopTimeout;
    while (!done->load() && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    if (!done->load()) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Camera did not finish StopLiveStreaming()/ShutdownCamera()/Close() within the timeout; "
        "giving up so the process can still exit before ros2 launch escalates to SIGKILL.");
    }
  }

  int run_camera()
  {
    // ROS parameters controlling manual exposure and live-stream
    // resolution. Defaults ("auto" / 3840x1920@20) reproduce the
    // previous hardcoded behavior exactly.
    const std::string exposure_mode_param = node_->declare_parameter<std::string>("exposure_mode", "auto");
    const int iso_param = node_->declare_parameter<int>("iso", 400);
    const double shutter_speed_param = node_->declare_parameter<double>("shutter_speed", 0.008);
    const std::string video_resolution_param =
      node_->declare_parameter<std::string>("video_resolution", "3840x1920@20");
    // 0 = unlimited (default; matches the original behavior of retrying
    // forever while rclcpp::ok()). A positive value is opt-in, intended for
    // explicitly testing an unsupported/untested resolution without hanging.
    const int stream_retry_limit_param = node_->declare_parameter<int>("stream_retry_limit", 0);

    ins_camera::DeviceDiscovery discovery;

    // Wait until a camera is enumerated. This lets the launch be started
    // before the camera is plugged in / switched into USB streaming mode,
    // and avoids an immediate "no camera found" failure on reconnection.
    std::vector<ins_camera::DeviceDescriptor> list;
    while (rclcpp::ok()) {
      list = discovery.GetAvailableDevices();
      if (!list.empty()) {
        break;
      }
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000,
        "No camera found. Waiting for device (plug in the camera / switch it to USB mode)...");
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    if (!rclcpp::ok()) {
      return -1;
    }

    cam = std::make_shared<ins_camera::Camera>(list[0].info);
    while (rclcpp::ok() && !cam->Open()) {
      RCLCPP_WARN(node_->get_logger(), "Failed to open camera. Retrying...");
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    if (!rclcpp::ok()) {
      discovery.FreeDeviceDescriptors(list);
      return -1;
    }
    RCLCPP_INFO(node_->get_logger(), "Camera opened successfully.");
    RCLCPP_INFO(
      node_->get_logger(), "Camera fw_version=%s serial_number=%s", list[0].fw_version.c_str(),
      list[0].serial_number.c_str());
    discovery.FreeDeviceDescriptors(list);

    // LIDAR-075: opt-in camera latency measurement instrumentation.
    // Defaults keep production behavior unchanged.
    const bool enable_latency_debug_log = node_->declare_parameter<bool>("enable_latency_debug_log", false);
    const std::string latency_debug_log_path =
      node_->declare_parameter<std::string>("latency_debug_log_path", "/tmp/insta360_latency_debug.csv");

    // Dual-lens side-by-side composition (X3 preview >= 5.7K delivers one stream
    // per lens). When only one stream arrives, stream 0 is published as is.
    ComposeOptions compose;
    compose.enable = node_->declare_parameter<bool>("compose_side_by_side", true);
    compose.combiner.eye_size = node_->declare_parameter<int>("eye_size", 1920);
    compose.combiner.swap_lenses = node_->declare_parameter<bool>("swap_lenses", false);
    compose.combiner.pair_tolerance = node_->declare_parameter<int>("pair_tolerance", 20);
    compose.detect_timeout_sec = node_->declare_parameter<double>("dual_stream_detect_timeout_sec", 3.0);
    if (compose.combiner.eye_size < 2 || compose.combiner.eye_size % 2 != 0) {
      RCLCPP_ERROR(node_->get_logger(), "eye_size must be even and >= 2; using 1920.");
      compose.combiner.eye_size = 1920;
    }

    std::shared_ptr<ins_camera::StreamDelegate> delegate =
      std::make_shared<TestStreamDelegate>(node_, enable_latency_debug_log, latency_debug_log_path, cam, compose);
    cam->SetStreamDelegate(delegate);

    uint64_t utc_time = static_cast<uint64_t>(time(NULL));
    uint32_t offset_time = 0;  // no offset from UTC
    cam->SyncLocalTimeToCamera(utc_time, offset_time);

    // exposure_mode=auto (default) skips SDK exposure calls entirely,
    // leaving the camera on its own AUTO exposure exactly as before.
    if (exposure_mode_param == "manual") {
      if (iso_param <= 0 || !std::isfinite(shutter_speed_param) || shutter_speed_param <= 0) {
        RCLCPP_ERROR(
          node_->get_logger(),
          "Invalid manual exposure values (iso=%d, shutter=%f); iso must be > 0 and shutter_speed must be finite "
          "and > 0. Skipping SetExposureSettings and falling back to auto exposure.",
          iso_param, shutter_speed_param);
      } else {
        auto exposure = std::make_shared<ins_camera::ExposureSettings>();
        exposure->SetExposureMode(ins_camera::PhotographyOptions_ExposureMode::MANUAL);
        exposure->SetIso(iso_param);
        exposure->SetShutterSpeed(shutter_speed_param);
        if (!cam->SetExposureSettings(ins_camera::CameraFunctionMode::FUNCTION_MODE_LIVE_STREAM, exposure)) {
          RCLCPP_WARN(
            node_->get_logger(),
            "Failed to apply manual exposure settings (iso=%d, shutter=%f). Continuing with camera defaults.",
            iso_param, shutter_speed_param);
        }
      }
    } else if (exposure_mode_param != "auto") {
      RCLCPP_WARN(
        node_->get_logger(), "Unknown exposure_mode '%s'. Falling back to auto (camera default exposure).",
        exposure_mode_param.c_str());
    }

    ins_camera::LiveStreamParam param;
    // 4K dual-fisheye at 20fps (max fps for this resolution on the X3) by
    // default; overridable via the `video_resolution` ROS parameter.
    param.video_resolution = ResolveVideoResolution(video_resolution_param, node_->get_logger());
    //Possible resolutions (results may vary per model) are:
    //RES_3840_1920P30
    //RES_2560_1280P30
    //RES_1152_1152P30 (this will give 2304 x 1152 at 30 FPS)
    //RES_1920_960P30
    param.lrv_video_resulution = ins_camera::VideoResolution::RES_1440_720P30;
    param.video_bitrate = 1024 * 1024 * 20;  // ~20 Mbps for 4K
    param.enable_audio = false;
    param.using_lrv = false;

    // stream_retry_limit=0 (default) retries forever while rclcpp::ok(),
    // matching the original recovery behavior relied on by the systemd
    // supervision of the launch process. A positive limit is opt-in only,
    // for explicitly testing an unsupported/rejected video_resolution
    // without hanging forever.
    int start_live_streaming_attempts = 0;
    while (rclcpp::ok() && !cam->StartLiveStreaming(param)) {
      ++start_live_streaming_attempts;
      if (stream_retry_limit_param > 0 && start_live_streaming_attempts >= stream_retry_limit_param) {
        RCLCPP_ERROR(
          node_->get_logger(),
          "StartLiveStreaming failed %d times (stream_retry_limit=%d). Check that video_resolution=%s is "
          "supported by this camera model.",
          start_live_streaming_attempts, stream_retry_limit_param, video_resolution_param.c_str());
        return -1;
      }
      RCLCPP_WARN(node_->get_logger(), "Failed to start live streaming. Retrying...");
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    if (!rclcpp::ok()) {
      return -1;
    }

    streaming_ = true;
    RCLCPP_INFO(node_->get_logger(), "Live streaming started.");
    return 0;
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("insta_publisher");

  CameraWrapper camera(node);
  if (camera.run_camera() != 0) {
    rclcpp::shutdown();
    return -1;
  }

  // Ensures the camera is stopped as part of rclcpp's own shutdown
  // sequence, rather than depending solely on `camera`'s destructor
  // running during a normal return from main() (see CameraWrapper::stop()
  // for why that isn't reliable on SIGINT/SIGTERM).
  rclcpp::on_shutdown([&camera]() { camera.stop(); });

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
