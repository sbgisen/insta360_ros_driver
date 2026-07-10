# insta360_ros_driver

A ROS driver for the Insta360 cameras. This driver is tested on Ubuntu 24.04 with ROS2 Jazzy. The driver has also been verified on the Insta360 X3 cameras.

For X4 cameras, see this [fix](https://github.com/ai4ce/insta360_ros_driver/issues/13#issuecomment-2727005037)

## Installation
To use this driver, you need to first have Insta360 SDK. Please apply for the SDK from the [Insta360 website](https://www.insta360.com/sdk/home). 

```
cd ~/ros2_ws/src
git clone -b jazzy https://github.com/nobuchi/insta360_ros_driver
cd ..
```
Then, the Insta360 libraries need to be installed as follows:
- add the <code>camera</code> and <code>stream</code> header files inside the <code>include</code> directory
- add the <code>libCameraSDK.so</code> library under the <code>lib</code> directory.

Afterwards, install the other required dependencies and build
```
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Before continuing, **make sure the camera is set to dual-lens mode**

The Insta360 requires sudo privilege to be accessed via USB. To compensate for this, a udev configuration can be automatically created that will only request for sudo once. The camera can thus be setup initially via:
```
cd ~/ros2_ws/src/insta360_ros_driver
./setup.sh
```
This creates a symlink  based on the vendor ID of Insta360 cameras. The symlink, in this case <code>/dev/insta</code> is used to grant permissions to the usb port used by the camera.

![setup](docs/setup.png)

**Sometimes, this does not work (e.g. you see "device /dev/insta not found" or something similar). You can try entering the commands manually, since that sometimes sees success, especially for the first time.**
```
echo SUBSYSTEM=='"usb"', ATTR{idVendor}=='"2e1a"', SYMLINK+='"insta"' | sudo tee /etc/udev/rules.d/99-insta.rules
sudo udevadm trigger
sudo chmod 777 /dev/insta
```
**Note that you need to setup permissions every time the camera is turned off or disconnected.**

## Usage
This driver directly publishes the video feed in YUV format, since that is the camera's native setting. Alongside this, the driver also publishes the camera feed as standard BGR images to the <code>/front_camera_image/compressed</code> and <code>/back_camera_image/compressed</code> topics. Note that the compressed images have some amount of latency (~50 ms) compared to the raw output. 

### Camera Bringup
The camera can be brought up with the following launch file
```
ros2 launch insta360_ros_driver bringup.launch.py
```
![bringup](docs/bringup_rqt.png)

A dual fisheye image will be published.

![dual_fisheye](docs/dual_fisheye.png)

#### Published Topics
- /dual_fisheye/image
- /equirectangular/image
- /imu/data
- /imu/data_raw

The launch file has the following optional arguments:
- equirectangular (default="true")

![equirectangular](docs/equirectangular.png)

Whether to enable equirectangular image projection

- undistort (default="false")

Whether to publish front and back rectilinear images

![rectilinear](docs/rectilinear.png)

- exposure_mode (default="auto")

Camera exposure mode. `auto` leaves the camera on its own automatic exposure (no SDK exposure call is
made, matching prior behavior). `manual` applies the `iso` and `shutter_speed` parameters via the SDK's
manual exposure settings.

- iso (default="400")

ISO value applied when `exposure_mode:=manual`. Must be a positive integer, or it is rejected
(logged as an error) and exposure falls back to auto. The SDK documents example ISO values such as
100, 400, 800, 1600, etc. as a guide, not as a strict allowlist.

- shutter_speed (default="0.008")

Shutter speed in seconds (e.g. 0.008 = 1/125s) applied when `exposure_mode:=manual`. Must be a finite
positive value, or it is rejected (logged as an error) and exposure falls back to auto. Example shutter
values from the SDK: 1/30, 1/60, 1/120, etc. (guide only, not a strict allowlist).

- video_resolution (default="3840x1920@20")

Live stream resolution. One of: `1920x960@30`, `2560x1280@30`, `3840x1920@20`, `3840x1920@30`.
An unrecognized value falls back to the default and logs a warning.

Note: 4K (`3840x1920`) is the effective live stream ceiling on real X3 hardware. The SDK also
exposes a `5312x2988@30` (5K) resolution enum, but X3 hardware testing (2026-07) confirmed that
requesting it does not fail -- `StartLiveStreaming` reports success -- while the actual stream
silently falls back to 4K (`3840x1920`), as verified via `ffprobe`. Since this fails silently
instead of erroring, it risked users believing they were capturing at 5K when they were not, so
the `5312x2988@30` option has been removed.

- stream_retry_limit (default="0")

Maximum number of `StartLiveStreaming` retry attempts before giving up and exiting the node. `0`
(default) means unlimited retries while the node is alive, matching the original recovery behavior
(this is relied on for production auto-recovery, since the node itself has no external respawn/restart
mechanism watching it individually). Only set a finite value when explicitly testing an
unsupported/untested `video_resolution` and you want the node to fail fast instead of retrying forever.

The IMU allows for frame stabilization. For instance, you are able to visualize the orientation of the camera.

![IMU](https://github.com/user-attachments/assets/02b50cad-8415-4dde-9014-9ab3a4d415b9)

## Star History

[![Star History Chart](https://api.star-history.com/svg?repos=ai4ce/insta360_ros_driver&type=Date)](https://star-history.com/#ai4ce/insta360_ros_driver&Date)

## Publish YUV

```
ros2 launch insta360_ros_driver bringup_yuv.launch.py 
```


#### Published Topics
- /dual_fisheye/yuv_image
- /imu/data_raw

## Developer: Latency Measurement

For camera timing/jitter measurement results and the `enable_latency_debug_log` debug logging mode, see [docs/latency_measurement.md](docs/latency_measurement.md).
