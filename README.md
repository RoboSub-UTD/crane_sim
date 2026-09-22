# CRANE Simulation Tool

## Purpose

The Cross-domain Robotics Autonomous Navigation Engine (CRANE) proposes a generalized and highly modular control stack designed to support autonomous navigation across a wide range of robotic platforms operating in distinct environmental domains, including aerial, underwater, ground, and surface environments. Existing navigation systems are frequently tailored to specific hardware or conditions, limiting portability and reuse. CRANE addresses this challenge by defining universal abstraction layers within perception, planning, and control that enable core navigation behaviors such as state estimation, mapping, waypoint following, obstacle avoidance, recovery behaviors, and high-level decision-making to be reused with minimal platform-specific modification. The project integrates simulation-based development with real-world testing through a shared physics abstraction layer to evaluate the transferability of navigation strategies between different domains. Data collection and cross-platform performance analysis will quantify navigation accuracy, robustness, and efficiency. The overarching objective of CRANE is to demonstrate that a unified control architecture can significantly reduce hardware-dependent redesign and enable scalable, domain-agnostic autonomous navigation. 

## Usage

Clone the repo by running:
```bash
git clone https://github.com/1unarzDev/crane_sim
```

Then, start by installing [Unity Hub](https://docs.unity3d.com/hub/manual/InstallHub.html) for your OS. Add the project (the repo you cloned) by navigating to `Add > Add project from disk`, then selecting the appropriate folder. You will then be prompted to install Unity Engine. 

If your device is running OpenGL graphics, you may need to install [Vulkan](https://www.vulkan.org/tools#vulkan-gpu-resources). After installing Vulkan and Unity Engine, navigate to `Edit > Project Settings > Player > Other Settings > Graphics API for Linux` and make sure it is set to use Vulkan.

Finally, make sure you have selected the correct scene from `Assets/Scenes` and set the `ROSConnection` object IP to the one corresponding to the device you're running ROS on. For more details view the bottom section of the [mhseals_docker repo](https://github.com/mhseals/mhseals_docker).

The simulator has been build with customizability and modularity in mind, all physics scripts can be easily adapted in the `Assets/Scripts` folder. Importing URDFs can also be easily done by right clicking the hierarchy and selecting `3D Object > URDF Model (Import)`. 

## Simulated ZED 2i (ZED SDK simulation mode)

Blastoise carries a virtual ZED 2i (`ZedSimCamera` on `front_camera_link`) that streams a rectified
stereo pair and IMU into the real ZED SDK using Stereolabs' simulation streamer, the same mechanism
the ZED Isaac Sim extension uses. The SDK computes depth from the stereo pair, so
`zed-ros2-wrapper` in `sim_mode` publishes the usual `/zed/zed_node/...` topics (rectified images,
depth, point cloud, IMU) exactly as on the boat. Nothing ZED-related is published from Unity directly.

Notes:
- The camera comes from the SDK's virtual serial pool as a **ZED 2i** (model 3, wide lens, serial
  20976320, 120 mm baseline), so launch the wrapper with `camera_model:=zed2i`. If the SDK reports a
  different model for the opened stream the wrapper only warns; it still runs.
- The two eyes sit where the real camera's lenses do: ±60 mm either side of `front_camera_link`
  (which is the centre of the ZED2i body, as the mesh is drawn) and 10 mm behind it, matching
  `optical_offset_x`/`baseline` in the wrapper's `zed_macro.urdf.xacro`. The left eye is the camera
  `/detections` renders from too, so bounding boxes come from the same view as the depth.
- Rendering defaults to HD720 @ 30 fps (`ZedSimCamera` inspector: resolution, lens, fps, port, serial).
- Requires an NVIDIA GPU (the stream is H.265 via NVENC).

Setup on the Unity host:
1. Linux: `sudo apt install libturbojpeg` (runtime dependency of the streamer library).
2. In Unity: **Tools > ZED Sim > Install streamer library** (downloads ~170 MB into `Assets/Plugins/x86_64`, git-ignored).
3. Press Play. The console prints `ZED streamer ready: SDK 5.4.1, ZED2i serial ..., 1280x720@30
   (fx 529.8, vFOV 68.4°, baseline 120.00 mm), port 30000`.

On the ROS side (see the `zed-sim` service in the `roboboat-docker` repo):
```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i camera_name:=zed \
  sim_mode:=true sim_address:=127.0.0.1 sim_port:=30000 use_sim_time:=true
```
Verified: the wrapper opens the stream, reports fx 529.8 @ 1280x720 in `camera_info`, and publishes
rectified images + NEURAL depth (~10 Hz on an RTX 3050). Known wrapper quirks in sim mode (IMU topic
sporadic, `CORRUPTED FRAME` warnings) are listed in the roboboat-docker README.
- Only one process can own UDP port 30000 on the host: if another streamer (or a stale Unity session)
  holds it, `ZedSimCamera` logs `init_streamer failed` and disables itself until the next Play.
`use_sim_time` consumes the `/clock` published by the `SimClock` object, so ZED stamps line up with
the other simulated sensors. Docker containers must use `network_mode: host` (the stream is RTP/UDP).

### Depth is metric only if the rig matches the SDK's calibration

The SDK does not take our word for the geometry: it computes depth from disparity using the
calibration it attaches to the virtual serial, whatever we stream. Read back from a running wrapper
(HD720, serial 20976320, `/zed/zed_node/*/camera_info`):

| | value |
|---|---|
| `fx` = `fy` | 529.8 px |
| `cx`, `cy` | 640, 360 (image centre) |
| distortion | all zero |
| baseline | `-P[3] / fx` = 63.576 / 529.8 = **120.000 mm** |

`ZedCameraSpecs` holds exactly these numbers, so a point rendered at Z metres comes back at Z metres.
If the rendered geometry ever drifts from them — a scaled parent shrinking the baseline, a physical
or oblique camera moving the principal point — every point in the cloud is wrong by the same factor,
and nothing in the images looks wrong. `ZedSimCamera` therefore re-measures the rig at start-up (the
eyes' world separation, and `fx`/`fy`/`cx`/`cy` read back out of each eye's projection matrix) and
refuses to stream, with the offending number in the error, rather than publish a cloud at the wrong
scale.

To confirm it end to end, tick **Log ground truth depth** on `ZedSimCamera` (it raycasts the scene
through five pixels and logs the true Z along the left eye's optical axis) and read the same pixels
back off the wrapper:

```bash
docker compose exec zed-sim python3 /root/depth_probe.py
```

The two sets of numbers should agree to a few centimetres, on whatever the raycast and the stereo
matcher both hit. A constant *ratio* between them is a baseline or focal-length mismatch; a constant
*offset* is a rig-origin mismatch.
