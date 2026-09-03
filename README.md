# UAV Red/White Mission Target System

A two-computer UAV vision prototype for detecting a red/white roof marker on a vehicle, estimating its ground position, recording mission evidence with ROS 2, and presenting live status to an operator.

The repository contains the latest integrated version recovered from a development file bundle. It is organized into an airborne Jetson Orin application and a separate PyQt5 ground station.

> [!CAUTION]
> This is research software that can issue a MAVLink Return-to-Launch (RTL) command. Keep `AUTO_RTL=0` during setup, simulation, and bench testing. Validate camera calibration, coordinate output, target confirmation, flight-controller mode mapping, failsafes, and operator override procedures before any flight.

## Features

- RTSP, GStreamer, V4L2, or Intel RealSense camera input
- YOLO-based vehicle detection and red/white mission-marker verification
- Camera/aircraft geometry-based target geolocation
- MAVLink GPS, attitude, battery, velocity, and flight-state telemetry
- HTTP MJPEG/status endpoints and Socket.IO events
- ROS 2 topic publication and split/compressed rosbag recording
- Optional, configurable automatic RTL after target confirmation
- PyQt5 operator display for video, telemetry, detection, and target coordinates

## System architecture

```text
G3P / D455 camera ─┐
                   ├─> Jetson Orin detector ── HTTP + Socket.IO ──> Ground station GUI
Flight controller ─┘            │
       MAVLink                  ├─> JSONL / CSV mission records
                                ├─> ROS 2 topics / rosbag
                                └─> optional MAVLink RTL command
```

## Repository layout

```text
.
├── airborne/
│   ├── red_white_target_server.py       # main mission detector/API
│   ├── orin_d455i_precision_tracker.py  # camera, telemetry and geolocation core
│   ├── run_mission.sh                   # preflight, rosbag and application launcher
│   ├── setup_g3p_network.sh             # G3P Ethernet setup
│   └── start_g3p_viewfinder.py          # G3P RTSP control helper
├── ground_station/
│   ├── red_white_ground_gui.py          # operator application
│   └── run_ground_station.sh            # Linux GUI launcher
├── docs/cleanup-audit.md                # original-program review and cleanup record
├── requirements-airborne.txt
└── requirements-ground.txt
```

## Important: missing project assets

The source bundle supplied for this cleanup was incomplete. The following files are required but are **not** included:

1. `detect_red_white_x.py` — must provide `Detection`, `detect_markers`, and `white_mask_bgr`. Place it in `airborne/` or set `ORIN_CODE_ROOT` to a directory containing it (or a `red_white/` subdirectory).
2. A trained Ultralytics vehicle model, normally `VisDrone_train1_5090best.pt` or its TensorRT `.engine` export.

The mission server will not start without the marker detector. The launcher will reject a missing vehicle model unless `DISABLE_VEHICLE_MODEL=1` is set for a limited non-YOLO test. No replacement detector was invented during cleanup because its flight-triggering behavior must match the model and marker specification used by the aircraft team.

## Requirements

### Airborne computer

- NVIDIA Jetson Orin running Ubuntu 22.04 / JetPack
- Python 3.10 or 3.11
- ROS 2 Humble, including `rclpy`, `sensor_msgs`, `geometry_msgs`, and `std_msgs`
- A JetPack-compatible CUDA PyTorch build
- G3P RTSP camera or Intel RealSense D455/D455i
- MAVLink-compatible flight controller and serial connection
- `ffmpeg`, `iproute2`, `curl`, and standard Linux networking tools

Install the Python packages after installing the Jetson-compatible PyTorch build:

```bash
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install --upgrade pip
python3 -m pip install -r requirements-airborne.txt
```

On Jetson, install RealSense and ROS packages using the vendor/ROS instructions for the installed JetPack and ROS distribution. The `pyrealsense2` wheel is intentionally not selected automatically on ARM64.

### Ground-station computer

- Linux desktop with Python 3.10+
- Network access to the Orin (the launch defaults assume `10.0.0.8`)
- An X11/desktop session for PyQt5

```bash
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install --upgrade pip
python3 -m pip install -r requirements-ground.txt
```

## Configuration

The launchers are configured with environment variables. Defaults reflect the recovered test setup and will usually need adjustment.

| Variable | Default | Meaning |
| --- | --- | --- |
| `CAMERA_SOURCE` | `rtsp` | `rtsp`, `gstreamer`, `v4l2`, or `d455` |
| `RTSP_URL` | `rtsp://192.168.144.135/live` | G3P video stream |
| `G3P_IFACE` | `eno1` | Ethernet interface connected to the camera |
| `MAVLINK` | hardware-specific `/dev/serial/by-id/...` | Flight-controller serial device |
| `VEHICLE_MODEL` | local `.engine` if found, otherwise `.pt` | YOLO/TensorRT model path |
| `RECORD_ROOT` | `./uav-mission-recordings` | Mission output root (or under `XDG_DATA_HOME` when set) |
| `PORT` | `5001` | Airborne HTTP and Socket.IO port |
| `AUTO_RTL` | `1` | Enables automatic RTL; set to `0` until validated |
| `RECORD_ROSBAG` | `1` | Enables ROS 2 bag recording |
| `ORIN_CODE_ROOT` | auto-detected | Optional directory containing shared airborne modules |

`airborne/run_mission.sh` contains the full list of tunable thresholds, camera calibration values, rosbag limits, and recovery controls. Treat the camera roll/pitch/yaw and intrinsic values as calibration data, not generic defaults.

## Run the airborne application

First make the scripts executable:

```bash
chmod +x airborne/*.sh ground_station/*.sh
```

For the first bench test, disable RTL and optionally use mock flight telemetry:

```bash
AUTO_RTL=0 \
MOCK_FLIGHT=1 \
VEHICLE_MODEL=/absolute/path/to/model.pt \
./airborne/run_mission.sh
```

For a configured G3P camera and real flight controller:

```bash
AUTO_RTL=0 \
G3P_IFACE=eno1 \
MAVLINK=/dev/serial/by-id/your-flight-controller \
VEHICLE_MODEL=/absolute/path/to/model.engine \
./airborne/run_mission.sh
```

The launcher checks ROS 2, the model, CUDA, the selected TCP port, the camera stream, and Python syntax before starting the detector and rosbag recorder.

## Run the ground station

On the operator laptop:

```bash
ORIN_IP=10.0.0.8 PORT=5001 FULLSCREEN=0 \
./ground_station/run_ground_station.sh
```

The GUI requires `DISPLAY` to be set. Do not launch it from a headless SSH session unless X forwarding is configured.

## HTTP and event interfaces

The airborne server exposes:

- `GET /video` and `GET /video/cam0` — MJPEG output
- `GET /status` — flight, camera, detector, target, recorder, and RTL state
- `GET /health` — lightweight health response
- Socket.IO namespace `/events` — live mission events consumed by the GUI

The server publishes ROS 2 data for annotated/original compressed images, camera/aircraft IMU, GPS, velocity, battery, flight state, detections, and target geolocation. See the `TOPICS` array in `airborne/run_mission.sh` for the exact recording set.

## Mission output

Each run creates a timestamped session under `RECORD_ROOT`, normally containing:

- `mission_summary.csv`
- `events.jsonl`
- a split/compressed `rosbag/` directory when recording is enabled

Runtime output, trained models, captures, and rosbag databases are excluded by `.gitignore` so they are not accidentally uploaded to GitHub.

## Validation

The retained Python files pass static bytecode compilation. Full end-to-end execution was not possible in the cleanup environment because the marker-detector source, trained model, Jetson CUDA stack, ROS 2 installation, cameras, and flight controller were not available.

Before flight, verify at minimum:

1. The camera stream is stable at the required resolution and frame rate.
2. Camera intrinsics, mounting rotation, and lever-arm offsets are measured correctly.
3. Target coordinates are validated against surveyed ground truth.
4. False-positive behavior is measured on representative non-target vehicles.
5. RTL is tested in SITL and then in a controlled, restrained hardware procedure.
6. The pilot can override automation and all flight-controller failsafes remain active.

## Reference projects

The documentation structure and system descriptions were informed by established open-source UAV/ROS projects:

- [PixEagle](https://github.com/alireza787b/PixEagle) — companion-computer vision, tracking, MAVLink integration, and operator dashboard organization.
- [PX4 ROS 2 User Guide](https://docs.px4.io/main/en/ros2/user_guide.html) — ROS 2/flight-controller architecture and environment setup conventions.
- [NVIDIA ROS 2 DeepStream](https://github.com/NVIDIA-AI-IOT/ros2_deepstream) — Jetson camera inference prerequisites and ROS 2 detection flow.

These projects are references only; their code was not copied into this repository.

## License

No license was present in the original source bundle. Add an appropriate `LICENSE` file before accepting external contributions or reuse.
