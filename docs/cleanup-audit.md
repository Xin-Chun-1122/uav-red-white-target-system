# Cleanup audit

This audit records the purpose of every program found in the original file bundle and the disposition chosen during cleanup. The latest integrated mission-target implementation was retained; generated data, caches, broken copies, and superseded prototypes were removed.

## Retained programs

| Current file | Purpose |
| --- | --- |
| `airborne/red_white_target_server.py` | Main Jetson Orin application. Reads the mission camera, detects a red/white roof marker on a vehicle, tracks candidates, estimates target coordinates from camera geometry and MAVLink telemetry, publishes HTTP/Socket.IO status, records mission events, publishes ROS 2 topics, and can request RTL. |
| `airborne/orin_d455i_precision_tracker.py` | Shared camera, telemetry, geolocation, ROS 2 publishing, recording, and generic YOLO tracking primitives used by the mission server. It can also run as a standalone D455 vehicle tracker. |
| `airborne/run_mission.sh` | Production launcher and preflight script. Configures the camera/model/MAVLink settings, checks CUDA and RTSP, starts ROS 2 bag recording, launches the server, and cleans up child processes. |
| `airborne/setup_g3p_network.sh` | Configures the wired route to the G3P camera and optionally sends camera-control commands. |
| `airborne/start_g3p_viewfinder.py` | Sends the G3P camera's UDP control protocol requests used to start or recover the RTSP viewfinder stream. |
| `ground_station/red_white_ground_gui.py` | PyQt5 operator UI. Connects to the Orin HTTP and Socket.IO endpoints, displays the camera/event view, telemetry, GPS and target state. |
| `ground_station/run_ground_station.sh` | Linux launcher for the operator UI, including Qt plugin-path handling and optional fullscreen mode. |

## Removed programs and directories

| Original item | Purpose and removal reason |
| --- | --- |
| `.orin_edit/` | Working copies of the D455 tracker, target server, launcher, `.orig` backups, and bytecode. The reusable tracker was retained; the other files were superseded by the final integrated version. |
| `.orin_roofred/` | Intermediate roof-red detection experiment. Its changes are already incorporated into the final server. |
| `.orin_fix_threshold/` | One-off launcher threshold patch, superseded by the final launcher. |
| `.orin_bag_145945/` | Generated annotated contact sheet and frame from a bag inspection. |
| `.tmp_align_camera_image/` | Temporary clone of an external G3P ROS 2 camera repository, including its nested `.git` database. The two locally used camera utilities were retained from the final integration instead. |
| `real_time/` | Earlier ground-station GUI and launcher. Superseded by the bag/RTL version. |
| `real_time_new_camera_orin/` | Earlier G3P airborne bundle. Its main server contains an unterminated string and fails Python compilation; the later copy was retained. |
| `real_time_bag_rtl/` | Packaging directory for the selected final files. Its code was promoted into the clearer `airborne/` and `ground_station/` layout; generated JSONL and bytecode were discarded. |
| `uav_signal/`, `hub_car.py` | Experimental low-bandwidth, progressive XBee transfer protocol and matching OpenCV ground viewer. This is a separate earlier transport path and is not used by the final HTTP/Socket.IO mission system. |
| `hub_precision.py` | Earlier PyQt precision-tracker dashboard for the generic port-5000 tracker. Superseded by the mission-target GUI on port 5001. |
| `car_0506.py` | Earlier PyQt vehicle event dashboard. Superseded by later ground-station clients. |
| `car_gps_laptop.py` | Earlier dual-TCP video/GPS laptop receiver on ports 8001 and 8003. Superseded by HTTP/Socket.IO. |
| `two_d435_detection_car.py` | Earlier two-camera TCP/OpenCV viewer on ports 8001 and 8002. Superseded by the integrated camera path. |
| `receiver.py` | Earlier length-prefixed TCP event/file receiver on port 9000. Not referenced by the final system. |
| `photo/` | Earlier FastAPI upload receiver/dashboard plus archived event images and logs. Not referenced by the final system. |
| `uav_interface.py`, `templates/`, `start.bat` | First-generation Flask web GCS and Windows launcher using port 9090/5000. Superseded by the PyQt5 mission GUI. |
| `v4l2_d455_infra_publisher.py` | Standalone ROS 2 V4L2 infrared-image publisher. It is unrelated to the selected G3P RGB mission path. |
| `car_events/`, `car_gps_data/` | Generated detection crops, scenes, metadata, and CSV output. Runtime data should not be committed. |
| `g3p_rtsp_test.jpg` | Generated RTSP test frame. |
| all `__pycache__/` directories | Generated Python bytecode caches. |

## Known external assets

The original bundle did **not** contain `detect_red_white_x.py`, although the retained mission server imports it. The original bundle also did not include the YOLO `.pt`/`.engine` model. Both must be supplied separately before the airborne application can start. These omissions are documented prominently in the main README rather than hidden by an unverified replacement implementation.
