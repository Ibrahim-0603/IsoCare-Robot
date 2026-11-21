# marker_detector stage 2: package, node and calibration tool

New files go into `src/isocare_marker_detector/` next to Hesham's `marker_core.py` (nothing of his
is changed). Build with the interface package, because the node publishes `MarkerSighting`:

```
cd ~/isocare_ws
colcon build --packages-select isocare_interfaces isocare_marker_detector 2>&1 | head -20
source install/setup.bash
ros2 pkg executables isocare_marker_detector   # marker_detector and calibrate_camera
```
(`ros2 interface show isocare_interfaces/msg/MarkerSighting` must print the six fields.)

## 1. Calibrate the camera (once per camera and resolution)

Print a checkerboard of 10 x 7 squares (that is 9 x 6 INNER corners), stick it on something flat,
and measure one square with a ruler. Take 15 to 25 views, tilted in different directions, near and
far, covering the corners of the image.

```
ros2 run isocare_marker_detector calibrate_camera capture --pattern 9x6 --square-m 0.025
# space keeps a view (the board must be drawn with coloured corners), q finishes
# or from photos in a folder:
ros2 run isocare_marker_detector calibrate_camera photos ~/calib_photos --pattern 9x6 --square-m 0.025
```
It writes `~/.isocare/camera_calibration.yaml` and refuses a result above 1.5 px error. Use the
same resolution later that you calibrated at.

## 2. Run the node

```
ros2 run isocare_marker_detector marker_detector --ros-args \
  -p marker_size_m:=0.15 -p cam_x_m:=0.10 -p cam_z_m:=0.40
ros2 topic echo /markers
```
Without a calibration file the node stops at start and says so. `-p calibration_file:=PATH` picks
another file. Marker size, `cam_x_m` and `cam_z_m` are placeholders until measured.

## 3. Wiring check with no camera (distances are NOT meaningful here)

This only proves the node sees a marker and publishes. Make a made-up calibration and the marker
image, then publish the image as the camera:

```
python3 -c "
import cv2, numpy as np
fs = cv2.FileStorage('/tmp/fake_cam.yaml', cv2.FILE_STORAGE_WRITE)
fs.write('camera_matrix', np.array([[800., 0, 300.], [0, 800., 300.], [0, 0, 1.]]))
fs.write('distortion_coefficients', np.zeros((5, 1))); fs.release()"
ros2 run image_publisher image_publisher_node /tmp/marker2.png --ros-args -r image_raw:=/camera/image_raw
ros2 run isocare_marker_detector marker_detector --ros-args -p calibration_file:=/tmp/fake_cam.yaml
ros2 topic echo /markers
```
(`/tmp/marker2.png` is the test image from section 8 of `Docs/marker_detector.md`.) You should see
`marker_id: 2`.

## What was and was not verified

- 8 new tests for the calibration tool pass (synthetic checkerboard photos recover the camera
  within 3%, the file loads with Hesham's `load_calibration`); with his 12 tests that is 20.
- The node file ran against a stand-in for rclpy: one correctly typed sighting per marker, the
  image stamp copied, nothing for blank or bad images, a clear stop without a calibration.
- NOT run: the node on real ROS and cv_bridge, a real camera, and `calibrate_camera capture`
  (needs a camera and a display).
