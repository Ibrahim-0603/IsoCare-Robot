# marker_detector: design decisions and contract (for Youssef Hesham and his AI)

**Status:** v1, Nov 21, 2026. Feature freeze is Sun Nov 22. Attach this file together with
`AI_CONTEXT.md` (project, platform, coding rules) to your AI chat. Ask Ibrahim before changing
anything marked DECIDED. Items marked DEFAULT or PLACEHOLDER are proposals, not team decisions:
change them if you have a reason and tell Ibrahim.

## 1. What the node is

`marker_detector` is **pure perception**. It reads camera images, finds ArUco markers, works out
where each marker is relative to the **robot**, and publishes that. It does **not** reset the
robot's position, steer, or filter over time. Ibrahim's navigator does all of that (section 7).

## 2. DECIDED (the team agreed these)

1. **Markers stand upright on walls at camera height.** No floor markers. One level camera sees
   both markers and beds (YOLO). The camera is level, or tilted at most a few degrees down.
2. **Demo layout:** the doorway is at the end of the hallway, and the marker is stuck to the end
   wall facing the robot, so the robot arrives head-on. No freestanding stand in the hallway.
   Room and bed markers face the door.
3. **Camera pose is a set of parameters.** The camera is on the robot centreline (left offset 0),
   pointing forward and level. Its forward offset, height and tilt are parameters with
   placeholder values until Belal mounts the camera and someone measures them with a tape.
4. **The detector only reports sightings. The navigator decides** whether to reset the pose, and
   it also uses the same sighting for steering. The detector never calls `/odom/set_pose`.
5. **Every sighting is published**, one message per detected marker per frame, with no
   confirmation over several frames. The navigator applies the gates (section 7).

## 3. Interfaces

**Subscribes:** `/camera/image_raw` (`sensor_msgs/Image`, converted with `cv_bridge` to `bgr8`).

**Publishes:** `/markers` (`isocare_interfaces/MarkerSighting`, reliable, depth 10): one message
per detected marker per frame. Nothing is published for a frame with no marker.

The message already exists in the repo as `src/isocare_interfaces/msg/MarkerSighting.msg`
(Ibrahim added it and listed it in that package's `CMakeLists.txt`). Do not edit it: ask Ibrahim
if you need a field changed. Rebuild `isocare_interfaces` after pulling, then import it with
`from isocare_interfaces.msg import MarkerSighting`. Its content is shown here for reference:

```
# One ArUco marker seen in one camera image, expressed relative to the ROBOT (not the camera).
# Robot frame: x forward, y left, angles counter-clockwise positive, as in ROS REP 103.
std_msgs/Header header        # stamp = the camera image's stamp (latency matters)
uint32 marker_id              # the ArUco id
float64 distance_m            # flat-floor distance from the robot centre to the marker centre
float64 bearing_rad           # direction of the marker from the robot's forward axis;
                              # a marker on the robot's left = positive
float64 facing_rel_rad        # the direction the marker's FRONT faces, measured from the
                              # robot's forward axis, wrapped to [-pi, pi]
                              # (robot looking straight at the marker's front: about +/- pi)
float32 side_px               # mean side length of the marker in the image, in pixels
```

Why `facing_rel_rad`: the marker table gives each marker's position and the direction it faces
(`heading_deg`). With a sighting the navigator solves the robot's pose: robot heading = marker
heading minus `facing_rel_rad`, then the robot position is the marker position minus the vector
`distance_m` at `bearing_rad`, rotated by that heading.

## 4. Parameters (all ROS parameters; floats need a decimal point)

| Parameter | Default | Meaning |
| --- | --- | --- |
| `marker_size_m` | 0.15 | PLACEHOLDER: printed side length of the black square (**measure it**) |
| `aruco_dictionary` | `DICT_4X4_50` | DEFAULT. The ids in `markers.yaml` are 0 to 4 |
| `image_topic` | `/camera/image_raw` | |
| `cam_x_m` | 0.10 | PLACEHOLDER: camera forward of the robot centre |
| `cam_y_m` | 0.0 | camera left of the robot centre (on the centreline) |
| `cam_z_m` | 0.40 | PLACEHOLDER: camera height above the floor |
| `cam_pitch_deg` | 0.0 | tilt down (positive = looking down) |
| `calibration_file` | none | DEFAULT: a YAML file with the camera matrix and distortion coefficients |

A real pose needs the camera intrinsics. DEFAULT: calibrate once with a printed checkerboard and
OpenCV `calibrateCamera`, and save the result in `config/camera_calibration.yaml`. If the file is
missing, the node must say so clearly at start and refuse to publish, never guess a lens.

## 5. What each output means (check these with tests, they are easy to get wrong)

Compute the marker pose with OpenCV (`cv2.aruco` detection, then `solvePnP`, preferably
`SOLVEPNP_IPPE_SQUARE`), convert from the camera's optical axes to the robot frame, then apply
the camera offset and tilt. The axis conventions are easy to flip, so **define correctness by
these checks, not by axis names:**

- A marker straight ahead, 1.0 m from the camera and facing the robot: `bearing_rad` is 0
  (within 0.02) and `facing_rel_rad` is +/- pi.
- The same marker moved 0.2 m to the robot's left: `bearing_rad` is positive.
- The same marker rotated so its front turns toward the robot's left: `facing_rel_rad` moves away
  from +/- pi in the counter-clockwise direction.
- Increasing `cam_x_m` by 0.1 m reduces `distance_m` by about 0.1 m for a marker straight ahead
  (the camera is closer to the marker, the robot centre is not).

## 6. Coding and testing rules (the same as `AI_CONTEXT.md`)

- Put all the maths in a ROS-free `marker_core.py`. Keep `marker_detector.py` a thin node.
- Tests need no camera and no ROS: draw a marker with `cv2.aruco.generateImageMarker`, project it
  through a made-up camera matrix at a known pose, run the core, and compare with the known
  answer. Also test: no marker, two markers, a marker id the table does not know (still
  published, the navigator ignores it), a missing calibration file, and a blank image.
- Wire types: `float(...)`, `int(...)`, `str(...)`. A wrong type aborts the whole process.
- Never name your own attributes `_clients`, `_services`, `_subscriptions`, `_publishers`,
  `_timers`, `_guards`, `_waitables` or `_parameters`.
- Detect on every frame: the navigator needs a steady stream while the robot drives.
- Say plainly what you ran. Untested on ROS means untested.

## 7. What Ibrahim's side does with your output (do not build this)

- **Navigator:** runs the gates and calls `/odom/set_pose` when a sighting passes. Planned
  gates, all parameters: marker within about 2 m, the same marker agreeing in 3 consecutive
  frames, not while turning faster than a small rate, and rejecting a pose more than about 0.5 m
  or 30 degrees from odometry unless the robot is lost.
- **Steering and final approach:** uses `bearing_rad`, `distance_m` and `facing_rel_rad`.
- **Marker table:** `isocare_bringup/config/markers.yaml` holds the positions, facing directions
  and edges. You do not read it, the navigator does.

## 8. Check it with a test image

Make an image of marker id 2 (the bed A marker in the placeholder map):

```
python3 -c "
import cv2
d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
img = cv2.aruco.generateImageMarker(d, 2, 400)
img = cv2.copyMakeBorder(img, 100, 100, 100, 100, cv2.BORDER_CONSTANT, value=255)
cv2.imwrite('/tmp/marker2.png', img)"
```

Publish it as the camera, run the node and watch the output (package and executable names are
suggestions):

```
ros2 run image_publisher image_publisher_node /tmp/marker2.png --ros-args -r image_raw:=/camera/image_raw
ros2 run isocare_marker_detector marker_detector
ros2 topic echo /markers
```

An image on a screen has no real size, so for a true distance check print a marker at exactly
`marker_size_m` and measure the distance with a tape.

## 9. Paste-in prompt for your AI

> You are helping me build the `marker_detector` node of a ROS 2 Jazzy project (IsoCare).
> I attached AI_CONTEXT.md (project and coding rules) and marker_detector.md (decisions and
> contract). Follow both. Do not write code yet: first repeat the contract back in your own words
> (what the node reads and publishes, what each output field means, what it must not do), list
> what you will build, and ask me one question. Then work one piece at a time, starting with the
> ROS-free marker_core and its tests.
