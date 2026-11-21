# One ArUco marker seen in one camera image, expressed relative to the ROBOT (not the camera).
# Published by marker_detector on /markers, one message per detected marker per frame.
# Robot frame: x forward, y left, angles counter-clockwise positive (ROS REP 103).
# The navigator turns a sighting into a robot pose using the marker table (markers.yaml).
std_msgs/Header header        # stamp = the camera image's stamp (the delay matters while driving)
uint32 marker_id              # the ArUco id printed on the marker
float64 distance_m            # flat-floor distance from the robot centre to the marker centre
float64 bearing_rad           # direction of the marker from the robot's forward axis; left = positive
float64 facing_rel_rad        # direction the marker's FRONT faces, from the robot's forward axis,
                              # wrapped to [-pi, pi]; robot looking at the marker's front = +/- pi
float32 side_px               # mean side length of the marker in the image, in pixels
