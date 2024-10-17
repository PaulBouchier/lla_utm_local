This is a ros2 package for converting latitude, longitude and altitude coordinates
to UTM coordinates, and visa versa.

The node subscribes to NavSatFix messages and extracts the coordinates and republishes 
them as UTM coordinates (with an optional parameterized offset applied) in a PoseStamped
message in the 'map' frame. It also publishes tf2 transforms between the 'map' frame and
the NavSatFix position.

# Requirement
GeographicLib (test on version 1.34)
https://geographiclib.sourceforge.io/

# Use
open a terminal and run roslaunch
```
ros2 launch lla_utm_local lla_utm-launch.py zero_e:=-594929.9431329881 zero_n:=-4139043.529676078 utm_zone:=12
```
if you want to convert LLA to UTM, publish your message to topic `/gps/fix`, ros message type `sensor_msgs::NavSatFix`. If you want to convert UTM to LLA, publish your message to topic `/utm/pose`, ros message type `geometry_msgs::PoseStamped`

# Credits
This package is a port of https://github.com/arpg/ROS-UTM-LLA to ROS2