source catkin_ws_iliad/devel/setup.bash

roslaunch iliad sim.launch

rostopic pub /robotnik_base_control/cmd_vel geometry_msgs/Twist "linear:
  x: 0.1
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" -r 10
