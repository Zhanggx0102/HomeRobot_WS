##############################################################
Author:   Shawn
Email:    zhanggx0102@163.com
Updata:     20180917
##############################################################
1. Run following command to simulate gmapping in gazebo

    $ roslaunch rover_navigation gazebo_gmapping_robot.launch 
    $ rosrun teleop_twist_keyboard teleop_twist_keyboard.py

    tips:
    <1> modify <dynamics damping="0.1" friction="0.5"/> in xacro file to get a better simulated effect.
    <2> <plugin name="skid_steer_drive_controller" filename="libgazebo_ros_skid_steer_drive.so"> parameters 
        <torque>1</torque> also influence the simulated effect.

2. Run following command to simulate navigation(AMCL) in gazebo

    $ roslaunch rover_navigation configuration_gazebo.launch
    $ roslaunch rover_navigation move_base.launch

