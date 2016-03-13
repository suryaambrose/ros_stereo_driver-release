Dependencies
------------

* gstreamer-app-0.10: `sudo apt-get install libgstreamer-plugins-base0.10-dev`
* ROS: `sudo apt-get install ros-{indigo,jade}-desktop-full`

Setup
-----

1.  Install dependencies
2. Source your ROS distribution; e.g. `source /opt/ros/jade/setup.bash`
3. If you don't have a catkin workspace, create one; e.g. `WS=stereo-workspace && mkdir -p $WS/src && cd $WS/src && catkin_init_workspace`
3. `cd` into the `src` folder of your catkin workspace
4. Clone sources:
  * `git clone https://github.com/ros-drivers/gscam.git`
  * `git clone https://github.com/suryaambrose/ros_stereo_driver.git`
5. Go to the workspace root and build all packages: `cd .. && catkin_make`

Getting started
---------------

1. Source the catkin workspace where stereo_driver is in any terminal in which it will be used: `source devel/setup.bash`
2. Create a launch file suited to your device (see sample launch file in launch/MyCam.launch)
3. Start capturing on the desired model of camera on the appropriate /dev/videoX: `roslaunch stereo_driver MyCam.launch DEVICE:=/dev/video0` in a separate terminal
  * _Note:_ if you have other connected cameras, you may have to adjust the `/dev/videoX` device
  * _Note:_ if you have different cameras, use several launch files accordingly

At this point stereo camera data should be published on appropriate topics; `rostopic list` to explore what's published.

If you need/want to use specific calibration files for your camera, you can replace camera_info/(left|right).yaml files with your owns.

Visualizing steams
------------------

To visualize the separated images:
* `rosrun image_view image_view image:=/stereo/left/image_raw`
* `rosrun image_view image_view image:=/stereo/right/image_raw`

To visualize the original image
* `rosrun image_view image_view image:=/camera/image_raw`

