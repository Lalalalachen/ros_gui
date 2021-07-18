# temporary_calib_interface

## Dependency
- jsk - Plugins for calling services (not necessary when using bag - files) 

      sudo apt install ros-noetic-jsk-visualization

- calibration_msgs

      git clone https://gitlab.com/matthias111/calibration_msgs

## Use with Bag-files
1. Clone *calibration_msgs* and *temporary_calib_interface* to catkin - workspace
2. Build workspace (caktin_make)
3. Source workspace (source devel/setup.bash)
4. Launch interface with roslaunch (roslaunch temporary_calib_interface calib_vis_bag.launch)
5. Play recorded Bag-file (rosbag play bag_file.bag -l) using -l for looping bag file

For better performance decompress bag file:
- check if bag file is compressed (rosbag info bag_file.bag)
- decompress bag file (rosbag decompress bag_file.bag)
