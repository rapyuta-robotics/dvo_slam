# Dense Visual Odometry and SLAM (dvo_slam)

*NOTE: this is an alpha release APIs and parameters are going to change in near future. No support is provided at this point.*

These packages provide an implementation of the rigid body motion estimation of an RGB-D camera from consecutive images.

 *  **dvo_core**

    Core implementation of the motion estimation algorithm.

 *  **dvo_ros**

    Integration of *dvo_core* with ROS.

 *  **dvo_slam**

    Pose graph SLAM system based on *dvo_core* and integration with ROS.


## Installation

 *  Dependency(updating the list)
    * libg2o
    * vtk
    * tbb
    * libsuitesparse-dev

 *  ROS Indigo:

 *  Installing g2o
  1. git clone -b devel https://github.com/rapyuta/g2o.git
  2. cd git
  3. mkdir build && cd build
  4. cmake ..
  (make sure it is building with csparse that was installed previously)
  5. make
  6. sudo make install

 *  Compiling dvo
  1. git clone -b asus-indigo https://github.com/rapyuta/dvo_slam.git
  2. link it to your catkin_ws/src
  3. cd catkin_ws
  4. catkin_make




## Usage
Connect asus.
Estimating the camera trajectory from an RGB-D image stream:

```bash
roslaunch dvo_slam quickstart.launch
```
This will launch openni2, an rviz window and dynamic reconfiguring window.
In reconfiguring window,
       - in camera/driver, check depth_registration, check color_depth_synchronization.
       - in camera_keyframe_tracker/slam, check graph_op_robust, check graph_opt_final.
       - in camera_keyframe_tracker/tracking, uncheck use_initial_estimate, check run_dense_tracking, check use_dense_tracking_estimate, check use_weighting.

This will show point cloud on the rviz window. Move asus slowly to update the position of new pointclouds.   


## Publications

The following publications describe the approach:

 *   **Dense Visual SLAM for RGB-D Cameras** (C. Kerl, J. Sturm, D. Cremers), In Proc. of the Int. Conf. on Intelligent Robot Systems (IROS), 2013.
 *   **Robust Odometry Estimation for RGB-D Cameras** (C. Kerl, J. Sturm, D. Cremers), In Proc. of the IEEE Int. Conf. on Robotics and Automation (ICRA), 2013
 *   **Real-Time Visual Odometry from Dense RGB-D Images** (F. Steinbruecker, J. Sturm, D. Cremers), In Workshop on Live Dense Reconstruction with Moving Cameras at the Intl. Conf. on Computer Vision (ICCV), 2011.

## License

The packages *dvo_core*, *dvo_ros*, *dvo_slam*, and *dvo_benchmark* are licensed under the GNU General Public License Version 3 (GPLv3), see http://www.gnu.org/licenses/gpl.html.

The package *sophus* is licensed under the MIT License, see http://opensource.org/licenses/MIT.
