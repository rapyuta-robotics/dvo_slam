/**
 *  This file is part of dvo.
 *
 *  Copyright 2012 Christian Kerl <christian.kerl@in.tum.de> (Technical University of Munich)
 *  For more information see <http://vision.in.tum.de/data/software/dvo>.
 *
 *  dvo is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  dvo is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with dvo.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <boost/bind.hpp>
#include <cv_bridge/cv_bridge.h>

#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <dvo/core/surface_pyramid.h>
#include <dvo/util/stopwatch.h>
//#include <dvo/visualization/visualizer.h> //why commented?
//#include <dvo/visualization/pcl_camera_trajectory_visualizer.h> //why commented ?

#include <dvo_ros/camera_dense_tracking.h>
#include <dvo_ros/util/util.h>
#include <dvo_ros/util/configtools.h>
#include <dvo_ros/visualization/ros_camera_trajectory_visualizer.h>


namespace dvo_ros
{

using namespace dvo;
using namespace dvo::core;
using namespace dvo::util;

CameraDenseTracker::CameraDenseTracker(ros::NodeHandle& nh, ros::NodeHandle& nh_private) :
  CameraBase(nh, nh_private),

  tracker_cfg(DenseTracker::getDefaultConfig()),
  frames_since_last_success(0),
  reconfigure_server_(nh_private),
  vis_(new dvo_ros::visualization::RosCameraTrajectoryVisualizer(nh)),
  use_dense_tracking_estimate_(false)
{
  ROS_INFO("1: CameraDenseTracker::Constructor(...)");

  ROS_INFO("2: Publishing Pose");
  pose_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("rgbd/pose", 1);
  odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom",1);
  ROS_INFO("3: Reconfigure server callback");
  std::cout<<tracker<<std::endl;
  ReconfigureServer::CallbackType reconfigure_server_callback = boost::bind(&CameraDenseTracker::handleConfig, this, _1, _2);
  reconfigure_server_.setCallback(reconfigure_server_callback);

  ROS_INFO("4: Try getting transform");
  dvo_ros::util::tryGetTransform(from_baselink_to_asus, tl, "base_link", "asus");

  ROS_INFO_STREAM("5: transformation: base_link -> asus" << std::endl << from_baselink_to_asus.matrix());

  ROS_INFO("6: Subscribe to pose");
  pose_sub_ = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("pelican/pose", 1, &CameraDenseTracker::handlePose, this);

  latest_absolute_transform_.setIdentity();
  accumulated_transform.setIdentity();

  //dvo::visualization::Visualizer::instance()
  //  .enabled(false)
  //  .useExternalWaitKey(false)
  //  .save(false)
  //;
}


CameraDenseTracker::~CameraDenseTracker()
{
  delete vis_;
}

bool CameraDenseTracker::hasChanged(const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg)
{
  return width != camera_info_msg->width || height != camera_info_msg->height;
}

void CameraDenseTracker::reset(const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg)
{
  ROS_INFO("Resetting camera dense tracker");
  //IntrinsicMatrix intrinsics = IntrinsicMatrix::create(camera_info_msg->K[0], camera_info_msg->K[4], camera_info_msg->K[2], camera_info_msg->K[5]);
  IntrinsicMatrix intrinsics = IntrinsicMatrix::create(camera_info_msg->P[0], camera_info_msg->P[5], camera_info_msg->P[2], camera_info_msg->P[6]);

  camera.reset(new dvo::core::RgbdCameraPyramid(camera_info_msg->width, camera_info_msg->height, intrinsics));
  camera->build(tracker_cfg.getNumLevels());

  tracker.reset(new DenseTracker(tracker_cfg));

  static RgbdImagePyramid* const __null__ = 0;

  reference.reset(__null__);
  current.reset(__null__);

  width = camera_info_msg->width;
  height = camera_info_msg->height;

  vis_->reset();
}

void CameraDenseTracker::handleConfig(dvo_ros::CameraDenseTrackerConfig& config, uint32_t level)
{
  ROS_INFO("Handling Config");
   ROS_INFO_STREAM("reconfigured tracker, config ( " << tracker_cfg << " )");
  if(level == 0) return;

  if(level & CameraDenseTracker_RunDenseTracking)
  {
    if(config.run_dense_tracking)
    {
      startSynchronizedImageStream();
    }
    else
    {
      stopSynchronizedImageStream();

      // force reset of tracker
      width = 0;
      height = 0;
    }
  }

  if(!config.run_dense_tracking && config.use_dense_tracking_estimate)
  {
    config.use_dense_tracking_estimate = false;
  }

  use_dense_tracking_estimate_ = config.use_dense_tracking_estimate;

  if(level & CameraDenseTracker_ConfigParam)
  {
    // fix config, so we don't die by accident
    if(config.coarsest_level < config.finest_level)
    {
      config.finest_level = config.coarsest_level;
    }

    dvo_ros::util::updateConfigFromDynamicReconfigure(config, tracker_cfg);

    // we are called in the ctor as well, but at this point we don't have a tracker instance
    if(tracker)
    {
      // lock tracker so we don't reconfigure it while it is running
      boost::mutex::scoped_lock lock(tracker_mutex_);

      tracker->configure(tracker_cfg);
      camera->build(tracker_cfg.getNumLevels());
    }

    ROS_INFO_STREAM("reconfigured tracker, config ( " << tracker_cfg << " )");
  }


  if(level & CameraDenseTracker_MiscParam)
  {
    vis_->reset();
    delete vis_;
    ROS_INFO("Visualizer reset");

    if(config.reconstruction)
    {
      //vis_ = new dvo::visualization::PclCameraTrajectoryVisualizer();
      vis_ = new dvo_ros::visualization::RosCameraTrajectoryVisualizer(nh_);
    }
    else
    {
      vis_ = new dvo::visualization::NoopCameraTrajectoryVisualizer();
    }
  }
}

void CameraDenseTracker::handlePose(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose)
{
  ROS_INFO("Handling pose");
  tf::Transform tmp;

  tf::poseMsgToTF(pose->pose.pose, tmp);
  tf::transformTFToEigen(tmp, latest_absolute_transform_);

  if(!use_dense_tracking_estimate_)
    publishPose(pose->header, latest_absolute_transform_, "base_link_estimate");
}

// change here for resolution of images
void CameraDenseTracker::handleImages(
    const sensor_msgs::Image::ConstPtr& rgb_image_msg,
    const sensor_msgs::Image::ConstPtr& depth_image_msg,
    const sensor_msgs::CameraInfo::ConstPtr& rgb_camera_info_msg,
    const sensor_msgs::CameraInfo::ConstPtr& depth_camera_info_msg
)
{

  ROS_INFO("Handling Images");
  static stopwatch sw_callback("callback");
  sw_callback.start();

  // lock tracker so no one can reconfigure it
  boost::mutex::scoped_lock lock(tracker_mutex_);

  // different size of rgb and depth image
  if(depth_camera_info_msg->width != rgb_camera_info_msg->width || depth_camera_info_msg->height != rgb_camera_info_msg->height)
  {
    ROS_WARN("RGB and depth image have different size!");

    return;
  }

  // something has changed
  if(hasChanged(rgb_camera_info_msg))
  {
    ROS_WARN("RGB image size has changed, resetting tracker!");

    reset(rgb_camera_info_msg);
  }

  cv::Mat intensity, depth;
  cv::Mat rgb_in = cv_bridge::toCvShare(rgb_image_msg)->image;

  if(rgb_in.channels() == 3)
  {
    cv::Mat tmp;
    cv::cvtColor(rgb_in, tmp, CV_BGR2GRAY, 1);

    tmp.convertTo(intensity, CV_32F);
  }
  else
  {
    rgb_in.convertTo(intensity, CV_32F);
  }

  cv::Mat depth_in = cv_bridge::toCvShare(depth_image_msg)->image;

  if(depth_in.type() == CV_16UC1)
  {
    SurfacePyramid::convertRawDepthImageSse(depth_in, depth, 0.001);
  }
  else
  {
    depth = depth_in;
  }

  /*
  //reducing the resolution of rgb and depth images
  cv::pyrDown( intensity, intensity, cv::Size( intensity.cols/2, intensity.rows/2) );
  cv::pyrDown( depth, depth, cv::Size( depth.cols/2, depth.rows/2) );
  */

  reference.swap(current);
  current = camera->create(intensity, depth);

  // time delay compensation TODO: use driver settings instead
  std_msgs::Header h = rgb_image_msg->header;
  //h.stamp -= ros::Duration(0.05);

  static Eigen::Affine3d first;

  if(!reference)
  {
    accumulated_transform = latest_absolute_transform_ * from_baselink_to_asus;
    first = accumulated_transform;
    ROS_INFO("Visualizing transformed");
    /*
    vis_->camera("first")->
        color(dvo::visualization::Color::blue()).
        update(current->level(0), latest_absolute_transform_).
        show();
    */
    return;
  }

  Eigen::Affine3d transform;

  static stopwatch sw_match("match", 100);
  sw_match.start();

  ROS_INFO("Starting tracker match");
  bool success = tracker->match(*reference, *current, transform);
  ROS_INFO_STREAM("Tracker matching is :"<< success);
  sw_match.stopAndPrint();

  if(success)
  {
    frames_since_last_success = 0;
    accumulated_transform = accumulated_transform * transform;

    Eigen::Matrix<double, 6, 6> covariance; // Is this used?

    //tracker->getCovarianceEstimate(covariance);

    //std::cerr << covariance << std::endl << std::endl;
    ROS_INFO("Visualizing trajectory");
    vis_->trajectory("estimate")->
        color(dvo::visualization::Color::blue())
        .add(accumulated_transform);
    ROS_INFO("Visualizing current frame");

    vis_->camera("current")->
        color(dvo::visualization::Color::red()).
        update(current->level(0), accumulated_transform).
        show();

  }
  else
  {
    frames_since_last_success++;
    reference.swap(current);
    ROS_WARN("fail");
  }


  publishTransform(h, accumulated_transform * from_baselink_to_asus.inverse(), "base_link_estimate");
//  publishTransform(rgb_image_msg->header, first_transform.inverse() * accumulated_transform, "asus_estimate");

  if(use_dense_tracking_estimate_)
  {
    publishPose(h, accumulated_transform * from_baselink_to_asus.inverse(), "base_link_estimate");
  }

  sw_callback.stopAndPrint();
}

void CameraDenseTracker::publishTransform(const std_msgs::Header& header, const Eigen::Affine3d& transform, const std::string frame)
{
  ROS_INFO("Publishing transform");
  static tf::TransformBroadcaster tb;

  tf::StampedTransform tf_transform;
  tf_transform.frame_id_ = "world";
  tf_transform.child_frame_id_ = frame;
  tf_transform.stamp_ = header.stamp;

  tf::transformEigenToTF(transform, tf_transform);

  tb.sendTransform(tf_transform);
}

void CameraDenseTracker::publishOdom(const std_msgs::Header& header, const Eigen::Affine3d& transform, const std::string frame)
{
  ROS_INFO("Publishing odom");
  if(odom_pub_.getNumSubscribers() == 0) return;




}
/* @brief: publishes both pose with covariance stamped and odom pose */
void CameraDenseTracker::publishPose(const std_msgs::Header& header, const Eigen::Affine3d& transform, const std::string frame)
{
  /* @todo : Add twist also */
  ROS_INFO("Publishing Pose");
  //if(pose_pub_.getNumSubscribers() == 0) return;

  geometry_msgs::PoseWithCovarianceStampedPtr msg(new geometry_msgs::PoseWithCovarianceStamped);

  nav_msgs::Odometry odom;
  static int seq = 1;

  msg->header.seq = seq++;
  msg->header.frame_id = frame;
  msg->header.stamp = header.stamp;

  odom.header.stamp = header.stamp;
  odom.header.frame_id = "world";
  odom.child_frame_id = "base_link_estimate" ;
  tf::Transform tmp;

  tf::transformEigenToTF(transform, tmp);
  tf::poseTFToMsg(tmp, msg->pose.pose);
  tf::poseTFToMsg(tmp,odom.pose.pose);

  msg->pose.covariance.assign(0.0);

  odom_pub_.publish(odom);
  pose_pub_.publish(msg);
}

} /* namespace dvo_ros */
