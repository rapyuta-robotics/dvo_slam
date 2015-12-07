/** 
	This is file for Visual odometry using dvo_core libraries

*/

#ifndef VISUAL_ODOM_H_
#define VISUAL_ODOM_H_
#include <ros/ros.h>
#include <ros/console.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <image_transport/image_transport.h>
#include <dvo/util/stopwatch.h>
#include <dvo/core/surface_pyramid.h>
#include <dvo/dense_tracking.h>
#include <dvo/core/intrinsic_matrix.h>
#include <dvo/core/rgbd_image.h>
#include <dvo/visualization/camera_trajectory_visualizer.h>

using namespace dvo;

typedef message_filters::sync_policies::ApproximateTime<
                sensor_msgs::Image,
                sensor_msgs::Image,
                sensor_msgs::CameraInfo,
                sensor_msgs::CameraInfo
                > RGBDWithCameraInfoPolicy;
//typedef dynamic_reconfigure::Server<dvo_ros::CameraDenseTrackerConfig> ReconfigureServer;



class CameraBase
{
protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  message_filters::Subscriber<sensor_msgs::Image> rgb_image_subscriber_;
  message_filters::Subscriber<sensor_msgs::Image> depth_image_subscriber_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> rgb_camera_info_subscriber_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> depth_camera_info_subscriber_;

  message_filters::Synchronizer<RGBDWithCameraInfoPolicy> synchronizer_;

  bool isSynchronizedImageStreamRunning();

  void startSynchronizedImageStream();
  void stopSynchronizedImageStream();
public:
  CameraBase(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
  virtual ~CameraBase();

  virtual void handleImages(
      const sensor_msgs::Image::ConstPtr& rgb_image_msg,
      const sensor_msgs::Image::ConstPtr& depth_image_msg,
      const sensor_msgs::CameraInfo::ConstPtr& rgb_camera_info_msg,
      const sensor_msgs::CameraInfo::ConstPtr& depth_camera_info_msg
  ) = 0;
private:
  message_filters::Connection connection;
  bool connected;
};


namespace rapyuta{



class VisualOdometry : public CameraBase
{
public:
	/**
	Class for interfacing dvo core libraries with ros 

	*/
	
	//dvo::core::RgbdCameraPyramidPtr camera;
  	dvo::core::RgbdImagePyramidPtr current, reference;
  	ros::Publisher odom_pub_;
  	//ReconfigureServer reconfigure_server_;
  	Eigen::Affine3d accumulated_transform, from_baselink_to_asus, latest_absolute_transform_;
  	boost::mutex tracker_mutex_;
  	size_t frames_since_last_success;
  	dvo::visualization::CameraTrajectoryVisualizerInterface* vis_;
  	//boost::shared_ptr<dvo::DenseTracker> tracker;
  	//dvo::DenseTracker tracker;
  	dvo::DenseTracker::Config tracker_cfg;

  	//ros::Publisher odom_pub_; 
  	ros::Time current_time, last_time;
	/** @brief: Constructor to initialize parameters */
	VisualOdometry(ros::NodeHandle& nh,ros::NodeHandle& nh_private);
	virtual ~VisualOdometry();

	/** @brief: Callback for images and transform calculation*/
	void handleImages(
		const sensor_msgs::Image::ConstPtr& rgb_image_msg,
		const sensor_msgs::Image::ConstPtr& depth_image_msg,
      	const sensor_msgs::CameraInfo::ConstPtr& rgb_camera_info_msg,
      	const sensor_msgs::CameraInfo::ConstPtr& depth_camera_info_msg );
	
	/** @brief: callback for pose subscriber */
	void publishTransform(const std_msgs::Header& header, const Eigen::Affine3d& transform, const std::string frame);
	/** @brief: odometry publisher */
	void publishOdom(const std_msgs::Header& header, const Eigen::Affine3d& transform, const std::string frame);



};

}// namespace rapyuta

#endif /* VISUAL_ODOM_H_ */