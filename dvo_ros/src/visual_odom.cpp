#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_listener.h>
#include <dvo/dense_tracking.h>
#include <dvo/core/rgbd_image.h>
#include <dvo/core/surface_pyramid.h>
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
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <dvo_ros/CameraDenseTrackerConfig.h>
#include <dvo_ros/util/util.h>
#include <dvo_ros/util/configtools.h>

using namespace dvo;
using namespace dvo_ros;
using namespace dvo::core;
class VisualOdomNode{
public:
	ros::NodeHandle nh_;
	dvo::core::RgbdImagePyramidPtr current_, reference_;
	Eigen::Affine3d accumulated_transform_, from_baselink_to_asus_, latest_absolute_transform_;

	//dynamic reconfigure
	typedef dynamic_reconfigure::Server<dvo_ros::CameraDenseTrackerConfig> ReconfigureServer;
	ReconfigureServer reconfigure_server_;
	bool use_dense_tracking_estimate_;
  	dvo::DenseTracker::Config tracker_cfg;

	//dvo stuff
	boost::shared_ptr<dvo::DenseTracker> tracker;
	boost::mutex tracker_mutex_;
	//global variables
	cv::Mat intensity_, depth_;
	dvo::core::IntrinsicMatrix intrinsics_;
	double camera_width_,camera_height_;
	bool first_,connected;
	// message filter for sync input
	message_filters::Subscriber<sensor_msgs::Image> rgb_image_subscriber_;
	message_filters::Subscriber<sensor_msgs::Image> depth_image_subscriber_;
  	message_filters::Subscriber<sensor_msgs::CameraInfo> rgb_camera_info_subscriber_;
  	message_filters::Subscriber<sensor_msgs::CameraInfo> depth_camera_info_subscriber_;

  	typedef message_filters::sync_policies::ApproximateTime<
                sensor_msgs::Image,
                sensor_msgs::Image,
                sensor_msgs::CameraInfo,
                sensor_msgs::CameraInfo
                > MySyncPolicy;

    message_filters::Synchronizer<MySyncPolicy> sync;
    message_filters::Connection connection;
	VisualOdomNode(ros::NodeHandle& nh);
	void startStream();
	void findTransform();
	void handleConfig(dvo_ros::CameraDenseTrackerConfig& config, uint32_t level);
	//void run();
	void RGBDcallback(const sensor_msgs::Image::ConstPtr& rgb_msg, const sensor_msgs::Image::ConstPtr& depth_msg, const sensor_msgs::CameraInfo::ConstPtr& rgb_info, const sensor_msgs::CameraInfo::ConstPtr& depth_info);
};

VisualOdomNode::VisualOdomNode(ros::NodeHandle& nh):
	rgb_image_subscriber_(nh, "camera/rgb/image_raw", 1),
  	depth_image_subscriber_(nh, "camera/depth_registered/image_raw", 1),
  	rgb_camera_info_subscriber_(nh, "camera/rgb/camera_info", 1),
  	depth_camera_info_subscriber_(nh, "camera/depth_registered/camera_info", 1),
	sync(MySyncPolicy(5), rgb_image_subscriber_,depth_image_subscriber_,rgb_camera_info_subscriber_,depth_camera_info_subscriber_),
	connected(false),
	first_(true),
	reconfigure_server_(nh),
	use_dense_tracking_estimate_(false),
	tracker_cfg(dvo::DenseTracker::getDefaultConfig())
{


	ROS_INFO("starting vo node");
	boost::shared_ptr<dvo::DenseTracker> tracker();
	ReconfigureServer::CallbackType reconfigure_server_callback = boost::bind(&VisualOdomNode::handleConfig, this, _1, _2);
  	reconfigure_server_.setCallback(reconfigure_server_callback);
	startStream();
}


void VisualOdomNode::handleConfig(dvo_ros::CameraDenseTrackerConfig& config, uint32_t level)
{
	ROS_INFO("Handling Config");

	if(level == 0) return;

	if(level & CameraDenseTracker_RunDenseTracking)
	{
	if(config.run_dense_tracking)
	{
	  startStream();
	}
	else
	{

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
		config.use_dense_tracking_estimate= true;
		config.run_dense_tracking = true;
		dvo_ros::util::updateConfigFromDynamicReconfigure(config, tracker_cfg);

		// we are called in the ctor as well, but at this point we don't have a tracker instance
		if(tracker)
		{
		  	// lock tracker so we don't reconfigure it while it is running
		  	boost::mutex::scoped_lock lock(tracker_mutex_);
		  	tracker->configure(tracker_cfg);

		}

		ROS_INFO_STREAM("reconfigured tracker, config ( " << tracker_cfg << " )");
	}


}

void VisualOdomNode::startStream()
{
	if(!connected)
  	{
    	connection = sync.registerCallback(boost::bind(&VisualOdomNode::RGBDcallback,this,_1,_2,_3,_4));
    	connected = true;
  	}
}

/*
void VisualOdomNode::run()
{
	message_filters::Connection connection;
	connection = sync.registerCallback(boost::bind(&VisualOdomNode::RGBDcallback,this,_1,_2,_3,_4));
	connection.disconnect();
	//findTransform();

}
*/


void VisualOdomNode::findTransform()
{
	ROS_INFO("findTransform");
	// intialize dvo stuff
	dvo::core::RgbdCameraPyramid camera(camera_width_,camera_height_, intrinsics_);
	//dvo::DenseTracker::Config cfg = dvo::DenseTracker::getDefaultConfig();
	camera.build(tracker_cfg.getNumLevels());



	//before creating new swap with previous
	reference_.swap(current_);
	current_ = camera.create(intensity_,depth_);
	Eigen::Affine3d transform;
	transform.setIdentity();
	//std::cout<< reference_<<std::endl;
	//std::cout<< current_<< std::endl;
	//std::cout << transform <<std::endl;
	bool success = tracker->match(*reference_, *current_, transform);
	if(success)
	{
		//frames_since_last_success = 0;
		accumulated_transform_ = accumulated_transform_ * transform;
	}
	else
	{
		reference_.swap(current_);
	}
	//std::cout<<"Relative transform is :" << transform <<std::endl;
	//std::cout<<"accumulated_transform is :"<< accumulated_transform_ <<std::endl;
	return;
}


void VisualOdomNode::RGBDcallback(const sensor_msgs::Image::ConstPtr& rgb_msg, const sensor_msgs::Image::ConstPtr& depth_msg, const sensor_msgs::CameraInfo::ConstPtr& rgb_info, const sensor_msgs::CameraInfo::ConstPtr& depth_info)
{	//create dvo suitable variable from raw data
	ROS_INFO("RGBDcallback");
	boost::mutex::scoped_lock lock(tracker_mutex_);
	if(depth_info->width != rgb_info->width || depth_info->height != rgb_info->height)
	{
		ROS_WARN("RGB and depth image have different size!");

		return;
	}

	cv::Mat rgb_in = cv_bridge::toCvShare(rgb_msg)->image;

	if(rgb_in.channels() == 3)
	{
		cv::Mat tmp;
		cv::cvtColor(rgb_in, tmp, CV_BGR2GRAY, 1);
		tmp.convertTo(intensity_, CV_32F);
	}
	else
	{
		rgb_in.convertTo(intensity_, CV_32F);
	}

	cv::Mat depth_in = cv_bridge::toCvShare(depth_msg)->image;

	if(depth_in.type() == CV_16UC1)
	{
		dvo::core::SurfacePyramid::convertRawDepthImageSse(depth_in, depth_, 0.001);
	}
	else
	{
		depth_ = depth_in;
	}

	intrinsics_ = dvo::core::IntrinsicMatrix::create(rgb_info->P[0], rgb_info->P[5], rgb_info->P[2], rgb_info->P[6]);
	camera_width_ = rgb_info->width;
	camera_height_ = rgb_info->height;

	if(first_)
	{
		dvo::core::RgbdCameraPyramid camera(camera_width_,camera_height_, intrinsics_);
		dvo::DenseTracker::Config cfg = dvo::DenseTracker::getDefaultConfig();
		camera.build(cfg.getNumLevels());
		current_ = camera.create(intensity_,depth_);
		accumulated_transform_.setIdentity();
		first_ = false;
		return;
	}
	else
	{
		findTransform();
	}
}

int main(int argc, char *argv[])
{

	ros::init(argc,argv,"Visual odom");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	VisualOdomNode vo(nh);
	ROS_INFO("started vo...");
	// while(ros::ok())
	// {
	// 	vo.run();

	// }
	ros::spin();

	return 0;
}
