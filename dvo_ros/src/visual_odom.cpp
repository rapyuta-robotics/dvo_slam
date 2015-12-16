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
#include <eigen_conversions/eigen_msg.h>
/**
	@brief: This class is ros wrapper for dvo_core library.
	It is used to calculate visual odometry from RGBD frames.

	Subscribers:
		- rgb-frame
		- depth-frame
		- rgb camera info
		- depth camera info

	Publishers:
		- odom
		- tf

*/


class VisualOdomNode{
public:
	ros::NodeHandle nh_;
	ros::Publisher odom_pub_;
	tf::TransformBroadcaster odom_broadcaster;
	ros::Time current_time, last_time;

	Eigen::Affine3d accumulated_transform_, from_baselink_to_asus_, latest_absolute_transform_;
	boost::mutex tracker_mutex_;
	//dvo stuff
	dvo::core::RgbdImagePyramidPtr current_, reference_;
	dvo::DenseTracker::Config cfg;

	//global variables
	cv::Mat intensity_, depth_,prev_intensity_, prev_depth_;
	dvo::core::IntrinsicMatrix intrinsics_;
	double camera_width_,camera_height_;
	bool first ;

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

	/** @brief:  Constructor containing initialization of several parameters */
	VisualOdomNode(ros::NodeHandle& nh);

	/** @brief: calulates Rigid body tranform between current and reference frame */
	void findTransform();

	/** @brief: synchronous rgbd frame callback */
	void RGBDcallback(const sensor_msgs::Image::ConstPtr& rgb_msg, const sensor_msgs::Image::ConstPtr& depth_msg, const sensor_msgs::CameraInfo::ConstPtr& rgb_info, const sensor_msgs::CameraInfo::ConstPtr& depth_info);

	/** @brief: order based loop */
	//void run();

	/** @brief: publish odometry information */
	void publishOdom();

	/** @brief: publish tf data between frames */
	void publishTransform();
};

VisualOdomNode::VisualOdomNode(ros::NodeHandle& nh):
	rgb_image_subscriber_(nh, "camera/rgb/image_color", 1),
  	depth_image_subscriber_(nh, "camera/depth/image", 1),
  	rgb_camera_info_subscriber_(nh, "camera/rgb/camera_info", 1),
  	depth_camera_info_subscriber_(nh, "camera/depth/camera_info", 1),
	sync(MySyncPolicy(5), rgb_image_subscriber_,depth_image_subscriber_,rgb_camera_info_subscriber_,depth_camera_info_subscriber_)
{
	cfg = dvo::DenseTracker::getDefaultConfig();													/* initialize configuration */
	//cfg.run_dense_tracking = true;
	std::cout<<cfg<<std::endl;
	connection =sync.registerCallback(boost::bind(&VisualOdomNode::RGBDcallback,this,_1,_2,_3,_4)); /* synchronous RGBD data callback */
	odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom",1);											/* Odom publisher */
	first = true; //flag for initialization
	accumulated_transform_.setIdentity();
	current_time = ros::Time::now();
  	last_time = ros::Time::now();
}

void VisualOdomNode::publishOdom( )
{
	ROS_INFO("odom publish");
	//it should be accumulated_transform
	nav_msgs::Odometry odom;
	static int seq = 1;
	odom.header.seq =seq++;
	odom.header.stamp = current_time;
	odom.header.frame_id = "odom";
	odom.child_frame_id = "base_link_estimate";
	tf::Transform tmp;
	tf::transformEigenToTF(accumulated_transform_, tmp);
	tf::poseTFToMsg(tmp,odom.pose.pose);
	odom_pub_.publish(odom);
	return;
}

void VisualOdomNode::publishTransform()
{
	ROS_INFO("Publishing transform");
	//it should be accumulated_transform
	geometry_msgs::TransformStamped odom_trans;

	odom_trans.header.stamp = current_time;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link_estimate";
	tf::transformEigenToMsg(accumulated_transform_,odom_trans.transform);
	//send the transform
	odom_broadcaster.sendTransform(odom_trans);
	return;
}

void VisualOdomNode::findTransform()
{

	// intialize dvo stuff
	dvo::core::RgbdCameraPyramid camera(camera_width_,camera_height_, intrinsics_);
	camera.build(cfg.getNumLevels());
	dvo::core::RgbdCameraPyramid prev_camera(camera_width_,camera_height_, intrinsics_);
	prev_camera.build(cfg.getNumLevels());
	cv::Size sz = intensity_.size();
	dvo::DenseTracker tracker(cfg);

	if(sz.width == 640 && sz.height == 480)
	{
		current_ = camera.create(intensity_,depth_);
		reference_ = prev_camera.create(prev_intensity_,prev_depth_);
		Eigen::Affine3d transform;
		bool success = tracker.match(*reference_, *current_, transform);
		std::cout<<success<<std::endl;

		if(success)
		{
			ROS_INFO("Finding transform");
			accumulated_transform_ = accumulated_transform_ * transform;
			std::cout<<transform.matrix()<<std::endl;
		}
	}
	return;

}


void VisualOdomNode::RGBDcallback(const sensor_msgs::Image::ConstPtr& rgb_msg, const sensor_msgs::Image::ConstPtr& depth_msg, const sensor_msgs::CameraInfo::ConstPtr& rgb_info, const sensor_msgs::CameraInfo::ConstPtr& depth_info)
{	//create dvo suitable variable from raw data
	ROS_INFO("RGBD callback");
	boost::mutex::scoped_lock lock(tracker_mutex_);

	current_time = ros::Time::now();

	/* check if the input is of the same size */
	if(depth_info->width != rgb_info->width || depth_info->height != rgb_info->height)
	{
		ROS_WARN("RGB and depth image have different size!");

		return;
	}

	/* convert corresponding images */
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
		std::cout<<depth_in.type()<<std::endl;
		depth_ = depth_in;
	}

	intrinsics_ = dvo::core::IntrinsicMatrix::create(517.3, 516.5, 318.6, 255.3);// for fr1/room data from TU
	camera_width_ = rgb_info->width;
	camera_height_ = rgb_info->height;

	//dvo stuff
	//dvo::core::RgbdCameraPyramid camera(camera_width_,camera_height_, intrinsics_);
	//camera.build(cfg.getNumLevels());
	//to check if image is available
	cv::Size sz = depth_.size();
	if(sz.width == camera_width_ && sz.height == camera_height_)
	{
		if(first)
		{
			//reference_ = camera.create(intensity_,depth_);
			//current_ = camera.create(intensity_,depth_);
			prev_intensity_=intensity_;
			prev_depth_ = depth_;
			std::cout<< "reference created"<<std::endl;
			//reference_.swap(current_);
			first = false;
			return;
		}

		//current_ = camera.create(intensity_,depth_);

		findTransform();
		publishTransform();
		publishOdom();
		ROS_INFO("swapping");
		prev_depth_= depth_;
		prev_intensity_ =intensity_;
	}


	return;
}
/*
void VisualOdomNode::run()
{
	dvo::core::RgbdCameraPyramid camera(camera_width_,camera_height_, intrinsics_);
	camera.build(cfg.getNumLevels());
	//to check if image is available
	cv::Size sz = depth_.size();
	if(sz.width == camera_width_ && sz.height == camera_height_)
	{
		if(first)
		{
			//reference_ = camera.create(intensity_,depth_);
			current_ = camera.create(intensity_,depth_);
			std::cout<< "reference created"<<std::endl;
			//reference_.swap(current_);
			first = false;
			return;
		}
		else
		{
			//reference_.swap(current_);
			reference_.swap(current_);

			current_ = camera.create(intensity_,depth_);
		}
		//current_ = camera.create(intensity_,depth_);

		//findTransform();
		publishTransform();
		publishOdom();
	}
}
*/
int main(int argc, char *argv[])
{

	ros::init(argc,argv,"Visual_odom");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	VisualOdomNode vo(nh);
	ROS_INFO("started vo...");
	//ros::spin();
	ros::Rate r(2.0);
	while(nh.ok())
	{

		ros::spinOnce();
		//vo.run();
		r.sleep();
	}
	return 0;
}
