/**
* This code provide Visual odometry from RGBD frames using dvo libraries.
*
*/

#include <dvo_ros/visual_odom.h>

using namespace dvo;





CameraBase::CameraBase(ros::NodeHandle& nh, ros::NodeHandle& nh_private) :
  nh_(nh),
  nh_private_(nh_private),

  rgb_image_subscriber_(nh, "camera/rgb/image_raw", 1),
  depth_image_subscriber_(nh, "camera/depth_registered/image_raw", 1),
  rgb_camera_info_subscriber_(nh, "camera/rgb/camera_info", 1),
  depth_camera_info_subscriber_(nh, "camera/depth_registered/camera_info", 1),

  synchronizer_(RGBDWithCameraInfoPolicy(5), rgb_image_subscriber_, depth_image_subscriber_, rgb_camera_info_subscriber_, depth_camera_info_subscriber_),

  connected(false)
{
}

CameraBase::~CameraBase()
{
  stopSynchronizedImageStream();
}

bool CameraBase::isSynchronizedImageStreamRunning()
{
  return connected;
}

void CameraBase::startSynchronizedImageStream()
{
  if(!connected)
  {
    connection = synchronizer_.registerCallback(boost::bind(&CameraBase::handleImages, this, _1, _2, _3, _4));
    connected = true;
  }
}

void CameraBase::stopSynchronizedImageStream()
{
  if(connected)
  {
    connection.disconnect();
    connected = false;
  }
}

namespace rapyuta 
{


VisualOdometry::VisualOdometry(ros::NodeHandle& nh, ros::NodeHandle& nh_private) :
	CameraBase(nh,nh_private)
{
	ROS_INFO("In VO node");
	odom_pub_= nh.advertise<nav_msgs::Odometry>("odom", 1);
	current_time = ros::Time::now();
  	last_time = ros::Time::now();
	startSynchronizedImageStream();
	vis_= new dvo::visualization::NoopCameraTrajectoryVisualizer();
	from_baselink_to_asus.setIdentity();
	latest_absolute_transform_.setIdentity();
  	accumulated_transform.setIdentity();
}

VisualOdometry::~VisualOdometry()
{

}

void VisualOdometry::handleImages(
	const sensor_msgs::Image::ConstPtr& rgb_image_msg,
	const sensor_msgs::Image::ConstPtr& depth_image_msg,
    const sensor_msgs::CameraInfo::ConstPtr& rgb_camera_info_msg,
    const sensor_msgs::CameraInfo::ConstPtr& depth_camera_info_msg )
{
	ROS_INFO("handleImages");
	boost::mutex::scoped_lock lock(tracker_mutex_); // for multiple threads
	if(depth_camera_info_msg->width != rgb_camera_info_msg->width || depth_camera_info_msg->height != rgb_camera_info_msg->height)
	{
		ROS_WARN("RGB and depth image have different size!");

		return;
	}
	current_time = ros::Time::now();
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
		dvo::core::SurfacePyramid::convertRawDepthImageSse(depth_in, depth, 0.001);
	}
	else
	{
		depth = depth_in;
	}

	dvo::core::IntrinsicMatrix intrinsics = dvo::core::IntrinsicMatrix::create(rgb_camera_info_msg->P[0], rgb_camera_info_msg->P[5], rgb_camera_info_msg->P[2], rgb_camera_info_msg->P[6]);
	//camera->build(1);
	//camera = new dvo::core::RgbdCameraPyramidPtr(rgb_camera_info_msg->width,rgb_camera_info_msg->height,intrinsics));
	dvo::core::RgbdCameraPyramid camera(rgb_camera_info_msg->width,rgb_camera_info_msg->height,intrinsics);
	dvo::DenseTracker::Config cfg = dvo::DenseTracker::getDefaultConfig();
	camera.build(cfg.getNumLevels());
	dvo::DenseTracker tracker;
	static Eigen::Affine3d first;

	if(!reference)
	{
		ROS_INFO("initialize transforms");
		accumulated_transform = latest_absolute_transform_ * from_baselink_to_asus;
		first = accumulated_transform;
		current = camera.create(intensity, depth);
		reference= camera.create(intensity, depth);
		tracker.configure(cfg);
		last_time = ros::Time::now();
		return;
	}

	
	current = camera.create(intensity, depth);

	// time delay compensation TODO: use driver settings instead
	std_msgs::Header h = rgb_image_msg->header;
	//h.stamp -= ros::Duration(0.05);

	Eigen::Affine3d transform;

	ROS_INFO("Starting tracker match");
	bool success = tracker.match(*reference, *current, transform);
	ROS_INFO_STREAM("Tracker matching is :"<< success);
	//sw_match.stopAndPrint();

	if(success)
	{
		//frames_since_last_success = 0;
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
		//frames_since_last_success++;
		reference.swap(current);
		ROS_WARN("fail");
	}


	publishTransform(h, accumulated_transform * from_baselink_to_asus.inverse(), "base_link_estimate");
	//  publishTransform(rgb_image_msg->header, first_transform.inverse() * accumulated_transform, "asus_estimate");
	publishOdom(h,transform,"base_link_estimate");
	reference.swap(current);
}

void VisualOdometry::publishTransform(const std_msgs::Header& header, const Eigen::Affine3d& transform, const std::string frame)
{
	/** 
	Publishes both transform  
	*/
	ROS_INFO("Publishing transform and Odometry");

	static tf::TransformBroadcaster tb;
	tf::StampedTransform tf_transform;
	tf_transform.frame_id_ = "world";
	tf_transform.child_frame_id_ = frame;
	tf_transform.stamp_ = header.stamp;

	tf::transformEigenToTF(transform, tf_transform);

	tb.sendTransform(tf_transform);
}

void VisualOdometry::publishOdom(const std_msgs::Header& header, const Eigen::Affine3d& transform, const std::string frame)
{
	nav_msgs::Odometry odom_msg;
	odom_msg.header.stamp = header.stamp;
	odom_msg.header.frame_id = "world";
	odom_msg.child_frame_id = frame;

	tf::Transform delta_tf;
	tf::transformEigenToTF(transform, delta_tf);
	tf::poseTFToMsg(delta_tf,odom_msg.pose.pose);
	if(!last_time.isZero())
	{
		double delta_t = (current_time - last_time).toSec();
		if(delta_t)
		{
			odom_msg.twist.twist.linear.x = delta_tf.getOrigin().getX() / delta_t;
			odom_msg.twist.twist.linear.y = delta_tf.getOrigin().getY() / delta_t;
			odom_msg.twist.twist.linear.z = delta_tf.getOrigin().getZ() / delta_t;
			tf::Quaternion delta_rot = delta_tf.getRotation();
			tfScalar angle = delta_rot.getAngle();
			tf::Vector3 axis = delta_rot.getAxis();
			tf::Vector3 angular_twist = axis * angle / delta_t;
			odom_msg.twist.twist.angular.x = angular_twist.x();
			odom_msg.twist.twist.angular.y = angular_twist.y();
			odom_msg.twist.twist.angular.z = angular_twist.z();
		}		
	}

	odom_msg.pose.covariance.assign(0.0);
	odom_msg.twist.covariance.assign(0.0);

	odom_pub_.publish(odom_msg);
	last_time = current_time;


}



}// namespace rapyuta
int main(int argc, char *argv[])
{
	/* code */
	ros::init(argc,argv,"Visual odom");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	rapyuta::VisualOdometry vo(nh,nh_private);
	ROS_INFO("started vo...");
	ros::spin(); 
	return 0;
}