/* I dont like the way i implemented this program. I wish i could use a sigle function 'transferFromKinectToOdom' and somehow pass a geometry_msgs PointStamped as the other
 * argument to this function. There must be some way arond to make this happen. However for the time being i'll leave it as it is.
 *
 */

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

geometry_msgs::PointStamped centroid_kinectframe_rung1;
geometry_msgs::PointStamped centroid_kinectframe_rung2;
ros::Publisher centroid_odomcombined_rung1_pub;
ros::Publisher centroid_odomcombined_rung2_pub;

void Find_Centroid_Kinect_Frame1(const geometry_msgs::Point msg)
{
	centroid_kinectframe_rung1.point.x = msg.x;
	centroid_kinectframe_rung1.point.y = msg.y;
	centroid_kinectframe_rung1.point.z = msg.z;
	//ROS_INFO("the point in the kinectframe is heha %f %f %f \n", centroid_kinectframe_rung1.point.x, centroid_kinectframe_rung1.point.y, centroid_kinectframe_rung1.point.z);	
}

void Find_Centroid_Kinect_Frame2(const geometry_msgs::Point msg)
{
	centroid_kinectframe_rung2.point.x = msg.x;
	centroid_kinectframe_rung2.point.y = msg.y;
	centroid_kinectframe_rung2.point.z = msg.z;
}

void transferFromKinectToOdom1(const tf::TransformListener& listener)
{
	centroid_kinectframe_rung1.header.frame_id = "head_mount_kinect_rgb_optical_frame";
	centroid_kinectframe_rung1.header.stamp = ros::Time();	

	try
	{
		geometry_msgs::PointStamped odompoint1;
		listener.transformPoint("odom_combined", centroid_kinectframe_rung1, odompoint1);
		ROS_INFO("the point1 in the kinectframe is %f %f %f \n", centroid_kinectframe_rung1.point.x, centroid_kinectframe_rung1.point.y, centroid_kinectframe_rung1.point.z);
		ROS_INFO("the point1 in the odomcombined frame is %f %f %f \n", odompoint1.point.x, odompoint1.point.y, odompoint1.point.z);		
		centroid_odomcombined_rung1_pub.publish(odompoint1);	
	}
	catch (tf::TransformException& ex)
	{
		ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());	
	}
	
}

void transferFromKinectToOdom2(const tf::TransformListener& listener)
{
	centroid_kinectframe_rung2.header.frame_id = "head_mount_kinect_rgb_optical_frame";
	centroid_kinectframe_rung2.header.stamp = ros::Time();
	
	try
	{
		geometry_msgs::PointStamped odompoint2;
		listener.transformPoint("odom_combined", centroid_kinectframe_rung2, odompoint2);
		ROS_INFO("the point2 in the kinectframe is %f %f %f \n", centroid_kinectframe_rung2.point.x, centroid_kinectframe_rung2.point.y, centroid_kinectframe_rung2.point.z);
		ROS_INFO("the point2 in the odomcombined frame is %f %f %f \n", odompoint2.point.x, odompoint2.point.y, odompoint2.point.z);			
		centroid_odomcombined_rung2_pub.publish(odompoint2);	
	}
	catch (tf::TransformException& ex)
	{
		ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());	
	}
}

int main(int argc, char **argv)
{
	// Initialize ROS
	ros::init(argc, argv, "robot_tf_listener");
	ros::NodeHandle n;
		
	centroid_odomcombined_rung1_pub = n.advertise<geometry_msgs::PointStamped>("Centroid_Odom_Combined_Rung1", 1);
	centroid_odomcombined_rung2_pub = n.advertise<geometry_msgs::PointStamped>("Centroid_Odom_Combined_Rung2", 1);
	// Create a ROS subscriber for the input point cloud
	ros::Subscriber centroid_kinectframe_rung1_sub = n.subscribe("Centroid_Rung1_Kinect_Frame", 1, Find_Centroid_Kinect_Frame1);
	ros::Subscriber centroid_kinectframe_rung2_sub = n.subscribe("Centroid_Rung2_Kinect_Frame", 1, Find_Centroid_Kinect_Frame2);
	
	tf::TransformListener listener1(ros::Duration(10));
	// we'll transform a point once every second
	ros::Timer timer1 = n.createTimer(ros::Duration(1.0), boost::bind(&transferFromKinectToOdom1, boost::ref(listener1)));
	
	tf::TransformListener listener2(ros::Duration(10));
	ros::Timer timer2 = n.createTimer(ros::Duration(1.0), boost::bind(&transferFromKinectToOdom2, boost::ref(listener2)));
 
	ros::spin();		
}
