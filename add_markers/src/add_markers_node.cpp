
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>


class AddMarker{
private:
	ros::NodeHandle n;
  	ros::Subscriber odom_sub;
  	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	double pickUp[2]  = {4.00, 6.00};
  	double dropOff[2] = {-2.00, -1.00};
  	double threshold_x;
  	double threshold_y;
  	double robot_x;
 	double robot_y;
  	nav_msgs::Odometry pose_msg;
  	visualization_msgs::Marker marker;

public:
  AddMarker(){
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    odom_sub = n.subscribe("/odom", 10,  &AddMarker::odom_callback, this);


    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one

    marker.ns = "add_markers";
    marker.id = 0;


    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
     marker.type = visualization_msgs::Marker::CUBE;

  //  marker.type = shape; (Gowtham)


    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    // Set marker's initial coordinates
    setMarkerPosition(pickUp[0], pickUp[1]);

    marker.lifetime = ros::Duration();
    threshold_x = marker.scale.x;
    threshold_y = marker.scale.y;
    marker_pub.publish(marker);
 }


//
 void odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg){
  pose_msg = *odom_msg;
  robot_x = pose_msg.pose.pose.position.x;
  robot_y = pose_msg.pose.pose.position.y;


  bool atPickup = (pickUp[0] - threshold_x < robot_x && robot_x < pickUp[0] + threshold_x)
                && (pickUp[1] - threshold_y < robot_y && robot_y < pickUp[1] + threshold_y);
  bool atDropOff = (dropOff[0] - threshold_x < robot_x && robot_x < dropOff[0] + threshold_x)
                && (dropOff[1] - threshold_y < robot_y && robot_y < dropOff[1] + threshold_y);

  if (atPickup) {
  	// pause 5s to simulate pickup
        ros::Duration(5).sleep();
        // hide marker and set it to new coordinates
        setMarkerPosition(dropOff[0], dropOff[1]);
        marker.color.a = 0.0;
	ROS_INFO("At pickup location...");
        } 
  if (atDropOff) {
	
        marker.color.a = 1.0;
	ROS_INFO("At dropoff location...");
        }
  marker_pub.publish(marker);	

 }

 void setMarkerPosition(double pos_x, double pos_y) {
 // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
 marker.pose.position.x = pos_x;
 marker.pose.position.y = pos_y;
}
};


int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  
  AddMarker addMarker;
  ros::spin();

  return 0;

}
