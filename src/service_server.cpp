#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "cwork4/tf_frame.h"


using namespace std;


bool service_callback( cwork4::tf_frame::Request &req, cwork4::tf_frame::Response &res) {

    tf::TransformListener listener;
    tf::StampedTransform transform;

     try {
            listener.waitForTransform(req.frame_a.data, req.frame_b.data, ros::Time(0), ros::Duration(1.0));
            listener.lookupTransform(req.frame_a.data, req.frame_b.data, ros::Time(0), transform);
            res.pose.position.x = transform.getOrigin().x();
            res.pose.position.y = transform.getOrigin().y();
            res.pose.position.z = transform.getOrigin().z();
            res.pose.orientation.x = transform.getRotation().x();
            res.pose.orientation.y = transform.getRotation().y();
            res.pose.orientation.z = transform.getRotation().z();
            res.pose.orientation.w = transform.getRotation().w();
            usleep(1000000);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

    ROS_INFO_STREAM(" Transform: " << 
        
            transform.getOrigin().x() << ", " << 
            transform.getOrigin().y() << ", " <<
            transform.getOrigin().z() << ", " << 
            transform.getRotation().x() << ", " << 
            transform.getRotation().y() << ", " << 
            transform.getRotation().z() << ", " << 
            transform.getRotation().w()
        );

	return true;
}





int main(int argc, char** argv){
    
    ros::init(argc, argv, "pose_service");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("pose", service_callback);

	//Call the spin function to maintain the node alive
	ros::spin();

    return 0;
}