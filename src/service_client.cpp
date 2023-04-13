#include "ros/ros.h"
#include "cwork4/tf_frame.h"
#include <tf/transform_broadcaster.h>

using namespace std;

int main(int argc, char **argv) {

	//Init the ROS node with service_client name
	ros::init(argc, argv, "pose_client");
	ros::NodeHandle n;

	ros::ServiceClient client = n.serviceClient<cwork4::tf_frame>("pose");
	
	cwork4::tf_frame srv;

	tf::TransformBroadcaster br;
    tf::Transform transform;

	srv.request.frame_a.data = "/base_link";
	srv.request.frame_b.data  = "/arm6_link";

	ros::Rate r(10);
	
	while(ros::ok()){

		ROS_INFO("Waiting for the client server");
		client.waitForExistence();
		ROS_INFO("Client server up now");
		
		if (!client.call(srv)) {
			ROS_ERROR("Error calling the service");
			return 1;
		}

		//Just print the output
		cout << "Service output: " << srv.response.pose << endl;

		tf::Quaternion q(srv.response.pose.orientation.x, srv.response.pose.orientation.y, srv.response.pose.orientation.z, srv.response.pose.orientation.w);
        transform.setOrigin(tf::Vector3(srv.response.pose.position.x, srv.response.pose.position.y, srv.response.pose.position.z));

        transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), srv.request.frame_a.data, srv.request.frame_b.data));

		r.sleep();
	}
	
	return 0;
}
