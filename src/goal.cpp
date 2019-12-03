#include <ros/ros.h>
#include <time.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int16MultiArray.h>
#include <tf/transform_listener.h>

using namespace std;
tf::StampedTransform transform_mo;

class Goal
{
public:
	Goal();
	void pspaceCB(const std_msgs::Int16MultiArray& msg);

private:
	ros::NodeHandle nh;
	ros::Subscriber pspace_sub;
	ros::Publisher goal_pub;

	geometry_msgs::PoseStamped goal_point;
	int parking_type;
	int prev_id;
};

Goal::Goal()
{
	nh = ros::NodeHandle("");
	
	pspace_sub = nh.subscribe("/detector/p_space", 1,&Goal::pspaceCB, this);
	goal_pub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal",1);
	
	nh.param("ptype", parking_type, 0);
	prev_id = -1;
}


void Goal::pspaceCB(const std_msgs::Int16MultiArray& msg)
{
	cout << "nav_goal callback : " << msg.data[0] << endl;
	double z_val = 0;
	double w_val = 1;
	if(parking_type == 1){
		cout << "parking type is front" << endl;
		if(msg.data[0] < 5){
			z_val = -0.0109296593219;
			w_val = 0.99994026949;
		}else{
			z_val = 0.999998850962;
			w_val = -0.00142391099113;
		}
	}else{
		cout << "parking type is back" << endl;
		if(msg.data[0] < 5){
			z_val = 0.999998850962;
			w_val = -0.00142391099113;
		}else{
			z_val = -0.0109296593219;
			w_val = 0.99994026949;

		}
	}
	if(msg.data[0] != prev_id){
		//ros::Duration(0.5).sleep();
		goal_point.header.frame_id = "odom";
		goal_point.header.stamp = ros::Time::now();
		if(msg.data[0] < 5){
			goal_point.pose.position.x = (1200 - (msg.data[2]+25))*0.005 - transform_mo.getOrigin().y();
		}
		else{
			goal_point.pose.position.x = (1200 - (msg.data[2]-25))*0.005 - transform_mo.getOrigin().y();
		}
		goal_point.pose.position.y = transform_mo.getOrigin().x() - (msg.data[1])*0.005;
		//goal_point.pose.position.x = transform_mo.getOrigin().x() - (msg.data[1])*0.005;
		//goal_point.pose.position.y = (1200 - msg.data[2])*0.005 - transform_mo.getOrigin().y();
		goal_point.pose.position.z = 0;
		goal_point.pose.orientation.x = 0;
		goal_point.pose.orientation.y = 0;
		goal_point.pose.orientation.z = z_val;
		goal_point.pose.orientation.w = w_val;
		
		goal_pub.publish(goal_point);
		cout << "nav_goal pub!!!!" << endl;
		prev_id = msg.data[0];
	}

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "nav_goal");
	Goal Goal;

	ros::Rate rate(1);
	tf::TransformListener listener;
	ros::Duration(2).sleep();
	while(ros::ok()){
    		try{
      			listener.lookupTransform("map", "odom",  ros::Time(0), transform_mo);
			break;
    		}
    		catch (tf::TransformException ex){
      			ROS_ERROR("%s",ex.what());
    		}
	}
	cout << transform_mo.getOrigin().x() << endl;
	while(ros::ok()){
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}
