#include <ros/ros.h>
#include <time.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int16MultiArray.h>

using namespace std;

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
	prev_id = 0;
}


void Goal::pspaceCB(const std_msgs::Int16MultiArray& msg)
{
	double z_val = 0;
	double w_val = 1;
	if(parking_type == 1){
		z_val = 0.70520064234;
		w_val = 0.70909008370475;
	}else{
		z_val = -0.711190612984;
		w_val = 0.0702999226476;
	}
	if(msg.data[0] != prev_id){
		goal_point.header.frame_id = "map";
		goal_point.header.stamp = ros::Time::now();
		goal_point.pose.position.x = (msg.data[1])*0.005;
		goal_point.pose.position.y = (1200 - msg.data[2])*0.005;
		goal_point.pose.position.z = 0;
		goal_point.pose.orientation.x = 0;
		goal_point.pose.orientation.y = 0;
		goal_point.pose.orientation.z = z_val;
		goal_point.pose.orientation.w = w_val;

		goal_pub.publish(goal_point);
		prev_id = msg.data[0];
	}

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "nav_goal");
	Goal Goal;

	ros::Rate rate(1);
	while(ros::ok()){
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}
