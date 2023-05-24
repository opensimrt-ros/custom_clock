#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "rosgraph_msgs/Clock.h"
#include <boost/chrono/duration.hpp>
#include <boost/chrono/system_clocks.hpp>
#include <boost/thread/thread.hpp>

class custom_clock
{
	public:
		custom_clock();
		void pub_once_different_time();
		uint64_t current_time, clock_step_nanosseconds;
		boost::chrono::nanoseconds real_sim_clock_between_msgs_duration;
		ros::Publisher clock_pub;
};

custom_clock::custom_clock()
{
	ros::NodeHandle n;
	clock_pub = n.advertise<rosgraph_msgs::Clock>("/clock", 10); 
	current_time = boost::chrono::system_clock::now().time_since_epoch().count();
}

void custom_clock::pub_once_different_time()
{
	rosgraph_msgs::Clock clock_msg;
	current_time += clock_step_nanosseconds;
	clock_msg.clock.fromNSec(current_time);
	clock_pub.publish(clock_msg);

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "custom_clock");
	custom_clock c;
	//set fake rate:
	c.real_sim_clock_between_msgs_duration = boost::chrono::milliseconds(10);
	c.clock_step_nanosseconds = 100;
	while(ros::ok())
	{
		c.pub_once_different_time();
		boost::this_thread::sleep_for( c.real_sim_clock_between_msgs_duration );

	}
	return 0;
}
