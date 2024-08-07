/**
 * @author      : frekle (frekle@bml01.mech.kth.se)
 * @file        : custom_clock_node
 * @created     : Monday Apr 17, 2023 10:35:14 CEST
 */
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "rosgraph_msgs/Clock.h"
#include "std_srvs/EmptyRequest.h"
#include <boost/chrono/duration.hpp>
#include <boost/chrono/system_clocks.hpp>
#include <custom_clock/custom_clock_node.h>
#include <boost/thread/thread.hpp>

custom_clock::custom_clock()
{
	ros::NodeHandle n("~");
	clock_pub = n.advertise<rosgraph_msgs::Clock>("/clock", 10);
	start_clock_srv = n.advertiseService("start", &custom_clock::start, this);
	stop_clock_srv = n.advertiseService("stop", &custom_clock::stop, this);
	n.param<bool>("start_playing", playing, true);
	n.param<int>("clock_step_microsseconds", clock_step_microsseconds, 1000);
	int slowdown_rate;
	//DEFAULT: run 100 times slower than normal clock
	n.param<int>("slowdown_rate", slowdown_rate, 100);
	int start_secs, start_nsecs; //I will split the times here like I did for ik_dumper.
	bool use_custom_start_time;
	n.param<bool>("use_custom_start_time", use_custom_start_time, false);
	n.param<int>("start_at_secs", start_secs, 1668000000);
	n.param<int>("start_at_nsecs", start_nsecs, 0);

	real_sim_clock_between_msgs_duration = slowdown_rate*clock_step_microsseconds*1000;
	if (use_custom_start_time)
	{
		ROS_INFO("using custom start time!");
		start_time = start_secs * 1000000000LL + start_nsecs;
	}
	else
		start_time = boost::chrono::system_clock::now().time_since_epoch().count();
	current_faketime = start_time;
}

bool custom_clock::start(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res )
{
	playing = true;
	return true;	
}
bool custom_clock::stop(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res )
{
	playing = false;
	return true;	
}
bool custom_clock::step(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res )
{
	pub_once_different_time();
	return true;	
}


void custom_clock::pub_same_time_at_different_rate()
{
	/*
	 *	Published the same time, but at any given rate
	 * */
	rosgraph_msgs::Clock clock_msg;
	//gets wall time
	boost::chrono::system_clock::time_point t = boost::chrono::system_clock::now();
	clock_msg.clock.fromNSec(t.time_since_epoch().count());
	clock_pub.publish(clock_msg);

}
void custom_clock::pub_once_different_time()
{
	rosgraph_msgs::Clock clock_msg;
	//gets wall time
	current_faketime += clock_step_microsseconds*1000;
	clock_msg.clock.fromNSec(current_faketime);
	clock_pub.publish(clock_msg);

}
void custom_clock::sleep()
{
	//lets try to make this more accurate. 
	boost::chrono::system_clock::rep before_spin = boost::chrono::system_clock::now().time_since_epoch().count();
	ros::spinOnce(); //If I want to be able to stop and start I need to spin it. 
	boost::chrono::system_clock::rep after_spin = boost::chrono::system_clock::now().time_since_epoch().count();
	uint64_t time_difference = after_spin-before_spin;
	//std::cout << time_difference << std::endl;
	boost::chrono::nanoseconds sleep_amount_ns(real_sim_clock_between_msgs_duration-time_difference);
	boost::this_thread::sleep_for( sleep_amount_ns );

}

custom_clock::~custom_clock()
{
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "custom_clock");
	custom_clock c;
	//set fake rate:
	while(ros::ok())
	{
		if (c.playing)
			c.pub_once_different_time();
		//c.pub_same_time_at_different_rate();
		c.sleep();	
	}
	return 0;
}
