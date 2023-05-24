/**
 * @author      : frekle (frekle@bml01.mech.kth.se)
 * @file        : custom_clock_node
 * @created     : Monday Apr 17, 2023 10:36:18 CEST
 */

#ifndef CUSTOM_CLOCK_NODE_H

#define CUSTOM_CLOCK_NODE_H

#include "ros/publisher.h"
#include "ros/service_server.h"
#include "std_srvs/Empty.h"
#include "std_srvs/EmptyRequest.h"
#include "std_srvs/EmptyResponse.h"
#include <boost/chrono/duration.hpp>
#include <boost/chrono/system_clocks.hpp>
class custom_clock
{
	public:
		custom_clock();
		~custom_clock();
		bool playing;
		void pub_once_different_time();
		void pub_same_time_at_different_rate();
		void sleep();
		uint64_t start_time, current_faketime;
		uint64_t real_sim_clock_between_msgs_duration;

		int clock_step_microsseconds; 
		ros::Publisher clock_pub;
		ros::ServiceServer start_clock_srv, stop_clock_srv, step_clock_srv;
		bool start(std_srvs::EmptyRequest&req, std_srvs::EmptyResponse &res);
		bool stop(std_srvs::EmptyRequest&req, std_srvs::EmptyResponse &res);
		bool step(std_srvs::EmptyRequest&req, std_srvs::EmptyResponse &res);
};

#endif /* end of include guard CUSTOM_CLOCK_NODE_H */

