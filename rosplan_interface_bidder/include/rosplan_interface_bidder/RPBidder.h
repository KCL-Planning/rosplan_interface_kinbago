#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>

#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "rosplan_knowledge_msgs/GetAttributeService.h"
#include "rosplan_dispatch_msgs/EsterelPlan.h"

#include "std_msgs/String.h"
#include "std_srvs/Empty.h"

#ifndef KCL_bidder
#define KCL_bidder

/**
 * This file defines the RPBidder class.
 * RPBidder is used to recieve announcements, make bids, recieve awards, and call the executor.
 */
namespace KCL_rosplan {

	class RPBidder
	{

	private:

		ros::NodeHandle* node_handle;

		/* rosplan knowledge interface */
		ros::ServiceClient update_knowledge_client;
		ros::ServiceClient current_goals_client;
		std::vector<rosplan_knowledge_msgs::KnowledgeItem> goals;

		/* planning and bidding interface */
		ros::ServiceClient problem_client;
		ros::ServiceClient planning_client;
		ros::ServiceClient parsing_client;
		ros::ServiceClient dispatch_client;

		ros::Publisher bid_publisher;

		rosplan_dispatch_msgs::EsterelPlan last_plan;
		bool generating_bid;
		bool new_plan_recieved;

	public:

		/* constructor */
		RPBidder(ros::NodeHandle &nh);

		/* listen to and process action_dispatch topic */
		void announceCallback(const std_msgs::String::ConstPtr& msg);
		void awardCallback(const std_msgs::String::ConstPtr& msg);
		void planCallback(const rosplan_dispatch_msgs::EsterelPlan& msg);
	};
}
#endif
