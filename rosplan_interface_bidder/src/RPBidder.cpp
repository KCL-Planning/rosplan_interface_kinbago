#include "rosplan_interface_bidder/RPBidder.h"

/* The implementation of RPBidder.h */
namespace KCL_rosplan {

	/* constructor */
	RPBidder::RPBidder(ros::NodeHandle &nh) {

		node_handle = &nh;

		// knowledge interface
		update_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
		current_goals_client = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/kcl_rosplan/get_current_goals");

		// planning interface
		std::string probTopic = "/rosplan_problem_interface/problem_generation_server";
		std::string planTopic = "/rosplan_planner_interface/planning_server";
		std::string parsTopic = "/rosplan_parsing_interface/parse_plan";
		std::string dispTopic = "/rosplan_plan_dispatcher/dispatch_plan";

		node_handle->getParam("problem_service_topic", probTopic);
		node_handle->getParam("planning_service_topic", planTopic);
		node_handle->getParam("parsing_service_topic", parsTopic);
		node_handle->getParam("dispatch_service_topic", dispTopic);

		problem_client = nh.serviceClient<std_srvs::Empty>(probTopic);
		planning_client = nh.serviceClient<std_srvs::Empty>(planTopic);
		parsing_client = nh.serviceClient<std_srvs::Empty>(parsTopic);
		dispatch_client = nh.serviceClient<std_srvs::Empty>(dispTopic);

		// create publisher
		std::string bidTopic = "/task/bid";
		node_handle->getParam("bid_topic", bidTopic);
		bid_publisher = node_handle->advertise<std_msgs::String>(bidTopic, 1, true);

		generating_bid = false;
	}

	/*--------------*/
	/* announcement */
	/*--------------*/

	void RPBidder::announceCallback(const std_msgs::String::ConstPtr& msg) {

		// only generate one bid at a time
		if(generating_bid) return;

		generating_bid = true;
		goals.clear();

		ROS_INFO("KCL: (%s) New task announced.", ros::this_node::getName().c_str());

		// retrieve old goals and save
		rosplan_knowledge_msgs::GetAttributeService currentGoalSrv;
		if (!current_goals_client.call(currentGoalSrv)) {
			ROS_ERROR("KCL: (%s) Failed to call goal service.", ros::this_node::getName().c_str());
		} else {
			goals = currentGoalSrv.response.attributes;
		}

		// clear old goals
		rosplan_knowledge_msgs::KnowledgeUpdateService updateSrv;
		updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_GOAL;
		updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		updateSrv.request.knowledge.attribute_name = "";
		updateSrv.request.knowledge.values.clear();
		update_knowledge_client.call(updateSrv);

		// insert new goals
		updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_GOAL;
		updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		updateSrv.request.knowledge.attribute_name = "docked";
		diagnostic_msgs::KeyValue pair;
		pair.key = "v";
		pair.value = "kenny";
		updateSrv.request.knowledge.values.push_back(pair);
		update_knowledge_client.call(updateSrv);

		// generate problem and plan
		ROS_INFO("KCL: (%s) Generating plan for bid.", ros::this_node::getName().c_str());
		new_plan_recieved = false;

		std_srvs::Empty empty;
		problem_client.call(empty);
		planning_client.call(empty);
		parsing_client.call(empty);

		while(!new_plan_recieved && ros::ok()) ros::spinOnce();

		// compute bid from plan


		ROS_INFO("KCL: (%s) Resetting goals.", ros::this_node::getName().c_str());

		// clear goals again
		updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_GOAL;
		updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		updateSrv.request.knowledge.attribute_name = "";
		updateSrv.request.knowledge.values.clear();
		update_knowledge_client.call(updateSrv);

		// replace with old goals
		updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_GOAL;
		std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator kit = goals.begin();
		for(; kit!=goals.end(); kit++) {
			updateSrv.request.knowledge = (*kit);
			update_knowledge_client.call(updateSrv);
		}

		// publish bid
		ROS_INFO("KCL: (%s) Publishing bid.", ros::this_node::getName().c_str());

		std_msgs::String bid_msg;
		bid_msg.data = "IWANTIT";
		bid_publisher.publish(bid_msg);

		generating_bid = false;
	}

	void RPBidder::planCallback(const rosplan_dispatch_msgs::EsterelPlan& msg) {
		last_plan = msg;
		new_plan_recieved = true;
	}

	/*-------*/
	/* award */
	/*-------*/

	void RPBidder::awardCallback(const std_msgs::String::ConstPtr& msg) {

		// no awarding while bidding
		if(generating_bid) return;

		ROS_INFO("KCL: (%s) New task awarded.", ros::this_node::getName().c_str());

		// insert new goals
		rosplan_knowledge_msgs::KnowledgeUpdateService updateSrv;
		updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_GOAL;
		updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		updateSrv.request.knowledge.attribute_name = "docked";
		diagnostic_msgs::KeyValue pair;
		pair.key = "v";
		pair.value = "kenny";
		updateSrv.request.knowledge.values.push_back(pair);
		update_knowledge_client.call(updateSrv);

		// generate problem and plan
		ROS_INFO("KCL: (%s) Sending to planning system.", ros::this_node::getName().c_str());

		std_srvs::Empty empty;
		problem_client.call(empty);
		planning_client.call(empty);
		parsing_client.call(empty);
		dispatch_client.call(empty);
	}

} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc, argv, "rosplan_interface_bidder");
		ros::NodeHandle nh("~");

		// params
		std::string ans_topic;
		std::string aws_topic;
		std::string lps_topic;
		nh.param("task_announcement_topic", ans_topic, std::string("/tasks/announce"));
		nh.param("task_award_topic", aws_topic, std::string("/tasks/award"));
		nh.param("complete_plan_topic", lps_topic, std::string("/rosplan_plan_dispatcher/dispatch_plan"));

		// create bidder
		KCL_rosplan::RPBidder rpb(nh);

		// listen for tasks
		ros::Subscriber ans = nh.subscribe(ans_topic, 100, &KCL_rosplan::RPBidder::announceCallback, &rpb);
		ros::Subscriber aws = nh.subscribe(aws_topic, 100, &KCL_rosplan::RPBidder::awardCallback, &rpb);
		ros::Subscriber lps = nh.subscribe(lps_topic, 100, &KCL_rosplan::RPBidder::planCallback, &rpb);

		ROS_INFO("KCL: (%s) Ready to receive", ros::this_node::getName().c_str());

		ros::spin();

		return 0;
	}
