#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <boost/foreach.hpp>

#include "rosplan_action_interface/RPActionInterface.h"

#include "actionlib/client/simple_action_client.h"
#include "mongodb_store/message_store.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_srvs/Empty.h"

#ifndef KCL_movebase
#define KCL_movebase

/**
 * This file defines the RPEnterElevator class.
 * RPExirElevator is used to start the elevator enter procedure.
 * PDDL "enter_elevator" action is listened for.
 * Waypoint goals are fetched by name from the SceneDB (implemented by mongoDB).
 */
namespace KCL_rosplan {

	class RPEnterElevator: public RPActionInterface
	{

	private:

		mongodb_store::MessageStoreProxy message_store; 		
							
	//initialpose o sa fie aici	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> action_client;


	public:

		/* constructor */
		RPEnterElevator(ros::NodeHandle &nh);     

		/* listen to and process action_dispatch topic */
		bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);  
	};
}
#endif
