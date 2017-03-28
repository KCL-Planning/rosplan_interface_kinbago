#include "rosplan_interface_enterelevator/RPEnterElevator.h"

/* The implementation of RPEnterElevator.h */
namespace KCL_rosplan {

	/* constructor */
	RPEnterElevator::RPEnterElevator(ros::NodeHandle &nh)
	 : message_store(nh) {	
		
	}

	/* action dispatch callback */
	bool RPEnterElevator::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

	
	ROS_INFO("KCL: (%s) action finished", params.name.c_str());
		return true;

	}
} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc, argv, "rosplan_interface_enterelevator");
		ros::NodeHandle nh("~");

		// create PDDL action subscriber
		KCL_rosplan::RPEnterElevator rpee(nh);

		// listen for action dispatch
		ros::Subscriber ds = nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::RPActionInterface::dispatchCallback, dynamic_cast<KCL_rosplan::RPActionInterface*>(&rpee));
		rpee.runActionInterface();


		return 0;
	}
