#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <tiago_iaslab_simulation/AssignmentAction.h>
#include <tiago_iaslab_simulation/AssignmentGoal.h>
#include <tiago_iaslab_simulation/AssignmentResult.h>
#include <tiago_iaslab_simulation/AssignmentFeedback.h>

#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/GetMap.h>

class FrameTransformer {

	private:
	geometry_msgs::PoseWithCovarianceStamped map_pose;

	protected:
	ros::NodeHandle _nh;
	actionlib::SimpleActionClient<tiago_iaslab_simulation::AssignmentAction> _ac;
	ros::ServiceClient _client;

	public:
	// FrameTransformer(const FrameTransformer& right_side) {};
	// 
	FrameTransformer():
		_ac("/goal_status", true)
	{
		ROS_INFO("Wait for the goal_status action server to start...");
		_ac.waitForServer();
		ROS_INFO("Server goal_status is now up.");
	}

	//__________________________________ Callbacks __________________________________//
	// Called once when the goal completes
	void doneCb(const actionlib::SimpleClientGoalState& state, const tiago_iaslab_simulation::AssignmentResultConstPtr& result)
	{
		ROS_INFO("Finished in state [%s]", state.toString().c_str());
		ros::shutdown();
	}

	// Called once when the goal becomes active
	void activeCb()
	{
		ROS_INFO("Goal just went active");
	}

	// Called every time feedback is received for the goal
	void feedbackCb(const tiago_iaslab_simulation::AssignmentFeedbackConstPtr& feedback)
	{
		int status = feedback->status;
		switch (status) 
		{
			case 1:
				ROS_INFO("Robot moving"); 
				break;
			case 2:
				ROS_INFO("Robot stopped");
				break;
			case 3:
				ROS_INFO("Robot obstacle detection with laser");
				break;
			case 4:
				ROS_INFO("Robot computing obstacle coordinates position");
				break;
			case 5:
				ROS_INFO("Robot draw marker");
				break;
		}


		// ROS_INFO("[X]:%f [Y]:%f [W]: %f",feedback->base_position.pose.position.x,feedback->base_position.pose.position.y,feedback->base_position.pose.orientation.w); 
		// ROS_INFO("Got Feedback");
	}


	//__________________________________ Methods __________________________________//
	void sendGoal() {

		// check if the pose goal is feasible
		_client = _nh.serviceClient<nav_msgs::GetMap>("/static_map");

		nav_msgs::GetMap srv;

		if (_client.call(srv)) // this actually calls the service server
		{
			ROS_INFO("Done to call service"); // Service called, srv get value inside
			ROS_INFO("I heard [%s]", srv.response.map.header.frame_id.c_str());
		}
		else
		{
			ROS_ERROR("Failed to call service");
		}

		// command line input from user
		float x; 
		float y;
		float z = 0.0;
		float yaw;
		float roll = 0.0;
		float pitch = 0.0;

		int cell_x;
		int cell_y;

		do // keep asking for goal pose until pose is feasible
		{
			printf("Enter your position goal x: ");
			scanf("%f", &x);
			printf("Enter your position goal y: ");
			scanf("%f", &y);
			printf("Enter your orientation goal yaw: ");
			scanf("%f", &yaw);

			printf("You have inserted position goal x = %f and y = %f and orientation goal w: %f \n", x, y, yaw);

			cell_x = int(x - srv.response.map.info.origin.position.x) / srv.response.map.info.resolution;
			cell_y = int(y - srv.response.map.info.origin.position.y) / srv.response.map.info.resolution;

			// check if feasible in our static map
			if (srv.response.map.data[cell_x+srv.response.map.info.width*cell_y] == -1) // unknown -> outside the map
			{
				ROS_ERROR("Goal pose is outside the static map");
			}

		} while (srv.response.map.data[cell_x+srv.response.map.info.width*cell_y] == -1);

		// trannsformation from Euler angles to Quaternion (used by move_base)
		tf2::Quaternion quaternion;
		quaternion.setRPY(roll,pitch,yaw);
		quaternion = quaternion.normalize();

		geometry_msgs::TransformStamped transformStamped;
		transformStamped.transform.rotation.x = quaternion.x();
		transformStamped.transform.rotation.y = quaternion.y();
		transformStamped.transform.rotation.z = quaternion.z();
		transformStamped.transform.rotation.w = quaternion.w();
		
		// we'll send a goal to the robot using quaternion
		tiago_iaslab_simulation::AssignmentGoal goal;

		goal.target_pose.header.frame_id = "map";
		goal.target_pose.header.stamp = ros::Time::now();
		goal.target_pose.pose.position.x = x;
		goal.target_pose.pose.position.y = y;
		goal.target_pose.pose.position.z = z;
		goal.target_pose.pose.orientation.x = transformStamped.transform.rotation.x;
		goal.target_pose.pose.orientation.y = transformStamped.transform.rotation.y;
		goal.target_pose.pose.orientation.z = transformStamped.transform.rotation.z;
		goal.target_pose.pose.orientation.w = transformStamped.transform.rotation.w;

		// send goal to the action server
		_ac.sendGoal(goal, 
			boost::bind(&FrameTransformer::doneCb, this, _1, _2),
			boost::bind(&FrameTransformer::activeCb, this),
			boost::bind(&FrameTransformer::feedbackCb, this, _1));
		ROS_INFO("Sending goal");

	}
	
};

int main (int argc, char **argv)
{	
	ros::init(argc, argv, "simple_navigation_goals");
	FrameTransformer frame_tf;
	ROS_INFO("Waiting for arm to be tucked");
	ros::Duration(5).sleep(); // sleep for half a second (wait gripper to close)
	frame_tf.sendGoal();
	ros::spin();
}

