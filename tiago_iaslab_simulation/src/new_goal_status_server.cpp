#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseResult.h>
#include <move_base_msgs/MoveBaseFeedback.h>

#include <tiago_iaslab_simulation/AssignmentAction.h>
#include <tiago_iaslab_simulation/AssignmentGoal.h>
#include <tiago_iaslab_simulation/AssignmentResult.h>
#include <tiago_iaslab_simulation/AssignmentFeedback.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>

#include <geometry_msgs/Twist.h>  // for publishing cmd_vel

#include <math.h> // isinf()

// action msg created by me -> adding ID 
// eg. 1 = moving
// eg. 2 = stopped

class CountUntilServer {

	private:	
	ros::Publisher vis_pub;
	ros::Publisher cmdVel_pub;

	ros::Subscriber laserScan_sub;
	ros::Subscriber tf_sub;

	std::vector<sensor_msgs::LaserScan> scanLaser;

	std::vector<float> ranges;
	std::vector<float> intensities;

	float angle_min;
	float angle_max;
	float angle_increment;

	std::vector<float> x_vector;
	std::vector<float> y_vector;

	geometry_msgs::PoseWithCovarianceStamped base_laser_link_pose;
	geometry_msgs::PoseWithCovarianceStamped map_pose;
	geometry_msgs::TransformStamped transformStamped;

	float frontal_laser_range;
	// float frontal_right_laser_range;
	// float frontal_left_laser_range;
	float right_laser_range;
	float left_laser_range;

	move_base_msgs::MoveBaseGoal move_base_goal;

	bool pose_reached;
	bool goal_canceled;
	bool detection_active;

	protected:
	ros::NodeHandle _nh;
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> _ac;
	actionlib::SimpleActionServer<tiago_iaslab_simulation::AssignmentAction> _as;
	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener* tfListener;	

	tiago_iaslab_simulation::AssignmentFeedback _feedback;
	tiago_iaslab_simulation::AssignmentResult _result;
	
	public:
	CountUntilServer(): goal_canceled(false), pose_reached(false), detection_active(true),
		_as(_nh, "/goal_status", 
			boost::bind(&CountUntilServer::acquireGoal, this, _1), 
			false),
		_ac("/move_base", true)
	{
		ROS_INFO("Wait for the move_base action server to start...");
		_ac.waitForServer();
		ROS_INFO("Server move_base is now up.");

		_as.start();
		ROS_INFO("Simple Action Server goal_status has been started.");

		laserScan_sub = _nh.subscribe("/scan", 1000, 
		    &CountUntilServer::laserScan_callback, this);

		cmdVel_pub = _nh.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 1);
		vis_pub = _nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

		tfListener = new tf2_ros::TransformListener(tfBuffer);
	}


	//__________________________________ Callbacks __________________________________//
	// Called once when the goal completes
	void doneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result)
	{

		if (goal_canceled==true) // _as.isPreemptRequested()
		{
			ROS_INFO("Goal move_base cancelled");
			return;
		}
		
		/*
		if (goal_canceled==true && detection_active==true) // _as.isPreemptRequested()
		{
			ROS_INFO("Goal move_base cancelled");
			return;
		}
		*/

		// otherwise with if/else and no narrow space always end with goal cancelled true 
		ROS_INFO("Goal move_base just done. Pose reached");
		pose_reached = true;
	}

	// Called once when the goal becomes active
	void activeCb()
	{
		ROS_INFO("Goal move_base just went active");
	}

	// Called every time feedback is received for the goal
	void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
	{

		// ROS_INFO("[X]:%f [Y]:%f [W]: %f",feedback->base_position.pose.position.x,feedback->base_position.pose.position.y,feedback->base_position.pose.orientation.w); 
		// ROS_INFO("Got Feedback");
	}
	
	void laserScan_callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
		
		_feedback.status = 3;
		// _as.publishFeedback(_feedback); // one feedback (not keep publishing in while)
	
		sensor_msgs::LaserScan tempScan = *msg;
		angle_min = msg->angle_min;
		angle_max = msg->angle_max;
		angle_increment = msg->angle_increment;

		scanLaser.push_back(tempScan); // perché non è un vettore (non nel for)

		int size_ranges = msg->ranges.size();
		// frontal_laser_range = msg->ranges[size_ranges/2]; // even number (size is 666)
		// frontal_right_laser_range = msg->ranges[200]; 
		// frontal_left_laser_range = msg->ranges[size_ranges-200]; 
		right_laser_range = msg->ranges[66]; 
		left_laser_range = msg->ranges[size_ranges-66];

		// ROS_INFO("Left laser_range: %f, Right laser_ramge: %f", left_laser_range, right_laser_range);

		if (pose_reached==true && detection_active==true) // exec callback one time and then unsubscribe
		{
			ranges.clear();
			intensities.clear();

			if (ranges.size()==0 || tempScan.header.seq != scanLaser.at(0).header.seq) {
				for (int i=0; i<(msg->ranges).size(); i++) { // same size of intensities
					float temp_ranges = msg->ranges[i];
					float temp_intensities = msg->intensities[i];
					int size_ranges = msg->ranges.size();
					int size_intensities = msg->intensities.size();

					ranges.push_back(temp_ranges); // perché è un vettore
					intensities.push_back(temp_intensities); // perché è un vettore
				}
			} else {
				ROS_INFO("subscriber for callback shutdown");
				laserScan_sub.shutdown();
			}
		}

		// ROS_INFO("subscriber for callback shutdown");
		// laserScan_sub.shutdown();
	}

	void tf_callback(const tf2_msgs::TFMessage::ConstPtr& msg) {

		ROS_INFO("Inside tf callback");
		_feedback.status = 4;
		// _as.publishFeedback(_feedback); // one feedback (not keep publishing in while)

		try
		{
		    ros::Time now = ros::Time::now();
		    if (tfBuffer.canTransform("map",
				              "base_laser_link",
				              now,
				              ros::Duration(0.01)))
		    {
			transformStamped = tfBuffer.lookupTransform("map",
				                                    "base_laser_link",
				                                    now);
			ROS_INFO("canTransform: TRUE");
			ROS_INFO("Transform: [%f, %f, %f]",
				 transformStamped.transform.translation.x,
				 transformStamped.transform.translation.y,
				 transformStamped.transform.translation.z);
			obstacle_detection();
		    }
		    else
		    {
			ROS_INFO("canTransform: FALSE");
		    }
		}
		catch (tf2::TransformException &ex)
		{
		    ROS_WARN("%s", ex.what());
		    ros::Duration(1);
		}
	}

	
	void acquireGoal(const tiago_iaslab_simulation::AssignmentGoalConstPtr& goal)
	{
		ROS_INFO("Goal received");
		// need conversion
		move_base_goal.target_pose = goal->target_pose; // same type (geometry_msgs/PoseStamped)
		makeGoal(move_base_goal);
	}

	void makeGoal(const move_base_msgs::MoveBaseGoal& move_base_goal)
	{
		
		ROS_INFO("Inside sendGoal"); 
		_ac.sendGoal(move_base_goal, 
			boost::bind(&CountUntilServer::doneCb, this, _1, _2),
			boost::bind(&CountUntilServer::activeCb, this),
			boost::bind(&CountUntilServer::feedbackCb, this, _1));
		ROS_INFO("Sending goal to move_base");
		
		onGoal();
	}

	
	void onGoal()
	{
		ROS_INFO("Inside onGoal");
		bool success = false;
		bool preempted = false;
		_feedback.status = 1;
		_as.publishFeedback(_feedback); // one feedback (not keep publishing in while)

		float threshold = 0.55;

		ros::Rate rate(5); // 10 Hz

		while (ros::ok()) {
		
			// ROS_INFO("Inside while");

			// ROS_INFO("Left laser_range: %f, Right laser_ramge: %f, Frontal laser_ramge: %f", left_laser_range, right_laser_range, frontal_laser_range);
			

			//_______________ Handling object detection _______________
			if (left_laser_range <= threshold && right_laser_range <= threshold)
			{	
				// ROS_INFO("Narrow space");
				if (goal_canceled == false)
				{
					goal_canceled = true;
					_ac.cancelGoal(); // cancel only once
					detection_active = false;
				} 
			
				//_______________ Handling movement to Goal _______________
				geometry_msgs::Twist vel;

				// compute difference between right and left laser scan
				float diff_laser_scan = right_laser_range - left_laser_range;
				
				// ROS_INFO("diff laser scan is: %f", diff_laser_scan); 

				vel.linear.x = 2;
				vel.linear.y = 0.5*diff_laser_scan;
				cmdVel_pub.publish(vel);
					
			}

			if ((left_laser_range > threshold || right_laser_range > threshold) && 					goal_canceled == true)
			{
					ROS_INFO("Out of narrow space"); 
					goal_canceled = false; // must be before the method call
					detection_active = true;
					makeGoal(move_base_goal);
			}
			
			
			if (pose_reached == true)
			{
				ROS_INFO("Activate tf_subscriber"); 
				pose_reached == false;
				detection_active = true;

				//_________________ Starting obstacle detection _________________
				tf_sub = _nh.subscribe("/tf", 1000, 
		    			&CountUntilServer::tf_callback, this);			
				ros::Duration(1).sleep(); // sleep for 1s

				obstacle_detection();
				draw_marker();
				success == true;
				break; // out of while loop
			}
			
			/*
			// handling result 
			if (_as.isPreemptRequested()) {
				preempted = true;
				break;
			}
			*/

			ros::spinOnce(); // so all our publishing in while are at fixed Rate
			rate.sleep();
		}

		/*
		ROS_INFO("Out of while loop");

		// sending result to client
		_result.task_completed = true;
		ROS_INFO("Send goal result to client");

		if (preempted) {
			ROS_INFO("Preempted");
			_as.setPreempted(_result);
		}
		else if (success) {
			ROS_INFO("Success");
			_as.setSucceeded(_result);
		}
		else {
			ROS_INFO("Aborted");
			_as.setAborted(_result);
		}
		*/
	}

	// Once reached Goal Pose, detect obstacles nearby
	void obstacle_detection() 
	{
		ROS_INFO("Obstacle detection");
		_feedback.status = 4;
		_as.publishFeedback(_feedback); // one feedback (not keep publishing in while)

		//_________________ Compute Cartesian Coordinates _________________
		// WARNING! This coordinates are wrt Laser Frame
		float j = 0;

		for (float i: ranges) {
			if (!isinf(i)) {
				// polar coordinates of the point		
				float angle = angle_min + (j * angle_increment);
				// polar to cartesian
				float x = i * cos(angle); // already in radians
				float y = i * sin(angle); // already in radians
    				// std::cout << i << ' ';
				// ROS_INFO("Cartesian coordinates x: %f, y: %f", x, y);
				x_vector.push_back(x);
				y_vector.push_back(y);				
			}
			j++;
		}

		for (int i = 0; i < x_vector.size(); i++) 
		{
			ROS_INFO("Position of obstacle base_link frame is (x:%f y:%f)\n", x_vector[i], y_vector[i]);

			base_laser_link_pose.pose.pose.position.x = x_vector[i];
			base_laser_link_pose.pose.pose.position.y = y_vector[i];
			base_laser_link_pose.pose.pose.position.z = 0;
			
			//_________________ Transform from Laser Frame to Map frame _________________
			tf2::doTransform(base_laser_link_pose, map_pose, transformStamped); // all datatype must be Stamped or notStamped

			ROS_INFO("Position transformed is (x:%f y:%f z:%f)\n", map_pose.pose.pose.position.x, map_pose.pose.pose.position.y, map_pose.pose.pose.position.z);
			// ROS_INFO("Orientation transformed is (x:%f y:%f z:%f w:%f)\n", map_pose.pose.pose.orientation.x, map_pose.pose.pose.orientation.y, map_pose.pose.pose.orientation.z, map_pose.pose.pose.orientation.w);
		}

	}

	void draw_marker() 
	{
		
		ROS_INFO("Starting draw_marker");
		_feedback.status = 5;
		_as.publishFeedback(_feedback); // one feedback (not keep publishing in while)

		while (ros::ok()) 
		{

			visualization_msgs::Marker marker;
			marker.header.frame_id = "/map";
			marker.header.stamp = ros::Time();
			marker.ns = "basic_shapes";
			marker.id = 0;
			marker.type = visualization_msgs::Marker::CYLINDER;
			marker.action = visualization_msgs::Marker::ADD;
			marker.pose.position.x = map_pose.pose.pose.position.x;
			marker.pose.position.y = map_pose.pose.pose.position.y;
			marker.pose.position.z = map_pose.pose.pose.position.z;
			marker.pose.orientation.x = 0.0;
			marker.pose.orientation.y = 0.0;
			marker.pose.orientation.z = 0.0;
			marker.pose.orientation.w = 0.0;
			marker.scale.x = 1;
			marker.scale.y = 1;
			marker.scale.z = 0.5;
			marker.color.a = 1.0; // Don't forget to set the alpha!
			marker.color.r = 1.0;
			marker.color.g = 0.0;
			marker.color.b = 0.0;

			vis_pub.publish(marker);
		}

		ROS_INFO("Ending draw_marker");
		tf_sub.shutdown();
	}	


};

int main (int argc, char **argv) 
{
	ros::init(argc, argv, "count_until_server");
	ROS_INFO("About to start server");
	CountUntilServer server;
	ros::spin();
}
