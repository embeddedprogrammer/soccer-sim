#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Pose2D.h"
#include "soccerref/GameState.h"

#define FIELD_WIDTH 		3.53  // in meters
#define FIELD_HEIGHT 		2.39

// the ball goes back to home after this threshold
#define GOAL_THRESHOLD		(FIELD_WIDTH/2 + 0.05)

namespace gazebo
{
	class SoccerBall : public ModelPlugin
	{
	public:
		void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
		{
			// Store the pointers to the model and to the sdf
			model = _parent;
			sdf_pointer = _sdf;
			link = model->GetLink("ball");

			// Connect to ROS
			if (sdf_pointer->HasElement("namespace"))
				ball_name = sdf_pointer->GetElement("namespace")->Get<std::string>();
			else
				gzerr << "[Soccer ball plugin] Please specify a namespace.\n";

			if (sdf_pointer->HasElement("friction"))
				friction = sdf_pointer->GetElement("friction")->Get<double>();
			else
				gzerr << "[Soccer ball plugin] Please specify friction.\n";

			// Subcribe to ball command topic
			gzmsg << "[Soccer ball plugin] Subscribing to " << ("/" + ball_name + "/command") << "\n";
			node_handle = ros::NodeHandle(ball_name);
			command_sub = node_handle.subscribe("/" + ball_name + "/command", 1, &SoccerBall::CommandCallback, this);

			// Publish ball location
			gzmsg << "[Soccer ball plugin] Publishing to /ball/truth\n";
			ball_pub = node_handle.advertise<geometry_msgs::Pose2D>("/ball/truth", 5);

			// Subcribe to game state
			gzmsg << "[Soccer ball plugin] Subscribing to /game_state\n";
			game_state_sub = node_handle.subscribe("/game_state", 1, &SoccerBall::GameStateCallback, this);
			game_state_msg = soccerref::GameState();
			game_state_msg.reset_field = false;

			// Listen to the update event. This event is broadcast every simulation iteration.
			updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&SoccerBall::OnUpdate, this, _1));
			newMessage = false;
		}

		void resetBall()
		{
			model->SetWorldPose(math::Pose(0, 0, 0.2, 0, 0, 0));
		}

		// Called by the world update start event
		void OnUpdate(const common::UpdateInfo & /*_info*/)
		{
			math::Vector3 linearVel = link->GetWorldLinearVel();
			math::Vector3 force = -linearVel*friction;
			link->AddForce(force);

			if(game_state_msg.reset_field)
				resetBall();
			else if(newMessage)
			{
				model->SetWorldPose(math::Pose(command_msg.x, command_msg.y, command_msg.z, 0, 0, 0));
				newMessage = false;

				// Clear the velocity when the user places the ball
				link->SetLinearVel(math::Vector3(0, 0, 0));
				link->SetForce(math::Vector3(0, 0, 0));
			}
			else if (model->GetWorldPose().pos.x < -GOAL_THRESHOLD)
				SoccerBall::resetBall();
			else if (model->GetWorldPose().pos.x > GOAL_THRESHOLD)
				SoccerBall::resetBall();

			// Publish true ball location
			geometry_msgs::Pose2D truth;
			truth.x = model->GetWorldPose().pos.x;
			truth.y = model->GetWorldPose().pos.y;
			truth.theta = 0;
			ball_pub.publish(truth);
		}

		void CommandCallback(const geometry_msgs::Vector3 msg)
		{
			command_msg = msg;
			newMessage = true;
		}

		void GameStateCallback(const soccerref::GameState msg)
		{
			game_state_msg = msg;
		}

	private:
		// Pointer to the model
		sdf::ElementPtr sdf_pointer;
		double friction;
		physics::LinkPtr link;
		std::string ball_name;
		physics::ModelPtr model;
		event::ConnectionPtr updateConnection;
		ros::NodeHandle node_handle;
		ros::Subscriber command_sub;
		ros::Subscriber game_state_sub;
		ros::Publisher ball_pub;
		geometry_msgs::Vector3 command_msg;
		soccerref::GameState game_state_msg;
		bool newMessage;
	};

	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(SoccerBall)
}
