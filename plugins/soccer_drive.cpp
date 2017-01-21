#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_srvs/Trigger.h"
#include "soccerref/GameState.h"
#include <math.h>

using namespace std;

#define KICK_TIME_MS 125

namespace gazebo
{
	class SoccerDrive : public ModelPlugin
	{
	public:
		void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
		{
			// Store the pointers to the model and to the sdf
			model = _parent;
			sdf_pointer = _sdf;
			link = model->GetLink("base_link");
			kicker_joint = model->GetJoint("kicker_joint");

			// Connect to ROS
			if (sdf_pointer->HasElement("namespace"))
				robot_name = sdf_pointer->GetElement("namespace")->Get<std::string>();
			else
				gzerr << "[soccer_drive] Please specify a namespace.\n";
			kP_xy = getValueFromSdf("kP_xy");
			kP_w = getValueFromSdf("kP_w");
			maxF_xy = getValueFromSdf("maxF_xy");
			maxF_w = getValueFromSdf("maxF_w");
			F_height = getValueFromSdf("F_height");
			friction = getValueFromSdf("friction");
			simulator_step_size = model->GetWorld()->GetPhysicsEngine()->GetMaxStepSize();

			node_handle = ros::NodeHandle(robot_name);
			gzmsg << "[soccer_drive] Subscribing to " << ("/" + robot_name + "/command") << "\n";
			command_sub = node_handle.subscribe("/" + robot_name + "/command", 1, &SoccerDrive::CommandCallback, this);
			game_state_sub = node_handle.subscribe("/game_state", 1, &SoccerDrive::GameStateCallback, this);
			game_state_msg = soccerref::GameState();
			kick_srv = node_handle.advertiseService("/" + robot_name + "/kick", &SoccerDrive::KickSrv, this);

			// Init kick counter
			kick_count = 0;

			// Listen to the update event. This event is broadcast every
			// simulation iteration.
			updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&SoccerDrive::OnUpdate, this, _1));
		}

		double getValueFromSdf(std::string name)
		{
			if (sdf_pointer->HasElement(name))
				return sdf_pointer->GetElement(name)->Get<double>();
			else
			{
				gzerr << "[soccer_drive] Please specify " << name << ".\n";
				return 0;
			}
		}

		int sgn(double val)
		{
			return (0 < val) - (val < 0);
		}

		void saturate2(double& Fx, double& Fy, double Fmax)
		{
			double F = sqrt(Fx*Fx + Fy*Fy);
			if(fabs(F) > Fmax)
			{
				Fx *= Fmax / F;
				Fy *= Fmax / F;
			}
		}		

		double saturate(double F, double Fmax)
		{
			if(fabs(F) > Fmax)
				return Fmax*SoccerDrive::sgn(F);
			else
				return F;
		}

		// Called by the world update start event
		void OnUpdate(const common::UpdateInfo & /*_info*/)
		{
			if(link->GetWorldPose().pos.z < .02) //If robot is more than 2 cm off the ground, wheels no longer have any traction
			{
				// Apply forces to the model (using P control) to achieve the commanded linear and angular velocities.
				math::Vector3 linearVel = link->GetWorldLinearVel();
				math::Vector3 angularVel = link->GetWorldAngularVel();
				double Fx = (command_msg.linear.x - linearVel.x)*kP_xy;
				double Fy = (command_msg.linear.y - linearVel.y)*kP_xy;
				double Fw = (command_msg.angular.z - angularVel.z)*kP_w;
				saturate2(Fx, Fy, maxF_xy);
				Fw = saturate(Fw, maxF_w);

				// Note: We are applying the force at height F_height. This seems a bit easier then creating a child link
				// at a different location and then trying to figure out how to access the child link.
				link->AddForceAtRelativePosition(math::Vector3(Fx, Fy, 0), math::Vector3(0, 0, F_height));
				link->AddTorque(math::Vector3(0, 0, Fw));
			}

			// Artificially add friction
			math::Vector3 vel = link->GetWorldLinearVel();
			math::Vector3 friction_force = -vel*friction;
			link->AddForce(friction_force);

			// Kick the ball!
			if (kick)
			{
				// Note: Setting and adding force are the same thing if only executed once per time step.
				// If AddForce is called more than once in a time step, the two forces will be added together.
				kicker_joint->SetForce(0, 15);

				kick_count++;

				// Find out when to release kicker
				if (kick_count >= KICK_TIME_MS / (1000.0 * simulator_step_size))
				{
					kick_count = 0;
					kick = false;
				}

			}
			else
			{
				kicker_joint->SetForce(0, -15);
			}
		}

		void CommandCallback(const geometry_msgs::Twist msg)
		{
			command_msg = msg;

			// Since the actual robot hardware receives velocity commands in
			// a robocentric coordinate frame (i.e., positive vel always goes
			// forward, regardless of orientation), that means the velocity
			// commands that are coming from the robot's controller are in a
			// robocentric frame. Gazebo, however, expects global coordinates
			// so if the team is away, we need to flip the vel_cmds to be in
			// Gazebo's global coordinate frame.
			// Also, do this if we have swapped sides for the second half.
			if (!isHome() ^ game_state_msg.second_half)
			{
		        command_msg.linear.x *= -1.0;
		        command_msg.linear.y *= -1.0;				
			}

			// convert from degrees to radians
			command_msg.angular.z *= M_PI/180.0;
		}

		void GameStateCallback(const soccerref::GameState msg)
		{
			game_state_msg = msg;
		}		

		bool KickSrv(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
		{
			kick = !kick;
			res.success = kick;

			return true;
		}

		bool isHome()
		{
			// does robot_name have the string 'home' in it?
			return robot_name.find("home") != string::npos;
		}

	private:
		// Pointer to the model
		sdf::ElementPtr sdf_pointer;
		std::string robot_name;
		physics::ModelPtr model;
		physics::LinkPtr link;
		physics::JointPtr kicker_joint;
		event::ConnectionPtr updateConnection;
		ros::NodeHandle node_handle;
		ros::Subscriber command_sub;
		ros::Subscriber kick_sub;
		ros::Subscriber game_state_sub;
		geometry_msgs::Twist command_msg;
		soccerref::GameState game_state_msg;
		bool kick;
		unsigned int kick_count;
		ros::ServiceServer kick_srv;
		double kP_xy;
		double kP_w;
		double friction;
		double maxF_xy;
		double maxF_w;
		double F_height;
		double simulator_step_size;
	};

	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(SoccerDrive)
}
