#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_srvs/Trigger.h"

using namespace std;

#define KICK_COUNT_MAX	125

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
				gzerr << "[model_push] Please specify a namespace.\n";
			kP_xy = getValueFromSdf("kP_xy");
			kP_w = getValueFromSdf("kP_w");
			maxF_xy = getValueFromSdf("maxF_xy");
			maxF_w = getValueFromSdf("maxF_w");

			node_handle = ros::NodeHandle(robot_name);
			gzmsg << "[model_push] Subscribing to " << ("/" + robot_name + "/command") << "\n";
			command_sub = node_handle.subscribe("/" + robot_name + "/command", 1, &SoccerDrive::CommandCallback, this);
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
				gzerr << "[model_push] Please specify " << name << ".\n";
				return 0;
			}
		}

		int sgn(double val)
		{
			return (0 < val) - (val < 0);
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
				// Apply forces to the model (using P control) to acheive the commanded linear and angular velocities.
				math::Vector3 linearVel = link->GetWorldLinearVel();
				math::Vector3 angularVel = link->GetWorldAngularVel();

				double fx = SoccerDrive::saturate((command_msg.linear.x - linearVel.x)*kP_xy, maxF_xy);
				double fy = SoccerDrive::saturate((command_msg.linear.y - linearVel.y)*kP_xy, maxF_xy);
				double fw = SoccerDrive::saturate((command_msg.angular.z - angularVel.z)*kP_w, maxF_w);

				link->AddForce(math::Vector3(fx, fy, 0));
				link->AddTorque(math::Vector3(0, 0, fw));
			}

			// Kick the ball!
			if (kick) {
				kicker_joint->SetForce(0, 15);

				kick_count++;

				// Find out when to release kicker
				if (kick_count == KICK_COUNT_MAX)
				{
					kick_count = 0;
					kick = false;
				}

			} else {
				kicker_joint->SetForce(0, -15);
			}
		}

		void CommandCallback(const geometry_msgs::Twist msg)
		{
			command_msg = msg;

			// convert from degrees to radians
			command_msg.angular.z = command_msg.angular.z*M_PI/180.0;
		}

		bool KickSrv(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
		{
			kick = !kick;
			res.success = kick;

			return true;
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
		geometry_msgs::Twist command_msg;
		bool kick;
		unsigned int kick_count;
		ros::ServiceServer kick_srv;
		double kP_xy;
		double kP_w;
		double maxF_xy;
		double maxF_w;
	};

	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(SoccerDrive)
}
