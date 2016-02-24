#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"

namespace gazebo
{
	class ModelPush : public ModelPlugin
	{
	public: 
		void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
		{
			// Store the pointers to the model and to the sdf
			model = _parent;
			sdf_pointer = _sdf;

			// Connect to ROS
			if (sdf_pointer->HasElement("namespace"))
				robot_name = sdf_pointer->GetElement("namespace")->Get<std::string>();
			else
				gzerr << "[model_push] Please specify a namespace.\n";
			node_handle = ros::NodeHandle(robot_name);
			gzmsg << "[model_push] Subscribing to " << ("/" + robot_name + "/command") << "\n";
			command_sub = node_handle.subscribe("/" + robot_name + "/command", 1, &ModelPush::CommandCallback, this);

			// Listen to the update event. This event is broadcast every
			// simulation iteration.
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ModelPush::OnUpdate, this, _1));
		}

		// Called by the world update start event
		void OnUpdate(const common::UpdateInfo & /*_info*/)
		{
			if(this->model->GetWorldPose().pos.z < .01) //If robot is more than 1 cm off the ground, it is probably going over obstacles and that's bad.
			{
				// Apply the commanded linear and angular velocities to the model.
				this->model->SetLinearVel(math::Vector3(command_msg.x, command_msg.y, 0));
				this->model->SetAngularVel(math::Vector3(0, 0, command_msg.z));
			}
		}

		void CommandCallback(const geometry_msgs::Vector3 msg)
		{
			command_msg = msg;
		}

	private:
		// Pointer to the model
		sdf::ElementPtr sdf_pointer;
		std::string robot_name;
		physics::ModelPtr model;
		event::ConnectionPtr updateConnection;
		ros::NodeHandle node_handle;
		ros::Subscriber command_sub;
		geometry_msgs::Vector3 command_msg;
	};

	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
