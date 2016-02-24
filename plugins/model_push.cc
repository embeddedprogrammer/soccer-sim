#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Vector3.h"

namespace gazebo
{
	class ModelPush : public ModelPlugin
	{
	public: 
		void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
		{
			// Store the pointer to the model
			this->model = _parent;

			/*
			 * Connect the Plugin to the Robot and Save pointers to the various elements in the simulation
			 */
//			if (_sdf->HasElement("namespace"))
//				namespace_ = _sdf->GetElement("namespace")->Get<std::string>();
//			else
//				gzerr << "[gazebo_aircraft_forces_and_moments] Please specify a namespace.\n";
//			node_handle_ = new ros::NodeHandle(namespace_);
			node_handle = ros::NodeHandle("robot");
			command_sub = node_handle.subscribe("/command", 1, &ModelPush::CommandCallback, this);

			// Listen to the update event. This event is broadcast every
			// simulation iteration.
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ModelPush::OnUpdate, this, _1));
		}

		// Called by the world update start event
		void OnUpdate(const common::UpdateInfo & /*_info*/)
		{
			// Apply the commanded linear and angular velocities to the model.
			this->model->SetLinearVel(math::Vector3(command_msg.x, command_msg.y, 0));
			this->model->SetAngularVel(math::Vector3(0, 0, command_msg.z));
		}

		void CommandCallback(const geometry_msgs::Vector3 msg)
		{
			command_msg = msg;
		}

	private:
		// Pointer to the model
		physics::ModelPtr model;
		event::ConnectionPtr updateConnection;
		ros::NodeHandle node_handle;
		ros::Subscriber command_sub;
		geometry_msgs::Vector3 command_msg;
	};

	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
