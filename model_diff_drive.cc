#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <iostream>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Time.hh>

// For Ach
#include <errno.h>
#include <fcntl.h>
#include <assert.h>
#include <unistd.h>
#include <pthread.h>
#include <ctype.h>
#include <stdbool.h>
#include <math.h>
#include <inttypes.h>
#include <ach.h>
#include <string.h>

// ach channels
ach_channel_t chan_diff_drive_ref;      // hubo-ach
ach_channel_t chan_time;

int debug = 0;
double H_ref[2] = {};
double ttime = 0.0;
namespace gazebo
{   
  class ModelDiffDrive : public ModelPlugin
  {

    // Function is called everytime a message is received.
//void cb(gazebo::msgs::Image &_msg)
//void cb(const std::string& _msg)
//void cb(gazebo::msgs::ImageStamped &_msg)
//void cb(ConstWorldStatisticsPtr &_msg)
//void cb(const std::string& _msg)
    public: void cb(ConstWorldStatisticsPtr &_msg)
    {
       gazebo::common::Time simTime  = gazebo::msgs::Convert(_msg->sim_time());
       //size_t size;
       double ttime = simTime.Double();
       //ach_put(&chan_time, _msg->image().data().c_str() , _msg->image().data().size());

    }


    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) 
    {

      // Open Ach channel
        /* open ach channel */
        memset( &H_ref,   0, sizeof(H_ref));
        int r = ach_open(&chan_diff_drive_ref, "robot-diff-drive" , NULL);
        assert( ACH_OK == r );
        ach_put(&chan_diff_drive_ref, &H_ref , sizeof(H_ref));

        
    
        memset( &ttime,   0, sizeof(ttime));
        r = ach_open(&chan_time, "robot-time" , NULL);
        assert( ACH_OK == r );
        ach_put(&chan_time, &ttime , sizeof(ttime));


      // Store the pointer to the model
      this->model = _parent;

      // Get then name of the parent model
//      std::string modelName = _sdf->GetParent()->Get<std::string>("name");

      // Get the world name.
//      std::string worldName = _sdf->GetName();
     this->world = physics::get_world("default");


      // Load parameters for this plugin
      if (this->LoadParams(_sdf))
      {
        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&ModelDiffDrive::OnUpdate, this));
      }

      // subscribe to thread
//      gazebo::transport::NodePtr node(new gazebo::transport::Node());
//      node->Init();
//      gazebo::transport::SubscriberPtr sub = node->Subscribe("/gazebo/default/world_stats", cb);
    }

    public: bool LoadParams(sdf::ElementPtr _sdf) 
    {
      if (this->FindJointByParam(_sdf, this->left_wheel_joint_,
                             "left_wheel_hinge") &&
          this->FindJointByParam(_sdf, this->right_wheel_joint_,
                             "right_wheel_hinge"))
        return true;
      else
        return false;
    }

    public: bool FindJointByParam(sdf::ElementPtr _sdf,
                                  physics::JointPtr &_joint,
                                  std::string _param)
    {
      if (!_sdf->HasElement(_param))
      {
        gzerr << "param [" << _param << "] not found\n";
        return false;
      }
      else
      {
        _joint = this->model->GetJoint(
          _sdf->GetElement(_param)->GetValueString());

        if (!_joint)
        {
          gzerr << "joint by name ["
                << _sdf->GetElement(_param)->GetValueString()
                << "] not found in model\n";
          return false;
        }
      }
      return true;
    }

    // Called by the world update start event
    public: void OnUpdate()
    {

      // Get Ach chan data
    size_t fs;
    int r = ach_get( &chan_diff_drive_ref, &H_ref, sizeof(H_ref), &fs, NULL, ACH_O_LAST );

    if(ACH_OK != r | ACH_STALE_FRAMES != r | ACH_MISSED_FRAME != r) {
                if(debug) {
                  //      printf("Ref ini r = %s\n",ach_result_to_string(r));}
                   printf("ref int r = %i \n\r",r);
		 }
        }
        else{   assert( sizeof(H_ref) == fs ); }

      this->left_wheel_joint_->SetMaxForce(0, 2);
      this->right_wheel_joint_->SetMaxForce(0, 2);
      this->right_wheel_joint_->SetVelocity(0, H_ref[0]);
      this->left_wheel_joint_->SetVelocity(0, H_ref[1]);

      ttime = this->world->GetSimTime().Double();
      //printf("- %f\n\r", ttime);

      ach_put(&chan_time, &ttime, sizeof(ttime));
//      double tmp = this->GetSimTime().double();
//      this->right_wheel_joint_->SetForce(0, H_ref[0]);
//      this->left_wheel_joint_->SetForce(0, H_ref[1]);

     //this->left_wheel_joint_->SetMaxForce(0, 10);
      //this->right_wheel_joint_->SetMaxForce(0, 10);
      //this->left_wheel_joint_->SetForce(0, -0.5);
//      this->left_wheel_joint_->SetForce(0, 0.2);
      //this->right_wheel_joint_->SetVelocity(0,0.5);
//      this->right_wheel_joint_->SetForce(0, -0.2);

      

    }

    // Pointer to the model
    private: physics::ModelPtr model;
    private: physics::WorldPtr world;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    private: physics::JointPtr left_wheel_joint_;
    private: physics::JointPtr right_wheel_joint_;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelDiffDrive)
}
