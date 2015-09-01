/*
 * Copyright 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/common/SingletonT.hh>
#include <gazebo/common/Timer.hh>
#include <gazebo/util/UtilTypes.hh>
#include <gazebo/common/common.hh>
#include <iostream>


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

// ach channels
ach_channel_t chan_vid_chan;      // hubo-ach


int i = 0;
/////////////////////////////////////////////////
// Function is called everytime a message is received.
//void cb(gazebo::msgs::Image &_msg)
//void cb(const std::string& _msg)
//void cb(gazebo::msgs::ImageStamped &_msg)
//void cb(ConstWorldStatisticsPtr &_msg)
//void cb(const std::string& _msg)
void cb(ConstImageStampedPtr &_msg)
{
  size_t size;
 ach_put(&chan_vid_chan, _msg->image().data().c_str() , _msg->image().data().size());

}

void cb2(ConstWorldStatisticsPtr &_msg)
    {
       gazebo::common::Time simTime  = gazebo::msgs::Convert(_msg->sim_time());
       //size_t size;
       double ttime = simTime.Double();
       //ach_put(&chan_time, _msg->image().data().c_str() , _msg->image().data().size());
//       printf("%f\n\r",ttime);
    }

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
        //gazebo::transport::clear_buffers();
        char robot_vid[320*240*3];
	memset( &robot_vid,   0, sizeof(robot_vid));


        /* open ach channel */
	int r = ach_open(&chan_vid_chan, "robot-vid-chan" , NULL);
	assert( ACH_OK == r );



  // Load gazebo
  printf("%i\n\r",-1);
  gazebo::load(_argc, _argv);

  printf("%i\n\r",0);
  gazebo::run();

  printf("%i\n\r",1);
  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  printf("%i\n\r",2);
  // Listen to Gazebo world_stats topic
  gazebo::transport::SubscriberPtr sub = node->Subscribe("/gazebo/default/DiffDrive/d_diff_drive_robot/camera/link/camera/image", cb);
//  gazebo::transport::SubscriberPtr sub = node->Subscribe("/gazebo/default/DiffDrive/d_diff_drive_robot/camera/link/camera/image", cb);

      gazebo::transport::NodePtr node2(new gazebo::transport::Node());
      node2->Init();
      gazebo::transport::SubscriberPtr sub2 = node2->Subscribe("/gazebo/default/world_stats", cb2);

//      gazebo::transport::NodePtr node3(new gazebo::transport::Node());
//      node3->Init();
//      gazebo::transport::SubscriberPtr sub3 = node3->Subscribe("/gazebo/default/diagnostics", cb3);


  printf("%i\n\r",3);
  // Busy wait loop...replace with your own code as needed.
  while (true)
    gazebo::common::Time::MSleep(100);

  // Make sure to shut everything down.
  gazebo::transport::fini();
}
