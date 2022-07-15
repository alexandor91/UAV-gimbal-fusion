// Copyright 2022, Xueyang Kang, KU Leuven
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <math.h>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
// #include <tf/transform_datatypes.h>
// #include <tf/transform_broadcaster.h>
#include <mutex>
#include <queue>
#include <thread>
#include <iostream>
#include <string>
#include "particle_filter.h"

#include <deque>
#include <fstream>
#include <numeric>
#include <queue>
#include <string>
#include <memory>
#include <vector>


float std_imu [];
std_imu[0] = 0.01;
std_imu[1] = 0.01;

void percepAngleCallback(const  std_msgs::float32::ConstPtr& angs_input)
{
	prediction(0.0, std_imu, pitch_rate, roll_rate);
	updateWeights(angs_input);
	resample();
}
void imuCallback(const  std_msgs::float32::ConstPtr& angs_input)
{
	if (is_initialized == false)
	    init();
	else
	{
		pitch_rate = msg->angular_velocity[0];
		roll_rate = msg->angular_velocity[1];
		prediction(0.0, std_imu, pitch_rate, roll_rate);
	}
}																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																					

int main(int argc, char **argv)
{
	ros::init(argc, argv, "sphParticleFilter");
	ros::NodeHandle nh;

	float paf_num = 0;
	float planeRes = 0;
	
	nh.param<float>("particle number", paf_num, 100);



	// ros::Subscriber subSkylineAngles = nh.subscribe<std_msgs::float32>("/skyline_angles", 10, skylineAngsCallback);

	ros::Subscriber subPerceptionAngles = nh.subscribe<std_msgs::float32>("/ground_plane_angles", 10, percepAngleCallback);

	ros::Subscriber subImuAngles = nh.subscribe<std_msgs::float32>("/filtered_imu", 10, imuCallback);

    double angles[];
	double roll_sum = 0.0;
	double pitch_sum = 0.0;
    std_msgs::float32 msg;
	double resolution = particles[0].angle_res;

	ros::Publisher chatter_pub = nh.advertise<std_msgs::float32>("/fusion_output", 10);

    for (int i = 0; i < particles.size(); i ++)
	{	
		roll_sum += particles[i].roll_idx ;
		pitch_sum += particles[i].pitch_idx ;
	}
	msg.data = roll_sum/particles.size()*resolution;
	msg.data = pitch_sum/particles.size()*resolution;

	chatter_pub.publish(msg);

	ros::spin();

	return 0;
}