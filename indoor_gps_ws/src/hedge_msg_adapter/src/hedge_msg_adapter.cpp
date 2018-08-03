/*
  hedge_msg_adapter
  ROS message adapter to convert Marvelmind Indoor GPS messages into eligible
  message types for use with robot_localization

                                   .     .
                                .  |\-^-/|  .
                               /| } O.=.O { |\
                              /´ \ \_ ~ _/ / `\
                            /´ |  \-/ ~ \-/  | `\
                            |   |  /\\ //\  |   |
                             \|\|\/-""-""-\/|/|/
                                     ______/ /
                                     '------'
                       _   _        _  ___
             _ __  ___| |_| |_ _  _| ||   \ _ _ __ _ __ _ ___ _ _
            | '  \/ -_)  _| ' \ || | || |) | '_/ _` / _` / _ \ ' \
            |_|_|_\___|\__|_||_\_, |_||___/|_| \__,_\__, \___/_||_|
                               |__/                 |___/
            -------------------------------------------------------
                           github.com/methylDragon

  Copyright (c) 2018, methylDragon
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
  * Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.
  * Neither the name of nor the names of its contributors may be used to
  endorse or promote products derived from this software without specific
  prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.

  References:
  http://wiki.ros.org/roscpp/Overview/Parameter%20Server

*/

#include "ros/ros.h"
#include "marvelmind_nav/hedge_imu_fusion.h"
#include "marvelmind_nav/hedge_pos_ang.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "tf/transform_broadcaster.h"

class hedge_msg_adapter_node
{
public:
  hedge_msg_adapter_node() // Class constructor
  {
    ros::NodeHandle nh_; // Public nodehandle for pub-sub
    ros::NodeHandle nh_private_("~"); // Private nodehandle for handling parameters

    // Init subscribers
    imu_fusion_sub_ = nh_.subscribe("hedge_imu_fusion", 10, &hedge_msg_adapter_node::imu_fusion_callback, this);
    pos_ang_sub_ = nh_.subscribe("hedge_pos_ang", 10, &hedge_msg_adapter_node::pos_ang_callback, this);

    // Init publishers
    hedge_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("hedge_pose", 10, false);
    hedge_imu_pub_ = nh_.advertise<sensor_msgs::Imu>("hedge_imu", 10, false);

    // You must provide the static transforms for these in a launch file!
    imu_out_.header.frame_id = "beacon_imu_link";
    pose_out_.header.frame_id = "beacon_map";

    // Init covariances grabbed from the parameter server
    init_covariances(nh_private_);
  }

  void imu_fusion_callback(const marvelmind_nav::hedge_imu_fusion::ConstPtr& imu_fusion_msg)
  {
    // Populate header
    imu_out_.header.stamp = ros::Time::now();

    // Populate orientation data
    imu_out_.orientation.x = imu_fusion_msg->qx;
    imu_out_.orientation.y = imu_fusion_msg->qy;
    imu_out_.orientation.z = imu_fusion_msg->qz;
    imu_out_.orientation.w = imu_fusion_msg->qw;

    // Populate angular velocity data
    imu_out_.angular_velocity.x = imu_fusion_msg->vx;
    imu_out_.angular_velocity.y = imu_fusion_msg->vy;
    imu_out_.angular_velocity.z = imu_fusion_msg->vz;

    // Populate linear acceleration data
    imu_out_.linear_acceleration.x = imu_fusion_msg->ax;
    imu_out_.linear_acceleration.y = imu_fusion_msg->ay;
    imu_out_.linear_acceleration.z = imu_fusion_msg->az;

    // Publish the sensor_msgs/Imu message
    hedge_imu_pub_.publish(imu_out_);
  }

  void pos_ang_callback(const marvelmind_nav::hedge_pos_ang::ConstPtr& pos_ang_msg)
  {
    // Populate header
    pose_out_.header.stamp = ros::Time::now();

    // Populate position data
    pose_out_.pose.pose.position.x = pos_ang_msg->x_m;
    pose_out_.pose.pose.position.y = pos_ang_msg->y_m;
    pose_out_.pose.pose.position.z = pos_ang_msg->z_m;

    // Populate orientation data
    pose_out_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(pos_ang_msg->angle);

    // Publish the geometry_msgs/PoseWithCovarianceStamped message
    hedge_pose_pub_.publish(pose_out_);
  }

  // Handy function for initialising covariance matrices from parameters
  void init_covariances(ros::NodeHandle &nh_private_)
  {
    // Create the vectors to store the covariance matrix arrays
    std::vector<double> orientation_covar;
    std::vector<double> ang_vel_covar;
    std::vector<double> linear_accel_covar;
    std::vector<double> pose_covar;

    // Grab the parameters and populate the vectors
    nh_private_.getParam("imu_orientation_covariance", orientation_covar);
    nh_private_.getParam("imu_angular_velocity_covariance", ang_vel_covar);
    nh_private_.getParam("imu_linear_acceleration_covariance", linear_accel_covar);
    nh_private_.getParam("pose_covariance", pose_covar);

    // Iterate through each vector and populate the respective message fields
    for (int i = 0; i < orientation_covar.size(); i++)
      imu_out_.orientation_covariance[i] = orientation_covar.at(i);

    for (int i = 0; i < ang_vel_covar.size(); i++)
      imu_out_.angular_velocity_covariance[i] = ang_vel_covar.at(i);

    for (int i = 0; i < linear_accel_covar.size(); i++)
      imu_out_.linear_acceleration_covariance[i] = linear_accel_covar.at(i);

    for (int i = 0; i < pose_covar.size(); i++)
      pose_out_.pose.covariance[i] = pose_covar.at(i);
  }

protected:
  // Subscriber objects
  ros::Subscriber imu_fusion_sub_;
  ros::Subscriber pos_ang_sub_;

  // Publisher objects
  ros::Publisher hedge_pose_pub_;
  ros::Publisher hedge_imu_pub_;

  // Message objects
  sensor_msgs::Imu imu_out_;
  geometry_msgs::PoseWithCovarianceStamped pose_out_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hedge_msg_adapter");

  hedge_msg_adapter_node adapter;

  ros::spin();
}
