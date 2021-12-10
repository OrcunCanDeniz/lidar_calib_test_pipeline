/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
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
 ********************
 *  v1.0: amc-nu (abrahammonrroy@yahoo.com)
 *
 * multi_lidar_calibrator.cpp
 *
 *  Created on: Feb 27, 2018
 */

#include "multi_lidar_calibrator.h"


void ROSMultiLidarCalibratorApp::PublishCloud(const ros::Publisher& in_publisher, pcl::PointCloud<PointT>::ConstPtr in_cloud_to_publish_ptr)
{
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
  cloud_msg.header.frame_id = parent_frame_;
  in_publisher.publish(cloud_msg);
}

void ROSMultiLidarCalibratorApp::PointsCallback(const sensor_msgs::PointCloud2::ConstPtr &in_parent_cloud_msg,
                                                  const sensor_msgs::PointCloud2::ConstPtr &in_child_cloud_msg)
{
  parent_frame_ = in_parent_cloud_msg->header.frame_id;
  child_frame_ = in_child_cloud_msg->header.frame_id;

  std::cout<< parent_frame_ << std::endl;
  std::cout<< child_frame_ << std::endl;

  UpdateCurrentGuessWithLookup();
  pcl::PointCloud<PointT>::Ptr in_parent_cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr in_child_cloud(new pcl::PointCloud<PointT>);

  pcl::PointCloud<PointT>::Ptr parent_cloud_filtered (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr child_cloud_filtered (new pcl::PointCloud<PointT>);

  pcl::fromROSMsg(*in_parent_cloud_msg, *in_parent_cloud);
  pcl::fromROSMsg(*in_child_cloud_msg, *in_child_cloud);



  if(downsample_parent_points_)
    DownsampleCloud(in_parent_cloud, parent_cloud_filtered, voxel_size_);

  if (downsample_child_points_)
    DownsampleCloud(in_child_cloud, child_cloud_filtered, voxel_size_);

  // Initializing Normal Distributions Transform (NDT).
  pcl::NormalDistributionsTransform<PointT, PointT> ndt;

  ndt.setTransformationEpsilon(ndt_epsilon_);
  ndt.setStepSize(ndt_step_size_);
  ndt.setResolution(ndt_resolution_);

  ndt.setMaximumIterations(ndt_iterations_);



  if (downsample_child_points_)
    ndt.setInputSource(child_cloud_filtered);
  else
    ndt.setInputSource(in_child_cloud);


  if (downsample_parent_points_)
    ndt.setInputTarget(parent_cloud_filtered);
  else
    ndt.setInputTarget(in_parent_cloud);


  pcl::PointCloud<PointT>::Ptr output_cloud(new pcl::PointCloud<PointT>);

  ndt.align(*output_cloud, current_guess_);

  std::cout << "Normal Distributions Transform converged:" << ndt.hasConverged ()
            << " score: " << ndt.getFitnessScore () << " prob:" << ndt.getTransformationProbability() << std::endl;
  std_msgs::String stat_msg;
  stat_msg.data = std::string("~~~ Converged: ") + std::to_string(ndt.hasConverged ())  +
                          std::string(" score: ") + std::to_string(ndt.getFitnessScore ()) + std::string(" prob:") +  std::to_string(ndt.getTransformationProbability());
  calibration_stat_pub.publish(stat_msg);

  std::cout << "transformation from " << child_frame_ << " to " << parent_frame_ << std::endl;

  // Transforming unfiltered, input cloud using found transform.
  pcl::transformPointCloud (*in_child_cloud, *output_cloud, ndt.getFinalTransformation());

  current_guess_ = ndt.getFinalTransformation();

  bridge->toEvalSrv(current_guess_);
  
  Eigen::Matrix3f rotation_matrix = current_guess_.block(0,0,3,3);
  Eigen::Vector3f translation_vector = current_guess_.block(0,3,3,1);
  std::cout << "This transformation can be replicated using:" << std::endl;
  std::cout << "<node pkg=\"tf\" type=\"static_transform_publisher\" name=\""<< parent_frame_.substr(7,6) << "_to_" << child_frame_.substr(7,6) <<"\" args=\""<< translation_vector.transpose()
            << " " << rotation_matrix.eulerAngles(2,1,0).transpose() << " " << parent_frame_
            << " " << child_frame_ << " 10\"/>" << std::endl;

  std::cout << "Corresponding transformation matrix:" << std::endl
            << std::endl << current_guess_ << std::endl << std::endl;

  PublishCloud(calibrated_cloud_publisher_, output_cloud);
  // timer end
  //auto end = std::chrono::system_clock::now();
  //auto usec = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
  //std::cout << "time: " << usec / 1000.0 << " [msec]" << std::endl;

}

void ROSMultiLidarCalibratorApp::InitialPoseCallback(geometry_msgs::PoseWithCovarianceStamped::ConstPtr in_initialpose)
{
  ROS_INFO("[%s] Initial Pose received.", __APP_NAME__);
    UpdateCurrentGuessWithLookup();
}

void ROSMultiLidarCalibratorApp::DownsampleCloud(pcl::PointCloud<PointT>::ConstPtr in_cloud_ptr,
                                                 pcl::PointCloud<PointT>::Ptr out_cloud_ptr,
                                                 double in_leaf_size)
{
  pcl::VoxelGrid<PointT> voxelized;
  voxelized.setInputCloud(in_cloud_ptr);
  voxelized.setLeafSize((float)in_leaf_size, (float)in_leaf_size, (float)in_leaf_size);
  voxelized.filter(*out_cloud_ptr);
}

void ROSMultiLidarCalibratorApp::InitializeROSIo(ros::NodeHandle &in_private_handle)
{
  //get params
  std::string points_parent_topic_str, points_child_topic_str;
  std::string initial_pose_topic_str = "/initialpose";
  std::string calibrated_points_topic_str = "/points_calibrated";

  in_private_handle.param<std::string>("points_parent_src", points_parent_topic_str, "/lidar/parent/points_raw_cropped");
  ROS_INFO("[%s] points_parent_src: %s",__APP_NAME__, points_parent_topic_str.c_str());

  in_private_handle.param<std::string>("points_child_src", points_child_topic_str, "/lidar/front_left/points_raw_cropped");
  ROS_INFO("[%s] points_child_src: %s",__APP_NAME__, points_child_topic_str.c_str());

  in_private_handle.param<std::string>("parent_frame", parent_frame_, "/lidar/parent/os_sensor");
  ROS_INFO("[%s] parent_frame: %s",__APP_NAME__, parent_frame_.c_str());

  in_private_handle.param<std::string>("child_frame", child_frame_, "/lidar/front_left/os_sensor");
  ROS_INFO("[%s] child_frame: %s",__APP_NAME__, child_frame_.c_str());
  UpdateCurrentGuessWithLookup();

  in_private_handle.param<double>("voxel_size", voxel_size_, 0.1);
  ROS_INFO("[%s] voxel_size: %.2f",__APP_NAME__, voxel_size_);

  in_private_handle.param<double>("ndt_epsilon", ndt_epsilon_, 0.01);
  ROS_INFO("[%s] ndt_epsilon: %.2f",__APP_NAME__, ndt_epsilon_);

  in_private_handle.param<double>("ndt_step_size", ndt_step_size_, 0.1);
  ROS_INFO("[%s] ndt_step_size: %.2f",__APP_NAME__, ndt_step_size_);

  in_private_handle.param<double>("ndt_resolution", ndt_resolution_, 1.0);
  ROS_INFO("[%s] ndt_resolution: %.2f",__APP_NAME__, ndt_resolution_);

  in_private_handle.param<int>("ndt_iterations", ndt_iterations_, 400);
  ROS_INFO("[%s] ndt_iterations: %d",__APP_NAME__, ndt_iterations_);

  in_private_handle.param<double>("x", initial_x_, 0.0);
  in_private_handle.param<double>("y", initial_y_, 0.0);
  in_private_handle.param<double>("z", initial_z_, 0.0);
  in_private_handle.param<double>("roll", initial_roll_, 0.0);
  in_private_handle.param<double>("pitch", initial_pitch_, 0.0);
  in_private_handle.param<double>("yaw", initial_yaw_, 0.0);
  in_private_handle.param<bool>("downsample_child_points", downsample_child_points_, true);
  in_private_handle.param<bool>("downsample_parent_points", downsample_parent_points_, true);




  ROS_INFO("[%s] Initialization Transform x: %.2f y: %.2f z: %.2f roll: %.2f pitch: %.2f yaw: %.2f", __APP_NAME__,
           initial_x_, initial_y_, initial_z_,
           initial_roll_, initial_pitch_, initial_yaw_);

  //generate subscribers and synchronizer
  // cloud_parent_subscriber_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_handle_,
  //                                                                                      points_parent_topic_str, 10);
  // ROS_INFO("[%s] Subscribing to... %s",__APP_NAME__, points_parent_topic_str.c_str());

  // cloud_child_subscriber_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_handle_,
  //                                                                                         points_child_topic_str, 10);
  // ROS_INFO("[%s] Subscribing to... %s",__APP_NAME__, points_child_topic_str.c_str());

  initialpose_subscriber_ = node_handle_.subscribe(initial_pose_topic_str, 10,
                                                            &ROSMultiLidarCalibratorApp::InitialPoseCallback, this);
  ROS_INFO("[%s] Subscribing to... %s",__APP_NAME__, initial_pose_topic_str.c_str());

  calibrated_cloud_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>(calibrated_points_topic_str, 1);
  ROS_INFO("[%s] Publishing PointCloud to... %s",__APP_NAME__, calibrated_points_topic_str.c_str());

  // cloud_synchronizer_ =
  //     new message_filters::Synchronizer<SyncPolicyT>(SyncPolicyT(100),
  //                                                    *cloud_parent_subscriber_,
  //                                                    *cloud_child_subscriber_);
  // cloud_synchronizer_->registerCallback(boost::bind(&ROSMultiLidarCalibratorApp::PointsCallback, this, _1, _2));

  bridge = new lidar_calib_test_bridge(&node_handle_, &in_private_handle, 
                      boost::bind(&ROSMultiLidarCalibratorApp::PointsCallback, this, _1, _2));

  calibration_stat_pub = node_handle_.advertise<std_msgs::String>("/calibration_stats", 1);

}


void ROSMultiLidarCalibratorApp::Run()
{
  ros::NodeHandle private_node_handle("~");

  InitializeROSIo(private_node_handle);

  ROS_INFO("[%s] Ready. Waiting for data...",__APP_NAME__);

  ros::spin();

  ROS_INFO("[%s] END",__APP_NAME__);
}
void ROSMultiLidarCalibratorApp::UpdateCurrentGuessWithLookup()
{
    tf::StampedTransform transform_stamped;
    try {
        tf_listener_.waitForTransform(parent_frame_, child_frame_, ros::Time(0), ros::Duration(10));
        tf_listener_.lookupTransform(parent_frame_, child_frame_, ros::Time(0), transform_stamped);

    } catch (tf2::TransformException & ex) {
        ROS_WARN("%s", ex.what());
        ROS_ERROR("Please publish TF %s to %s", parent_frame_.c_str(), child_frame_.c_str());
        transform_stamped.setRotation(tf::Quaternion::getIdentity());
        transform_stamped.getOrigin().setZero();
    }
    Eigen::Affine3d parent_to_child_affine;
    tf::transformTFToEigen(transform_stamped, parent_to_child_affine);
    current_guess_ = parent_to_child_affine.matrix().cast<float>();
    std::cout << current_guess_ << std::endl;
}
ROSMultiLidarCalibratorApp::ROSMultiLidarCalibratorApp()
{
    current_guess_ = Eigen::Matrix4f::Identity();

}