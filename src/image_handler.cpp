/*
* image_handler.h
* Copyright (c) 2014 PAL Robotics sl. All Rights Reserved
* Created on: 10/01/2014
* Author: Jéremie Deray
*/

#include "ros_imresize/image_handler.h"

#include <cv_bridge/cv_bridge.h>
#include <ros/callback_queue.h>
#include <opencv2/imgproc/imgproc.hpp>


////////////////////////////////////////////////////////////////////////////
////////////                                                    ////////////
////////////                    ImageHandler                    ////////////
////////////                                                    ////////////
////////////////////////////////////////////////////////////////////////////


ImageHandler::ImageHandler() :
_infoReceived(false),
_nh("/ros_imresize"),
_width(0),
_height(0),
_it(_nh)
{
    std::string inputCameraTopicName;
    std::string inputCameraInfoTopicName;
    std::string outputCameraTopicName;
    std::string outputCameraInfoTopicName;

    ros::Rate rate(10);

    ROS_INFO("Retrieving parameters ...");

    while(!(_nh.getParam("input_camera_topic", inputCameraTopicName) && _nh.getParam("input_camera_info_topic", inputCameraInfoTopicName) && 
            _nh.getParam("output_camera_topic", outputCameraTopicName) && _nh.getParam("output_camera_info_topic", outputCameraInfoTopicName)) && 
            ros::ok())
    {
    	ros::spinOnce();
    	rate.sleep();
    }

    ROS_INFO("Parameters retrieved ...");

    _nh.param("width", _width, (int)640);
    _nh.param("height", _height, (int)480);

    ros::Subscriber sub_info = _nh.subscribe(inputCameraInfoTopicName, 1, &ImageHandler::setCameraInfo, this);

    ROS_INFO("WAITING for ROS camera calibration!\n");
    ros::Rate rate(10);
    while (!_infoReceived  && ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("RECEIVED ROS camera calibration!\n");

    sub_info.shutdown();

    _sub_img = _it.subscribe(inputCameraTopicName, 1, &ImageHandler::topicCallback, this);

    _pub_img = _it.advertise(inputCameraTopicName + "_resized", 1);

    _pub_info = _nh.advertise<sensor_msgs::CameraInfo>(inputCameraInfoTopicName + "_resized", 1);

    ROS_INFO("Running\n");
}

ImageHandler::~ImageHandler()
{
}

void ImageHandler::topicCallback(const sensor_msgs::ImageConstPtr& received_image)
{
    cv_bridge::CvImagePtr cvPtr;
    cvPtr = cv_bridge::toCvCopy(received_image, sensor_msgs::image_encodings::BGR8);
       
    cv::Mat img = cvPtr->image;

    cv::resize(img, cvPtr->image, cv::Size(_width, _height),
               0, 0, cv::INTER_LINEAR);

    _pub_img.publish(cvPtr->toImageMsg());
    _pub_info.publish(_infoCam);
}

void ImageHandler::setCameraInfo(const sensor_msgs::CameraInfoConstPtr &received_info)
{
    _infoCam = *received_info;

    float scale_x = (float)(_width) / (float)(_infoCam.width);
    float scale_y = (float)(_height) / (float)(_infoCam.height);

    _infoCam.K[0] *= scale_x;
    _infoCam.K[2] *= scale_x;

    _infoCam.K[4] *= scale_y;
    _infoCam.K[5] *= scale_y;

    ROS_INFO_STREAM("Previous camera info :\n" << *received_info << "\n");

    if (_undistord)
    {
        _K = cv::Mat::eye(3, 3, CV_32F);

        _K.at<float>(0) = _infoCam.K[0];
        _K.at<float>(2) = _infoCam.K[2];

        _K.at<float>(4) = _infoCam.K[4];
        _K.at<float>(5) = _infoCam.K[5];

        if (_infoCam.distortion_model == "plumb_bob")
        {
            _dist = cv::Mat(_infoCam.D);
        }
        else
        {
            _dist = cv::Mat::zeros(5, 1, CV_32F);
            //TODO : check for other model
        }

        _infoCam.distortion_model = "";
        _infoCam.D.clear();

        _infoCam.D.clear();

        ROS_INFO_STREAM("Undistortion active with param :\n" << _dist << "\n");
        ROS_INFO_STREAM("Undistortion active with param :\n" << _K << "\n");
    }

    _infoCam.width = _width;
    _infoCam.height = _height;

    ROS_INFO_STREAM("New camera info :\n" << _infoCam << "\n");

    _infoReceived = true;

}
