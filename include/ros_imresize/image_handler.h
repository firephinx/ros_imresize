#ifndef IMAGE_HANDLER_H_
#define IMAGE_HANDLER_H_

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cv_bridge/cv_bridge.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <image_transport/image_transport.h>


class ImageHandler
{

public:

    ImageHandler();
    ~ImageHandler();

protected:

    void setCameraInfo();
    void imageCallback(const sensor_msgs::ImageConstPtr& image_msg);

    sensor_msgs::CameraInfo camera_info_msg_;

    ros::NodeHandle nh_;

    image_transport::ImageTransport it_;

    image_transport::Subscriber image_subscriber_;
    image_transport::Publisher image_publisher_;

    ros::Publisher camera_info_publisher_;

    int width_;
    int height_;

};

#endif
