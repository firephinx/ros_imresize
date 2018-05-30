#include <ros/ros.h>
#include "ros_imresize/image_handler.h"

int main( int argc, char** argv )
{
    ros::init(argc, argv, "ros_imresize");

    ImageHandler resize;

    ros::spin();

    return 0;
}
