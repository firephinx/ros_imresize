#include "ros_imresize/image_handler.h"

ImageHandler::ImageHandler() : nh_("/ros_imresize"), it_(nh_)
{
    std::string inputCameraTopicName;
    std::string outputCameraTopicName;
    std::string outputCameraInfoTopicName;

    nh_.param("/ros_imresize/input_camera_topic", inputCameraTopicName, std::string("/pepper_local_republisher/pepper_robot/camera/front/image_rect_color"));
    nh_.param("/ros_imresize/output_camera_topic", outputCameraTopicName, std::string("/pepper_local_republisher/pepper_robot/camera/front/image_rect_color_resized"));
    nh_.param("/ros_imresize/output_camera_info_topic", outputCameraInfoTopicName, std::string("/pepper_local_republisher/pepper_robot/camera/front/camera_info"));
    nh_.param("/ros_imresize/width", width_, (int)640);
    nh_.param("/ros_imresize/height", height_, (int)480);

    setCameraInfo();

    image_subscriber_ = it_.subscribe(inputCameraTopicName, 1, &ImageHandler::imageCallback, this);

    image_publisher_ = it_.advertise(outputCameraTopicName, 1);

    camera_info_publisher_ = nh_.advertise<sensor_msgs::CameraInfo>(outputCameraInfoTopicName, 1);

    ROS_INFO("Running imresize node.\n");
}

ImageHandler::~ImageHandler()
{
}

void ImageHandler::imageCallback(const sensor_msgs::ImageConstPtr& image_msg)
{
    cv_bridge::CvImagePtr cvPtr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
       
    cv::Mat image_copy = cvPtr->image;

    cv::resize(image_copy, cvPtr->image, cv::Size(width_, height_), 0, 0, cv::INTER_LINEAR);

    image_publisher_.publish(cvPtr->toImageMsg());
    camera_info_msg_.header = image_msg->header;
    camera_info_publisher_.publish(camera_info_msg_);
}

void ImageHandler::setCameraInfo()
{
    camera_info_msg_.height = height_;
    camera_info_msg_.width = width_;
    camera_info_msg_.distortion_model = "plumb_bob";
    camera_info_msg_.D.push_back(-0.0870160932911717);
    camera_info_msg_.D.push_back(0.128210165050533);
    camera_info_msg_.D.push_back(0.003379500659424);
    camera_info_msg_.D.push_back(-0.00106205540818586);
    camera_info_msg_.D.push_back(0.0);
    camera_info_msg_.K[0] = 274.139508945831;
    camera_info_msg_.K[1] = 0.0;
    camera_info_msg_.K[2] = 141.184472810944;
    camera_info_msg_.K[3] = 0.0;
    camera_info_msg_.K[4] = 275.741846757374;
    camera_info_msg_.K[5] = 106.693773654172;
    camera_info_msg_.K[6] = 0.0;
    camera_info_msg_.K[7] = 0.0;
    camera_info_msg_.K[8] = 1.0;
    camera_info_msg_.R[0] = 1.0;
    camera_info_msg_.R[1] = 0.0;
    camera_info_msg_.R[2] = 0.0;
    camera_info_msg_.R[3] = 0.0;
    camera_info_msg_.R[4] = 1.0;
    camera_info_msg_.R[5] = 0.0;
    camera_info_msg_.R[6] = 0.0;
    camera_info_msg_.R[7] = 0.0;
    camera_info_msg_.R[8] = 1.0;
    camera_info_msg_.P[0] = 272.423675537109;
    camera_info_msg_.P[1] = 0.0;
    camera_info_msg_.P[2] = 141.131930791285;
    camera_info_msg_.P[3] = 0.0;
    camera_info_msg_.P[4] = 0.0;
    camera_info_msg_.P[5] = 273.515747070312;
    camera_info_msg_.P[6] = 107.391746054313;
    camera_info_msg_.P[7] = 0.0;
    camera_info_msg_.P[8] = 0.0;
    camera_info_msg_.P[9] = 0.0;
    camera_info_msg_.P[10] = 1.0;
    camera_info_msg_.P[11] = 0.0;
}
