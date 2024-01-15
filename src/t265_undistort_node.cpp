#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

// cv::Mat K, D;
cv::Mat map1, map2;
cv::Vec2d temp_size;

image_transport::Publisher img_pub;

void read_param(std::string param_file_path)
{
    cv::FileStorage param_file = cv::FileStorage(param_file_path, cv::FileStorage::READ);
    cv::Mat K, D;

    param_file["K"] >> K;
    param_file["D"] >> D;
    param_file["size"] >> temp_size;

    cv::Size size(temp_size[0], temp_size[1]);

    cv::fisheye::initUndistortRectifyMap(K, D, cv::noArray(), K, size, CV_32FC1, map1, map2);

}

void image_callback(const sensor_msgs::ImageConstPtr& img)
{
    cv::Mat img_in = cv_bridge::toCvShare(img, "bgr8")->image;
    cv::Mat img_out;
    // cv::Size size(temp_size[0], temp_size[1]);


    // cv::fisheye::undistortImage(img_in, img_out, K, D, cv::noArray(), size);
    cv::remap(img_in, img_out, map1, map2, cv::INTER_LINEAR);

    sensor_msgs::ImagePtr rect_img = cv_bridge::CvImage(img->header, "bgr8", img_out).toImageMsg();
    
    img_pub.publish(rect_img);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "t265_undistort");
    ros::NodeHandle nh("~");
    
    std::string param, topic_name;
    if (nh.getParam("param_file_path", param) && (nh.getParam("topic_name", topic_name)))
    {
      ROS_INFO("Got param file path: %s", param.c_str());
      ROS_INFO("Got topic name: %s", topic_name.c_str());
    }
    else
    {
      ROS_ERROR("Failed to get param.");
      ros::shutdown();
      return 0;
    }

    read_param(param);

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber img_sub = it.subscribe(topic_name, 1, image_callback);
    img_pub = it.advertise("/utat/fisheye1/undistorted", 1);

    ros::spin();
}