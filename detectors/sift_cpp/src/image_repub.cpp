#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>  // Include header for cvtColor

void imageCallback(const sensor_msgs::ImageConstPtr& msg, const image_transport::Publisher &pub)
{
    try
    {
        cv::Mat color_img = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::Mat gray_img;
        cv::cvtColor(color_img, gray_img, cv::COLOR_BGR2GRAY);  // Convert to grayscale
        sensor_msgs::ImagePtr gray_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", gray_img).toImageMsg();
        pub.publish(gray_msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sift_cpp");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("gray_image", 1);
    image_transport::Subscriber sub = it.subscribe("/zed2i/zed_node/left/image_rect_color", 1, boost::bind(imageCallback, _1, pub));

    ros::spin();
}
