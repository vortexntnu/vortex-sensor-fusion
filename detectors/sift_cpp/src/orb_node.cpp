#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <ros/package.h>

std::string path_to_image = ros::package::getPath("sift") + "/data/sift_images/cup.png";
cv::Mat object_img = cv::imread(path_to_image, cv::IMREAD_GRAYSCALE);


void imageCallback(const sensor_msgs::ImageConstPtr& msg, const image_transport::Publisher &pub)
{
    try
    {
        cv::Ptr<cv::ORB> orb = cv::ORB::create();
        std::vector<cv::KeyPoint> object_keypoints;
        cv::Mat object_descriptors;
        orb->detectAndCompute(object_img, cv::noArray(), object_keypoints, object_descriptors);
        cv::BFMatcher matcher(cv::NORM_HAMMING);

        cv::Mat color_img = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::Mat gray_img;
        cv::cvtColor(color_img, gray_img, cv::COLOR_BGR2GRAY);
        
        // Compute ORB features for the incoming image
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        orb->detectAndCompute(gray_img, cv::noArray(), keypoints, descriptors);

        // Match the features
        std::vector<cv::DMatch> matches;
        matcher.match(object_descriptors, descriptors, matches);

        // Draw the matches
        cv::Mat output_img;
        cv::drawMatches(object_img, object_keypoints, color_img, keypoints, matches, output_img);
        
        sensor_msgs::ImagePtr output_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", output_img).toImageMsg();
        pub.publish(output_msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("matched_image", 1);
    image_transport::Subscriber sub = it.subscribe("/zed2i/zed_node/left/image_rect_color", 1, boost::bind(imageCallback, _1, pub));

    ros::spin();
}
