#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/version.hpp>
#include <opencv2/opencv.hpp>
// #include <opencv2/features2d.hpp>
// #include <opencv2/xfeatures2d/nonfree.hpp>
// #include <opencv2/xfeatures2d.hpp>

// global variables for storing object features
cv::Mat object_img, object_descriptors;
std::vector<cv::KeyPoint> object_keypoints;

// global SIFT detector
cv::Ptr<cv::xfeatures2d::SIFT> sift = cv::xfeatures2d::SIFT::create();

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::Mat scene_img = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::Mat scene_img_gray;
    cv::cvtColor(scene_img, scene_img_gray, cv::COLOR_BGR2GRAY);

    // compute scene features
    std::vector<cv::KeyPoint> scene_keypoints;
    cv::Mat scene_descriptors;
    sift->detectAndCompute(scene_img_gray, cv::noArray(), scene_keypoints, scene_descriptors);

    // match features
    cv::BFMatcher matcher;
    std::vector<cv::DMatch> matches;
    matcher.match(object_descriptors, scene_descriptors, matches);

    // draw matches
    cv::Mat img_matches;
    cv::drawMatches(object_img, object_keypoints, scene_img, scene_keypoints,
      matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
      std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    // publish matched image
    sensor_msgs::ImagePtr output_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_matches).toImageMsg();
    pub.publish(output_msg);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_matcher");
  ros::NodeHandle nh;

  // load object image and compute its features
  object_img = cv::imread("/home/vortex/vortex_ws/src/vortex-sensor-fusion/data/sift_images/cup.png", cv::IMREAD_GRAYSCALE);
  sift->detectAndCompute(object_img, cv::noArray(), object_keypoints, object_descriptors);

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/zed2i/zed_node/left/image_rect_color", 1, imageCallback);
  image_transport::Publisher pub = it.advertise("/matched_images", 1);

  ros::spin();
}
