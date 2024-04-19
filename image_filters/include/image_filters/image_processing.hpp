#ifndef IMAGE_PROCESSING_HPP
#define IMAGE_PROCESSING_HPP

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/xphoto.hpp>
#include <iostream>
#include <map>

namespace vortex::image_processing
{


struct UnsharpeningParams {
    int blur_size;
};

struct ErodingParams {
    int size;
};

struct DilatingParams {
    int size;
};

struct WhiteBalancingParams {
    double contrast_percentage;
};

struct EbusParams {
    int erosion_size;
    int blur_size;
    int mask_weight;
};

struct FilterParams {
    UnsharpeningParams unsharpening;
    ErodingParams eroding;
    DilatingParams dilating;
    WhiteBalancingParams white_balancing;
    EbusParams ebus;
};

typedef void (*FilterFunction)(const FilterParams&, const cv::Mat&, cv::Mat&);

/**
 * Reads the filter_type from the FilterParams struct 
 * and calls the appropriate filter function from the filter_functions map.
 */
void apply_filter(const std::string& filter, const FilterParams& params, const cv::Mat &original, cv::Mat &filtered);

/**
 * No filter, just copy the image
 */
void no_filter(const FilterParams& params, const cv::Mat &original, cv::Mat &modified);

/**
 * Makes edges harder
 */
void sharpening(const FilterParams& params, const cv::Mat &original, cv::Mat &modified);

/**
 * Makes edges harder in a smarter way
 */
void unsharpening(const FilterParams& params, const cv::Mat &original, cv::Mat &modified);

/**
 * Expands the dark areas of the image
 */
void eroding(const FilterParams& params, const cv::Mat &original, cv::Mat &modified);

/**
 * Expands the bright areas of the image
 */
void dilating(const FilterParams& params, const cv::Mat &original, cv::Mat &modified);

/**
 * White Balancing Filter
 */
void white_balance(const FilterParams& params, const cv::Mat &original, cv::Mat &filtered);

/**
 * A filter that worked well-ish in the mc-lab conditions easter 2023
 * Uses a combination of dilation and unsharpening
 */
void ebus(const FilterParams& params, const cv::Mat &original, cv::Mat &filtered);

const static std::map<std::string, FilterFunction> filter_functions ={
	{"no_filter", no_filter},
	{"sharpening", sharpening},
	{"unsharpening", unsharpening},
	{"eroding", eroding},
	{"dilating", dilating},
	{"white_balancing", white_balance},
	{"ebus", ebus}};


} // namespace image_processing
#endif // IMAGE_PROCESSING_HPP