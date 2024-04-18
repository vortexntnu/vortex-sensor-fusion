#ifndef IMAGE_PROCESSING_HPP
#define IMAGE_PROCESSING_HPP

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/xphoto.hpp>
#include <iostream>
#include <map>

namespace vortex::image_filters
{

struct UnsharpeningFilterParams {
    int blur_size;
};

struct ErodingFilterParams {
    int size;
};

struct DilatingFilterParams {
    int size;
};

struct WhiteBalancingFilterParams {
    double contrast_percentage;
};

struct EbusFilterParams {
    int erosion_size;
    int blur_size;
    int mask_weight;
};

struct FilterParams {
    std::string filter_type;
    UnsharpeningFilterParams unsharpening;
    ErodingFilterParams eroding;
    DilatingFilterParams dilating;
    WhiteBalancingFilterParams white_balancing;
    EbusFilterParams ebus;
};

typedef void (*FilterFunction)(const cv::Mat&, cv::Mat&, const FilterParams&);

extern std::map<std::string, FilterFunction> filter_functions;


/**
 * No filter, just copy the image
 */
void no_filter(const cv::Mat &original, cv::Mat &modified, const FilterParams& filter_params);

/**
 * Makes edges harder
 */
void sharpening_filter(const cv::Mat &original, cv::Mat &modified, const FilterParams& filter_params);

/**
 * Makes edges harder in a smarter way
 */
void unsharpening_filter(const cv::Mat &original, cv::Mat &modified, const FilterParams& filter_params);

/**
 * Expands the dark areas of the image
 */
void eroding_filter(const cv::Mat &original, cv::Mat &modified, const FilterParams& filter_params);

/**
 * Expands the bright areas of the image
 */
void dilating_filter(const cv::Mat &original, cv::Mat &modified, const FilterParams& filter_params);

/**
 * White Balancing Filter
 */
void white_balance_filter(const cv::Mat &original, cv::Mat &filtered, const FilterParams& filter_params);

/**
 * A filter that worked well-ish in the mc-lab conditions easter 2023
 * Uses a combination of dilation and unsharpening
 */
void ebus_filter(const cv::Mat &original, cv::Mat &filtered, const FilterParams& filter_params);


/**
 * Reads the filter_type from the FilterParams struct 
 * and calls the appropriate filter function from the filter_functions map.
 */
void apply_filter(const cv::Mat &original, cv::Mat &filtered, FilterParams& filter_params);

} // namespace image_filters
#endif // IMAGE_PROCESSING_HPP