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
enum class FilterType {
    nofilter,
    sharpening,
    unsharpening,
    eroding,
    dilating,
    white_balancing,
    ebus
};


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

std::map<std::string, FilterType> filter_map = {
    {"nofilter", FilterType::nofilter},
    {"sharpening", FilterType::sharpening},
    {"unsharpening", FilterType::unsharpening},
    {"eroding", FilterType::eroding},
    {"dilating", FilterType::dilating},
    {"white_balancing", FilterType::white_balancing},
    {"ebus", FilterType::ebus}};

    FilterType get_filter_type() {
        return filter_map[filter_type];
    }

};

/**
 * Makes edges harder
 */
void sharpeningFilter(const cv::Mat &original, cv::Mat &modified);
/**
 * Makes edges harder in a smarter way
 */
void unsharpeningFilter(const cv::Mat &original, cv::Mat &modified, size_t blurSize = 1);

/**
 * Expands the dark areas of the image
 */
void erodingFilter(const cv::Mat &original, cv::Mat &modified, size_t erosionSize = 1);

/**
 * Expands the bright areas of the image
 */
void dilatingFilter(const cv::Mat &original, cv::Mat &modified, size_t dilationSize = 1);

/**
 * White Balancing Filter
 */
void whiteBalanceFilter(const cv::Mat &original, cv::Mat &filtered, double contrastPercentage = 0.2);

/**
 * A filter that worked well-ish in the mc-lab conditions easter 2023
 * Uses a combination of dilation and unsharpening
 */
void ebusFilter(const cv::Mat &original, cv::Mat &filtered, size_t erosionSize = 2, size_t blurSize = 30, size_t maskWeight = 5);

void apply_filter(const cv::Mat &original, cv::Mat &filtered, FilterParams &filter_params);

} // namespace image_filters
#endif // IMAGE_PROCESSING_HPP