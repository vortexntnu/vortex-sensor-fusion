#include <image_filters/image_processing.hpp>
#include <iostream>

namespace vortex::image_filters
{

std::map<std::string, FilterFunction> filter_functions = {
    {"no_filter", no_filter},
    {"sharpening", sharpening_filter},
    {"unsharpening", unsharpening_filter},
    {"eroding", eroding_filter},
    {"dilating", dilating_filter},
    {"white_balancing", white_balance_filter},
    {"ebus", ebus_filter}
};

void no_filter(const cv::Mat &original, cv::Mat &filtered,[[maybe_unused]] const FilterParams& filter_params)
{
	original.copyTo(filtered);
}

void sharpening_filter(const cv::Mat &original, cv::Mat &filtered,[[maybe_unused]] const FilterParams& filter_params)
{
	// Sharpen image
	cv::Mat kernel = (cv::Mat_<float>(3, 3) << 0, -1, 0, -1, 5, -1, 0, -1, 0);
	cv::filter2D(original, filtered, -1, kernel);
}


void unsharpening_filter(const cv::Mat &original, cv::Mat &filtered, const FilterParams& filter_params)
{
	int blur_size = filter_params.unsharpening.blur_size;
	// Create a blurred version of the image
	cv::Mat blurred;
	GaussianBlur(original, blurred, cv::Size(2 * blur_size + 1, 2 * blur_size + 1), 0);

	// Compute the unsharp mask
	cv::Mat mask = original - blurred;
	cv::Mat unsharp;

	addWeighted(original, 1, mask, 3, 0, filtered);
}

void eroding_filter(const cv::Mat &original, cv::Mat &filtered, const FilterParams& filter_params)
{
	int erosion_size = filter_params.eroding.size;
	// Create a structuring element for dilation and erosion
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1), cv::Point(erosion_size, erosion_size));

	// Apply erosion to the image
	cv::erode(original, filtered, element);
}

void dilating_filter(const cv::Mat &original, cv::Mat &filtered, const FilterParams& filter_params)
{
	int dilation_size = filter_params.dilating.size;
	// Create a structuring element for dilation and erosion
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1), cv::Point(dilation_size, dilation_size));

	// Apply dilation to the image
	cv::dilate(original, filtered, element);
}

void white_balance_filter(const cv::Mat &original, cv::Mat &filtered, const FilterParams& filter_params)
{
	double contrast_percentage = filter_params.white_balancing.contrast_percentage;
	cv::Ptr<cv::xphoto::SimpleWB> balance = cv::xphoto::createSimpleWB();
	balance->setP(contrast_percentage);
	balance->balanceWhite(original, filtered);
}

void ebus_filter(const cv::Mat &original, cv::Mat &filtered, const FilterParams& filter_params)
{
	int blur_size = filter_params.ebus.blur_size;
	int mask_weight = filter_params.ebus.mask_weight;
	// Erode image to make blacks more black
	cv::Mat eroded;
	eroding_filter(original, eroded, filter_params);

	// Make an unsharp mask from original image
	cv::Mat blurred;
	GaussianBlur(original, blurred, cv::Size(2 * blur_size + 1, 2 * blur_size + 1), 0);

	// Compute the unsharp mask
	cv::Mat mask = original - blurred;
	cv::Mat unsharp;

	// Add mask to the eroded image instead of the original
	// Higher weight of mask will create a sharper but more noisy image
	addWeighted(eroded, 1, mask, mask_weight, 0, filtered);
}

void apply_filter(const cv::Mat &original, cv::Mat &filtered, const FilterParams& filter_params)
{
	// Find the filter in the filter_functions map
    auto it = filter_functions.find(filter_params.filter_type);
    if (it != filter_functions.end()) {
		// Calls the filter function
        it->second(original, filtered, filter_params);
    } else {
        original.copyTo(filtered);  // Default to no filter
    }
}

} // namespace vortex::image_filters