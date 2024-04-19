#include <image_filters/image_processing.hpp>
#include <iostream>

namespace vortex::image_processing
{

void no_filter([[maybe_unused]] const FilterParams& params, const cv::Mat &original, cv::Mat &filtered)
{
	original.copyTo(filtered);
}

void sharpening([[maybe_unused]] const FilterParams& params, const cv::Mat &original, cv::Mat &filtered)
{
	// Sharpen image
	cv::Mat kernel = (cv::Mat_<float>(3, 3) << 0, -1, 0, -1, 5, -1, 0, -1, 0);
	cv::filter2D(original, filtered, -1, kernel);
}


void unsharpening(const FilterParams& params, const cv::Mat &original, cv::Mat &filtered)
{
	int blur_size = params.unsharpening.blur_size;
	// Create a blurred version of the image
	cv::Mat blurred;
	GaussianBlur(original, blurred, cv::Size(2 * blur_size + 1, 2 * blur_size + 1), 0);

	// Compute the unsharp mask
	cv::Mat mask = original - blurred;
	cv::Mat unsharp;

	addWeighted(original, 1, mask, 3, 0, filtered);
}

void eroding(const FilterParams& params, const cv::Mat &original, cv::Mat &filtered)
{
	int erosion_size = params.eroding.size;
	// Create a structuring element for dilation and erosion
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1), cv::Point(erosion_size, erosion_size));

	// Apply erosion to the image
	cv::erode(original, filtered, element);
}

void dilating(const FilterParams& params, const cv::Mat &original, cv::Mat &filtered)
{
	int dilation_size = params.dilating.size;
	// Create a structuring element for dilation and erosion
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1), cv::Point(dilation_size, dilation_size));

	// Apply dilation to the image
	cv::dilate(original, filtered, element);
}

void white_balance(const FilterParams& params, const cv::Mat &original, cv::Mat &filtered)
{
	double contrast_percentage = params.white_balancing.contrast_percentage;
	cv::Ptr<cv::xphoto::SimpleWB> balance = cv::xphoto::createSimpleWB();
	balance->setP(contrast_percentage);
	balance->balanceWhite(original, filtered);
}

void ebus(const FilterParams& params, const cv::Mat &original, cv::Mat &filtered)
{
	int blur_size = params.ebus.blur_size;
	int mask_weight = params.ebus.mask_weight;
	// Erode image to make blacks more black
	cv::Mat eroded;
	
	int erosion_size = params.eroding.size;
	// Create a structuring element for dilation and erosion
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1), cv::Point(erosion_size, erosion_size));

	// Apply erosion to the image
	cv::erode(original, eroded, element);

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

void apply_filter(const std::string& filter, const FilterParams& params, const cv::Mat &original, cv::Mat &filtered)
{
	if(filter_functions.contains(filter)){
		((filter_functions.at(filter)))(params, original, filtered);
    } else {
        original.copyTo(filtered);  // Default to no filter
    }
}

} // namespace vortex::image_processing