// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
#include <algorithm>
#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <limits>

#include <k4a/k4a.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "TransformationOpenCV.h"
#include "Transformation.h"
#include "MultiDeviceCapturer.h"
#include "FileHandler.h"

// Allowing at least 160 microseconds between depth cameras should ensure they do not interfere with one another.
constexpr uint32_t MIN_TIME_BETWEEN_DEPTH_CAMERA_PICTURES_USEC = 160;

/**
* Converts Azure Kinect color image to OpenCV Format
* @param im : Azure Kinect color image
* @return color image to OpenCV format
*/
static cv::Mat colorToOpencv(const k4a::image& im)
{
    cv::Mat cv_image_with_alpha(im.get_height_pixels(), im.get_width_pixels(), CV_8UC4, (void*)im.get_buffer());
    cv::Mat cv_image_no_alpha;
    cv::cvtColor(cv_image_with_alpha, cv_image_no_alpha, cv::COLOR_BGRA2BGR);
    return cv_image_no_alpha;
}

/**
* Converts Azure Kinect calibration to OpenCV matrix format
* @param cal : Azure Kinect calibration
* @return calibration to OpenCV matrix format
*/
static cv::Matx33f calibrationToColorCameraMatrix(const k4a::calibration& cal)
{
    const k4a_calibration_intrinsic_parameters_t::_param& i = cal.color_camera_calibration.intrinsics.parameters.param;
    cv::Matx33f camera_matrix = cv::Matx33f::eye();
    camera_matrix(0, 0) = i.fx;
    camera_matrix(1, 1) = i.fy;
    camera_matrix(0, 2) = i.cx;
    camera_matrix(1, 2) = i.cy;
    return camera_matrix;
}

/**
* Get individual parameters of the intrisic model of the camera given its calibration
*
* @param cal : Azure kinect calibration
* @return a vector containing the radial and tangential distortion coefficients
*/
static std::vector<float> calibrationToColorCameraDistCoeffs(const k4a::calibration& cal)
{
    const k4a_calibration_intrinsic_parameters_t::_param& i = cal.color_camera_calibration.intrinsics.parameters.param;
    return { i.k1, i.k2, i.p1, i.p2, i.k3, i.k4, i.k5, i.k6 };
}

/**
* Function getting the positions of the corners of the chessboard in the world for each device
*
* @param main_color_image / secondary_color_image : the OpenCV color image of the main/secondary device
* @param chessboard_pattern : size of the chessboard
* @param main_chessboard_corners / secondary_chessboard_corners : vector containing the positions of the corners of the chessboard in the main/secondary im
* @return a boolean indicating if the corner where found or not
*/
bool findChessboardCornersHelper(const cv::Mat& main_color_image,
    const cv::Mat& secondary_color_image,
    const cv::Size& chessboard_pattern,
    std::vector<cv::Point2f>& main_chessboard_corners,
    std::vector<cv::Point2f>& secondary_chessboard_corners)
{
    bool found_chessboard_main = cv::findChessboardCorners(main_color_image,
        chessboard_pattern,
        main_chessboard_corners);
    bool found_chessboard_secondary = cv::findChessboardCorners(secondary_color_image,
        chessboard_pattern,
        secondary_chessboard_corners);

    // Cover the failure cases where chessboards were not found in one or both images.
    if (!found_chessboard_main || !found_chessboard_secondary)
    {
        if (found_chessboard_main)
        {
            std::cout << "Could not find the chessboard corners in the secondary image. Trying again...\n";
        }
        // Likewise, if the chessboard was found in the secondary image, it was not found in the main image.
        else if (found_chessboard_secondary)
        {
            std::cout << "Could not find the chessboard corners in the main image. Trying again...\n";
        }
        // The only remaining case is the corners were in neither image.
        else
        {
            std::cout << "Could not find the chessboard corners in either image. Trying again...\n";
        }
        return false;
    }
    // Before we go on, there's a quick problem with calibration to address.  Because the chessboard looks the same when
    // rotated 180 degrees, it is possible that the chessboard corner finder may find the correct points, but in the
    // wrong order.

    // A visual:
    //        Image 1                  Image 2
    // .....................    .....................
    // .....................    .....................
    // .........xxxxx2......    .....xxxxx1..........
    // .........xxxxxx......    .....xxxxxx..........
    // .........xxxxxx......    .....xxxxxx..........
    // .........1xxxxx......    .....2xxxxx..........
    // .....................    .....................
    // .....................    .....................

    // The problem occurs when this case happens: the find_chessboard() function correctly identifies the points on the
    // chessboard (shown as 'x's) but the order of those points differs between images taken by the two cameras.
    // Specifically, the first point in the list of points found for the first image (1) is the *last* point in the list
    // of points found for the second image (2), though they correspond to the same physical point on the chessboard.

    // To avoid this problem, we can make the assumption that both of the cameras will be oriented in a similar manner
    // (e.g. turning one of the cameras upside down will break this assumption) and enforce that the vector between the
    // first and last points found in pixel space (which will be at opposite ends of the chessboard) are pointing the
    // same direction- so, the dot product of the two vectors is positive.

    cv::Vec2f main_image_corners_vec = main_chessboard_corners.back() - main_chessboard_corners.front();
    cv::Vec2f secondary_image_corners_vec = secondary_chessboard_corners.back() - secondary_chessboard_corners.front();
    if (main_image_corners_vec.dot(secondary_image_corners_vec) <= 0.0)
    {
        std::reverse(secondary_chessboard_corners.begin(), secondary_chessboard_corners.end());
    }
    return true;
}

/**
* Returns the transformation to go from the main device to the secondary
*
* @param main_calib / secondary_calib : Azure Kinect calibration of the main/secondary device
* @param main_chessboard_corners_list / secondary_chessboard_corners_list : list of all positions of the corners for each fram for the main/secondary device
* @param image_size : size of the frames
* @param chessboard_pattern : size of the chessboard
* @param chessboard_square_length : size of the border of a square in the chessboard
*/
TransformationOpenCV stereoCalibration(const k4a::calibration& main_calib,
    const k4a::calibration& secondary_calib,
    const std::vector<std::vector<cv::Point2f>>& main_chessboard_corners_list,
    const std::vector<std::vector<cv::Point2f>>& secondary_chessboard_corners_list,
    const cv::Size& image_size,
    const cv::Size& chessboard_pattern,
    float chessboard_square_length)
{
    // We have points in each image that correspond to the corners that the findChessboardCorners function found.
    // However, we still need the points in 3 dimensions that these points correspond to. Because we are ultimately only
    // interested in find a transformation between two cameras, these points don't have to correspond to an external
    // "origin" point. The only important thing is that the relative distances between points are accurate. As a result,
    // we can simply make the first corresponding point (0, 0) and construct the remaining points based on that one. The
    // order of points inserted into the vector here matches the ordering of findChessboardCorners. The units of these
    // points are in millimeters, mostly because the depth provided by the depth cameras is also provided in
    // millimeters, which makes for easy comparison.
    std::vector<cv::Point3f> chessboard_corners_world;
    for (int h = 0; h < chessboard_pattern.height; ++h)
    {
        for (int w = 0; w < chessboard_pattern.width; ++w)
        {
            chessboard_corners_world.emplace_back(
                cv::Point3f{ w * chessboard_square_length, h * chessboard_square_length, 0.0 });
        }
    }

    // Calibrating the cameras requires a lot of data. OpenCV's stereoCalibrate function requires:
    // - a list of points in real 3d space that will be used to calibrate*
    // - a corresponding list of pixel coordinates as seen by the first camera*
    // - a corresponding list of pixel coordinates as seen by the second camera*
    // - the camera matrix of the first camera
    // - the distortion coefficients of the first camera
    // - the camera matrix of the second camera
    // - the distortion coefficients of the second camera
    // - the size (in pixels) of the images
    // - R: stereoCalibrate stores the rotation matrix from the first camera to the second here
    // - t: stereoCalibrate stores the translation vector from the first camera to the second here
    // - E: stereoCalibrate stores the essential matrix here (we don't use this)
    // - F: stereoCalibrate stores the fundamental matrix here (we don't use this)
    //
    // * note: OpenCV's stereoCalibrate actually requires as input an array of arrays of points for these arguments,
    // allowing a caller to provide multiple frames from the same camera with corresponding points. For example, if
    // extremely high precision was required, many images could be taken with each camera, and findChessboardCorners
    // applied to each of those images, and OpenCV can jointly solve for all of the pairs of corresponding images.
    // However, to keep things simple, we use only one image from each device to calibrate.  This is also why each of
    // the vectors of corners is placed into another vector.
    //
    // A function in OpenCV's calibration function also requires that these points be F32 types, so we use those.
    // However, OpenCV still provides doubles as output, strangely enough.
    std::vector<std::vector<cv::Point3f>> chessboard_corners_world_nested_for_cv(main_chessboard_corners_list.size(),
        chessboard_corners_world);

    cv::Matx33f main_camera_matrix = calibrationToColorCameraMatrix(main_calib);
    cv::Matx33f secondary_camera_matrix = calibrationToColorCameraMatrix(secondary_calib);
    std::vector<float> main_dist_coeff = calibrationToColorCameraDistCoeffs(main_calib);
    std::vector<float> secondary_dist_coeff = calibrationToColorCameraDistCoeffs(secondary_calib);

    // Finally, we'll actually calibrate the cameras.
    // Pass secondary first, then main, because we want a transform from secondary to main.
    TransformationOpenCV tr;
    double error = cv::stereoCalibrate(chessboard_corners_world_nested_for_cv,
        secondary_chessboard_corners_list,
        main_chessboard_corners_list,
        secondary_camera_matrix,
        secondary_dist_coeff,
        main_camera_matrix,
        main_dist_coeff,
        image_size,
        tr.R, // output
        tr.t, // output
        cv::noArray(),
        cv::noArray(),
        cv::CALIB_FIX_INTRINSIC | cv::CALIB_RATIONAL_MODEL | cv::CALIB_CB_FAST_CHECK);
    std::cout << "Finished calibrating!\n";
    std::cout << "Got error of " << error << "\n";
    return tr;
}

/**
* Function that calibrate each device independantly
*
* @param capturer :
* @param main_config / secondary_config : config of the main/secondary device
* @param chessboard_pattern : size of the chessboard
* @param chessboard_square_length : size of the border of a square in the chessboard
* @param calibration_timeout :
* @return
*/
static TransformationOpenCV calibrateDevices(MultiDeviceCapturer& capturer,
    const k4a_device_configuration_t& main_config,
    const k4a_device_configuration_t& secondary_config,
    const cv::Size& chessboard_pattern,
    float chessboard_square_length,
    double calibration_timeout,
    int firstDevice,
    int secondDevice)
{
    // We first get the cameras calibrations
    k4a::calibration main_calibration;
    if (firstDevice == 0) {
        main_calibration = capturer.getMasterDevice().get_calibration(main_config.depth_mode,
                main_config.color_resolution);
    }
    else {
        main_calibration = capturer.getSubordinateDeviceByIndex(firstDevice-1).get_calibration(main_config.depth_mode,
            main_config.color_resolution);
    }

    k4a::calibration secondary_calibration =
        capturer.getSubordinateDeviceByIndex(secondDevice-1).get_calibration(secondary_config.depth_mode,
            secondary_config.color_resolution);

    std::vector<std::vector<cv::Point2f>> main_chessboard_corners_list;
    std::vector<std::vector<cv::Point2f>> secondary_chessboard_corners_list;

    std::chrono::time_point<std::chrono::system_clock> start_time = std::chrono::system_clock::now();
    while (std::chrono::duration<double>(std::chrono::system_clock::now() - start_time).count() < calibration_timeout)
    {
        std::vector<k4a::capture> captures = capturer.getSynchronizedCaptures(secondary_config);
        k4a::capture& main_capture = captures[firstDevice];
        k4a::capture& secondary_capture = captures[secondDevice];
        // get_color_image is guaranteed to be non-null because we use get_synchronized_captures for color
        // (get_synchronized_captures also offers a flag to use depth for the secondary camera instead of color).
        k4a::image main_color_image = main_capture.get_color_image();
        k4a::image secondary_color_image = secondary_capture.get_color_image();
        cv::Mat cv_main_color_image = colorToOpencv(main_color_image);
        cv::Mat cv_secondary_color_image = colorToOpencv(secondary_color_image);

        std::vector<cv::Point2f> main_chessboard_corners;
        std::vector<cv::Point2f> secondary_chessboard_corners;
        bool got_corners = findChessboardCornersHelper(cv_main_color_image,
            cv_secondary_color_image,
            chessboard_pattern,
            main_chessboard_corners,
            secondary_chessboard_corners);
        if (got_corners)
        {
            main_chessboard_corners_list.emplace_back(main_chessboard_corners);
            secondary_chessboard_corners_list.emplace_back(secondary_chessboard_corners);
            cv::drawChessboardCorners(cv_main_color_image, chessboard_pattern, main_chessboard_corners, true);
            cv::drawChessboardCorners(cv_secondary_color_image, chessboard_pattern, secondary_chessboard_corners, true);
        }

        cv::imshow("Chessboard view from main camera", cv_main_color_image);
        cv::waitKey(1);
        cv::imshow("Chessboard view from secondary camera", cv_secondary_color_image);
        cv::waitKey(1);

        // Get 20 frames before doing calibration.
        if (main_chessboard_corners_list.size() >= 20)
        {
            return stereoCalibration(main_calibration,
                secondary_calibration,
                main_chessboard_corners_list,
                secondary_chessboard_corners_list,
                cv_main_color_image.size(),
                chessboard_pattern,
                chessboard_square_length);
        }
    }
    std::cerr << "Calibration timed out !\n ";
    exit(1);
}

/**
* The following functions provide the configurations that should be used for each camera.
* NOTE: For best results both cameras should have the same configuration (framerate, resolution, color and depth
* modes). Additionally the both master and subordinate should have the same exposure and power line settings. Exposure
* settings can be different but the subordinate must have a longer exposure from master. To synchronize a master and
* subordinate with different exposures the user should set `subordinate_delay_off_master_usec = ((subordinate exposure
* time) - (master exposure time))/2`.
*
*/
static k4a_device_configuration_t getDefaultConfig()
{
    k4a_device_configuration_t camera_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    camera_config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    camera_config.color_resolution = K4A_COLOR_RESOLUTION_720P;
    camera_config.depth_mode = K4A_DEPTH_MODE_WFOV_UNBINNED; // No need for depth during calibration
    camera_config.camera_fps = K4A_FRAMES_PER_SECOND_15;     // Don't use all USB bandwidth
    camera_config.subordinate_delay_off_master_usec = 0;     // Must be zero for master
    camera_config.synchronized_images_only = true;
    return camera_config;
}

/**
*  Master customizable settings
*/
static k4a_device_configuration_t getMasterConfig()
{
    k4a_device_configuration_t camera_config = getDefaultConfig();
    camera_config.wired_sync_mode = K4A_WIRED_SYNC_MODE_MASTER;

    // Two depth images should be seperated by MIN_TIME_BETWEEN_DEPTH_CAMERA_PICTURES_USEC to ensure the depth imaging
    // sensor doesn't interfere with the other. To accomplish this the master depth image captures
    // (MIN_TIME_BETWEEN_DEPTH_CAMERA_PICTURES_USEC / 2) before the color image, and the subordinate camera captures its
    // depth image (MIN_TIME_BETWEEN_DEPTH_CAMERA_PICTURES_USEC / 2) after the color image. This gives us two depth
    // images centered around the color image as closely as possible.
    camera_config.depth_delay_off_color_usec = -static_cast<int32_t>(MIN_TIME_BETWEEN_DEPTH_CAMERA_PICTURES_USEC / 2);
    camera_config.synchronized_images_only = true;
    return camera_config;
}

/**
*  Subordinate customizable settings
*/
static k4a_device_configuration_t getSubordinateConfig()
{
    k4a_device_configuration_t camera_config = getDefaultConfig();
    camera_config.wired_sync_mode = K4A_WIRED_SYNC_MODE_SUBORDINATE;
    camera_config.depth_delay_off_color_usec = MIN_TIME_BETWEEN_DEPTH_CAMERA_PICTURES_USEC / 2;
    return camera_config;
}

/*
===================================================
**              MAIN FUNCTION                    **
===================================================
Launch with given parameters :

argv[1] : filePath (where to save extrinsic calibration file)
argv[2] : fileName (the name of the file where to save the calibration; without extension)

*/
int main(int argc, char** argv)
{
    float chessboard_square_length = 26;
    int32_t color_exposure_usec = 8000;
    int32_t powerline_freq = 2;
    cv::Size chessboard_pattern(6, 9);
    double calibration_timeout = 60.0;
    uint32_t num_devices = k4a::device::get_installed_count();
    FileHandler fileHandler;
    if (argv[1] && argv[2]) {
        fileHandler.setFilePath(argv[1]);
        fileHandler.setFileName(argv[2]);
    }

    std::vector<uint32_t> device_indices;
    for (uint32_t i = 0; i < num_devices; i++) {
        device_indices.push_back(i);
    }

    // Open each of the existing devices
    MultiDeviceCapturer capturer(device_indices, color_exposure_usec, powerline_freq);

    // Create configurations for the different devices
    k4a_device_configuration_t mainConfig = getMasterConfig();
    k4a_device_configuration_t secondaryConfig = getSubordinateConfig();

    // Start all opened devices
    capturer.startDevices(mainConfig, secondaryConfig);

    if (num_devices > 1 ){
        for (int i = 0; i < num_devices - 1; i++) {
            TransformationOpenCV transfSecToMain = calibrateDevices(capturer,
                mainConfig,
                secondaryConfig,
                chessboard_pattern,
                chessboard_square_length,
                calibration_timeout, i, i+1);
            fileHandler.registerTransformationIntoFile(transfSecToMain);
        }
    } 
    else
    {
        std::cerr << "Invalid number of devices! (must be more than 1)" << std::endl;
        exit(1);
    }

    return 0;
}
