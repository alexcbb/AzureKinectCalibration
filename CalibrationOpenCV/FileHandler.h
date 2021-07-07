#pragma once
#include <string>
#include <iostream>
#include <fstream>
#include "TransformationOpenCV.h"
#include "Transformation.h"

/**
* class FileHandler
* 
* This class aims to be able to write OpenCV transformations obtained after calibration into a '.txt' file.
* It also permits to read the file in order to get the Transformations into OpenCV and also classical C++ format.
*/
class FileHandler
{
public:
	FileHandler() : filePath(".\\"), fileName("file") {};
	FileHandler(std::string filePath, std::string fileName);

	// Write Transformation into file
	void registerTransformationIntoFile(TransformationOpenCV tr);
	void registerTransformationIntoFile(cv::Matx33d R, cv::Vec3d t);

	// Reset file
	void resetFile();

	/**
	* Get the transformation of all devices in an OpenCV type struct
	* 
	* @return : a vector containing the tranform of each device
	*/
	std::vector<TransformationOpenCV> getOpenCVTransformationsFromFile();

	/**
	* Get the transformation of all devices in a classic C++ structure
	* 
	* @return : a vector containing tuples which are composed of the matrix of rotation (in form of a vector : [r11, r12, r13, ...., r32, r33]) and
	* the vector of translation (which is a tuple containing 3 values : [tx, ty, tz])
	*/
	std::vector<Transformation> getTransformationsFromFile();

	void setFilePath(std::string filePath) {
		this->filePath = filePath;
	}
	void setFileName(std::string fileName) {
		this->fileName = fileName;
	}

private:
	std::string filePath;
	std::string fileName;
};

