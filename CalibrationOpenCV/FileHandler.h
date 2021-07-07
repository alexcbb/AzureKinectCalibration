#pragma once
#include <string>
#include <iostream>
#include <fstream>
#include "TransformationOpenCV.h"
#include "Transformation.h"

class FileHandler
{
public:
	FileHandler(std::string filePath, std::string fileName);

	void registerTransformationIntoFile(int deviceIndex, TransformationOpenCV tr);
	void registerTransformationIntoFile(int deviceIndex, cv::Matx33d R, cv::Vec3d t);
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
private:
	std::string filePath;
	std::string fileName;
};

