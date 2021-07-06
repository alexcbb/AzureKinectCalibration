#pragma once
#include <string>
#include <iostream>
#include <fstream>
#include "Transformation.h"

class FileHandler
{
public:
	FileHandler(std::string filePath, std::string fileName);

	void registerTransformationIntoFile(int deviceIndex, Transformation tr);
	void registerTransformationIntoFile(int deviceIndex, cv::Matx33d R, cv::Vec3d t);
	void resetFile();
	std::vector<Transformation> getTransformationsFromFile();
	std::tuple<std::vector<float>, std::tuple<float, float, float>> getRotAndTransFromFile();
private:
	std::string filePath;
	std::string fileName;
};

