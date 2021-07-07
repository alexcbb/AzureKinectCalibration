#include "FileHandler.h"

FileHandler::FileHandler(std::string filePath, std::string fileName) : filePath(filePath), fileName(fileName) {}

void FileHandler::registerTransformationIntoFile(int deviceIndex, TransformationOpenCV tr) {
	// ios::app permits to just add the datas after the one already written into the file 
	std::ofstream file(filePath + "\\" + fileName + ".txt", std::ios::app);
	if (file.is_open()) {
		// Each line corresponds to one device, beginning from the device 0; Format for each device : 
		// [r11] [r12] [r13], .... [r32] [r33]; [tx] [ty] [tz]  
		file << tr.R(0, 0) << " " << tr.R(0, 1) << " " << tr.R(0, 2) << " "
			<< tr.R(1, 0) << " " << tr.R(1, 1) << " " << tr.R(1, 2) << " "
			<< tr.R(2, 0) << " " << tr.R(2, 1) << " " << tr.R(2, 2) << "; "
			<< tr.t[0] << " " << tr.t[1] << " " << tr.t[2] << std::endl;
		file.close();
	}
	else {
		std::cerr << "Error while trying to open file !" << std::endl;
	}
}

void FileHandler::registerTransformationIntoFile(int deviceIndex, cv::Matx33d R, cv::Vec3d t) {
	this->registerTransformationIntoFile(deviceIndex, TransformationOpenCV(R, t));
}


void FileHandler::resetFile() {
	std::ofstream file(filePath + "\\" + fileName + ".txt", std::ofstream::out | std::ofstream::trunc);
	if (file.is_open()) {
		file.close();
	}
	else {
		std::cerr << "Error while trying to open file !" << std::endl;
	}
 }

std::vector<TransformationOpenCV> FileHandler::getOpenCVTransformationsFromFile() {
	std::vector<TransformationOpenCV> result;
	std::ifstream file(filePath + "\\" + fileName + ".txt");
	if (file) {
		std::string line;

		while (getline(file, line)) {
			int pos = 0;
			std::string token;
			std::vector<double> values;
			while ((pos = line.find(" ")) != std::string::npos) {
				token = line.substr(0, pos);
				values.push_back(std::stod(token));
				line.erase(0, pos + 1);
			}
			values.push_back(std::stod(line));
			if (values.size() > 0) {
				cv::Matx33d Rot(
					values[0], values[1], values[2],
					values[3], values[4], values[5],
					values[6], values[7], values[8]);
				cv::Vec3d trans(values[9], values[10], values[11]);
				result.push_back(TransformationOpenCV(Rot, trans));
			}
			else {
				std::cerr << "Error : Not enough values for a given device !" << std::endl;
			}
		}
	}
	else {
		std::cerr << "Error : Could not open file !" << std::endl;
	}
	return result;
}


std::vector<Transformation> FileHandler::getTransformationsFromFile() {
	std::vector<Transformation> result;
	std::ifstream file(filePath + "\\" + fileName + ".txt");
	if (file) {
		std::string line;

		while (getline(file, line)) {
			int pos = 0;
			std::string token;
			std::vector<double> values;
			Transformation trans;
			while ((pos = line.find(" ")) != std::string::npos) {
				token = line.substr(0, pos);
				values.push_back(std::stod(token));
				line.erase(0, pos + 1);
			}
			values.push_back(std::stod(line));
			if (values.size() > 0) {
				std::vector<double> rot;
				for (int i = 0; i < 9; i++) {
					rot.push_back(values[i]);
				}
				trans.setRotationMatrix(rot);
				trans.t[0] = values[9];
				trans.t[1] = values[10];
				trans.t[2] = values[11];
				result.push_back(trans);
			}
			else {
				std::cerr << "Error : Not enough values for a given device !" << std::endl;
			}
		}
	}
	else {
		std::cerr << "Error : Could not open file !" << std::endl;
	}
	return result;
}
