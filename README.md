# AzureKinectCalibration
Project that permits to calibrate Azure Kinect together

Program based on the green screen example from Azure Kinect documentation : [Link to the repository](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/tree/develop/examples/green_screen)

Program only tested on Windows.

To use the program : 
- need to install Azure Kinect SDK v.1.4.1 : [Download link](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/docs/usage.md)
- need to install OpenCV v.4.5.2 : [Download link](https://sourceforge.net/projects/opencvlibrary/files/4.5.2/opencv-4.5.2-vc14_vc15.exe/download)

Need to set the following global variables :
OPENCV_PATH : to 'opencv' directory
KINECT_SDK : to 'Azure Kinect SDK v1.4.1' directory

Need to put the following dll's next to the .exe file :  k4a.dll, k4arecord.dll, depthengine_2_0.dll, opencv_world452.dll, opencv_world452d.dll.

Launch the solution under Realease mode on Visual Studio 2019.

Example of how to use the FileHandler class :

```cpp

    // Open the file 'file.txt' into the current folder
    FileHandler fileHandler(".\\", "file");
    // Reset the file
    fileHandler.resetFile();

    // Add new transformations inside the file (with OpenCV format)
    fileHandler.registerTransformationIntoFile(0, TransformationOpenCV());
    fileHandler.registerTransformationIntoFile(0, TransformationOpenCV(
        cv::Matx33d(22, 10, -97, 
            13, -25, 54, 
            178, 245, -25), 
        cv::Vec3d(12, 1.83, -6.37)
    ));
    fileHandler.registerTransformationIntoFile(0, TransformationOpenCV(
        cv::Matx33d(1, 82, -97,
            10, 20, 54,
            14, 24, 25),
        cv::Vec3d(2, 3, 6)
    ));

    // Get what's inside the file (in OpenCV format) and print it
    std::vector<TransformationOpenCV> result = fileHandler.getOpenCVTransformationsFromFile();
    int index = 0;
    for (const TransformationOpenCV& trans : result) {
        std::cout << "Index device : " << index++ << std::endl;
        std::cout << "Rotation matrix : " << trans.R << std::endl;
        std::cout << "Translation vector : " << trans.t << std::endl;
    }

    index = 0;
    // Get what's inside the file (in C++ format) and print it
    std::vector<Transformation> result2 = fileHandler.getTransformationsFromFile();
    for (auto& const value : result2) {
        std::cout << "Index device : " << index++ << std::endl;
        std::cout << "Rotation matrix :";
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                std::cout << " " << value.R[i][j];
            }
        }
        std::cout << "Translation vector : " << value.t[0] << " " << value.t[1] << " " << value.t[2] << std::endl;
    }
```