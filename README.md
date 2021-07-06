# AzureKinectCalibration
Program that permits to calibrate Azure Kinect together

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