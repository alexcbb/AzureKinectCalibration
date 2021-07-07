# AzureKinectCalibration Project

### 1) Introduction
The AzureKinectCalibration project permits to get the extrinsic parameter from several Azure Kinect (that are synced together)
and to write those parameters in a file (so as to use this datas in another project).
The idea is to know where each camera are positioned in relation with other into 3D space.

A big part of the project is based on the green screen example from Azure Kinect documentation : [Link to the repository](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/tree/develop/examples/green_screen)
This program was only tested on Windows.

### 2) How to use 

##### 2.1 Visual studio project :
- need to install Azure Kinect SDK v.1.4.1 : [Download link](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/docs/usage.md)
- need to install OpenCV v.4.5.2 : [Download link](https://sourceforge.net/projects/opencvlibrary/files/4.5.2/opencv-4.5.2-vc14_vc15.exe/download)

Need to set the following global variables :
- OPENCV_PATH : to 'opencv' directory
- KINECT_SDK : to 'Azure Kinect SDK v1.4.1' directory

Need to put the following dll's next to the .exe file :  k4a.dll, k4arecord.dll, depthengine_2_0.dll, opencv_world452.dll, opencv_world452d.dll.

Launch the solution under Realease mode and under x64 solution on Visual Studio 2019.

TODO : add just a .exe file to be able to launch the app with command-line

