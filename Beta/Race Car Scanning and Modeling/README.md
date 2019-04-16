## Getting started

- Download the latest version of the ZED SDK on [stereolabs.com](https://www.stereolabs.com).
- For more information, read the ZED [API documentation](https://www.stereolabs.com/developers/documentation/API/).

### All Prerequisite Software

- Windows 10 64bits
- [ZED SDK](https://www.stereolabs.com/developers/release/#sdkdownloads_anchor)
- CUDA 10.0 from Nvidia [https://developer.nvidia.com/cuda-downloads]
- Cmake 3.14.2 [https://cmake.org/download/]
- OpenCV 3.4.6 [https://opencv.org/releases.html]


## Build the program

#### Setup Overview for Windows

- Install Cuda 10.1
- Install ZED SDK for Windows
- Install Cmake for all users
- Install Visual Studio Community 2017
- Extract Opencv to c:/ (if this is anyware else please adjust next two instructions accordingly) 
- Open Administrator console and type "setx -m OPENCV_DIR C:\OpenCV\Build\x64\vc14" & "setx -m opencv C:\opencv\build\x64\vc14\bin"
- Go to System Variables and edit your Path with the following two paths: C:\opencv\build\include\opencv2 & C:\opencv\build\x64\vc14\bin


#### Build for Windows

- Create a "build" folder in the source folder
- Open cmake-gui and select the source and build folders
- Generate the Visual Studio 2017 `Win64` solution
- Open the resulting solution and change configuration to `Release`
- Open PRE_VISION_MAIN Property Pages(right click on PRE_VISION_MAIN in solution exporer)
- In C/C++ -> General -> Additianal Include Directories: Click the drop down and edit then Add C:\opencv\build\include
- In Linker -> All Options -> Additional Dependencies: Add opencv_world[VERSON].lib (the virsion can be found in C:\opencv\build\x64\vc14\lib)
- In Linker -> All Options -> Additional Library Directories: Add C:\opencv\build\x64\vc14\lib
- Build solution
