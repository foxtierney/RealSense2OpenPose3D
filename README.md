﻿<div align="center">
    <img src="./icon.png" width="600">
</div>

# RealSense2OpenPose3D

This project provides a simple way to use an [**Intel RealSense depth camera**](https://www.intelrealsense.com/depth-camera-d435/) with [**OpenPose**](https://github.com/CMU-Perceptual-Computing-Lab/openpose) to get 3D keypoints. It was first developed for a Master's project while doing an internship at [Advanced Telecommunications Research Institute International (ATR)](https://www.atr.jp/index_e.html).

* [Use](https://github.com/foxtierney/RealSense2OpenPose3D#use)
* [Installation](https://github.com/foxtierney/RealSense2OpenPose3D#installation)
	- [Install RealSense SDK](https://github.com/foxtierney/RealSense2OpenPose3D#install-realsense-sdk)
	- [Install OpenPose](https://github.com/foxtierney/RealSense2OpenPose3D#install-openpose)
	- [Download RealSense2OpenPose3D exe](https://github.com/foxtierney/RealSense2OpenPose3D#download-realsense2openpose3d-exe)
	- [Compile RealSense2OpenPose3D](https://github.com/foxtierney/RealSense2OpenPose3D#compile-realsense2openpose3d)
* [Tips](https://github.com/foxtierney/RealSense2OpenPose3D#tips)

## Use
1. After [installing all required components and either compiling or downloading OpenPose2RealSense3D](https://github.com/foxtierney/RealSense2OpenPose3D#installation), edit the default paths in the `launch.py` file to match your layout.
	1. In particular, change `openPosePath`, `openPoseOutputPath`, `RealSense2OpenPoseEXE`, and `PointViewer`.
2. Run the program by entering `python .\launch.py` into a console where the launch file is located.
3. Arguments (do not enter spaces after the '='):
	1. `frames=` number >= -1. Is the number of frames beyond 10 that will not be deleted during run time (-1 is save all)
	2. `view=` True or false. Whether to start the point viewer or not
	3. `quit=` An alphanumeric character. This determines which keyboard key will terminate the program
	4. `d=` float number >= 0. The depth limit beyond which values are ignored
	5. `lr=` float number >= 0`,`float number >= 0. The right and left limits of the point viwer in meters
	6. `ud=` float number >= 0`,`float number >= 0. The up and down limits of the point viwer in meters
	7. `color-res=` integer number `x` integer number. The resolution of the color sensor of the camera, defaults to 1920x1080
	8. `face=` True or False. Detect hands or not, defaults to false
	9. `hand=` True or False. Detect face or not, defaults to false
	10. `output=` <`path\to\openPoseOutputFolder`>. The full path to the OpenPose output folder you would like to use.
	11. `r2oexe=` <`path\to\RS2OP3D.exe`> The full path to and including the RS2OP3D.exe file.
	12. Other input will yield the help menu
	13. Example: `python .\launch.py frames=-1 view=true quit=q d=2.5 lr=1.5,1.5 ud=1,1 color-res=1280x720 face=false hand=true output=C:\Users\Bingus\Desktop\output r2oexe=C:\Users\Bingus\Desktop\RealSense2OpenPose3D\64bit\RS2OP3D.exe`
4. The output files will be marked as `############_keypointsD.json` in the output folder
5. See [**OpenPose's documentation**](https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/02_output.md) for the format of the output JSON files

## Installation

This guide will walk you through all required components.

### Install RealSense SDK:
1. [Download RealSense SDK v2.34](https://github.com/IntelRealSense/librealsense/releases/tag/v2.34.0)
    1.	Note: The current version at the time of writing this is v2.50.0. However, there is a bug that prevents the depth and image alignment using “software devices” that was introduced in some build after v2.34.0
    2.	See [this issue](https://github.com/IntelRealSense/librealsense/issues/4523) for more info

### Install OpenPose:
1.	Prerequisites: [Prerequisite List](https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/installation/1_prerequisites.md)
    1.	CMake GUI
        1.	[CMake download page](https://cmake.org/download/)
		2.	Download and install `cmake-#.#.#-win64-x64.msi`
	2.	Install Microsoft Visual Studio Community 2019
		1.	[Visual Studio download page](https://archive.org/details/vs_Community)
		3.	Run the installer after downloading the executable
		4.	Select the C++ console option and then all checkboxes on the right that say `C++` in them
		5.	Click `Install`
		6.	Restart your computer
	3.	Install CUDA and CuDNN
		1.	*Note:* You must wait until after installing Visual Studio before proceeding to this step!
		2.	Install CUDA 11.11 for 30 series GPUs [CUDA download page](https://developer.nvidia.com/cuda-11.1.1-download-archive?target_os=Windows&target_arch=x86_64&target_version=10&target_type=exenetwork)
		3.	[CuDNN download page](https://developer.nvidia.com/rdp/cudnn-download)
		4.	Merge CuDNN files with `C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v11.1`
	4.	Install python: [Python download page](https://www.python.org/downloads/windows/)
		1.	Install open-cv:
		2.	In an admin powershell run: `pip install numpy opencv-python`
2.	Clone OpenPose:
	1.	Create a new folder `C:/Program Files/OpenPose`
	2.	With an administrator powershell run the following commands
		<pre><code>git clone https://github.com/CMU-Perceptual-Computing-Lab/openpose
		cd openpose/
		git submodule update --init --recursive --remote</code></pre>
3.	CMake Configuration
	1.	Enter the `OpenPose/openpose` directory
	2.	Make a new folder named “build”
	3.	Enter that new folder
	4.	Run CMake: `cmake-gui ..`
	5.	Make sure that the source code path field is `…OpenPose/openpose` and that the build directory is `…/OpenPose/openpose/build`
	6.	Click “Configure”
	7.	Select the version of Visual Studio that is installed and select x64
	8.	Click “Finish”
		1. If the model downloads fail (should not take long since files are ~100-150Mb each), need to manually install model files
		2. The models can be found in [OpenPoseDependancies/models](https://github.com/foxtierney/RealSense2OpenPose3D/tree/master/OpenPoseDependancies/models)
		3. Download all of them to an non-admin accessable folder and uncompress them using 7Zip by `Right-click ...7z.001 > 7Zip > Extract Files here`
		4. Merge the now uncompressed `face, hand, pose` folders into  `.../OpenPose/openpose/models`
		5. If the windows dependancy downloads fail, repeat the above steps b-d but with the files in [OpenPoseDependancies/3rdPartyWindows](https://github.com/foxtierney/RealSense2OpenPose3D/tree/master/OpenPoseDependancies/3rdPartyWindows) and placing the `.zip` folders into `...openpose/3rdparty/windows`
		6. Extract the contents (not parent .zip folder itself) of each .zip folder into the `.../3rdparty/windows` folder that the .zip folders are now in. 
	9.	Make sure that the GPU mode is set to CUDA, WITH_3D_RENDERER is on, and WITH_FLIR_CAMERA is off
	10.	Click “Configure” one more time
	11.	Click “Generate”
4.	Compilation:
	1.	Click on “Open Project” to open the Visual Studio solution
	2.	Switch the configuration from “Debug” to “Release”
	3.	Press “Ctrl+Shift+B” (Build)
	4.	Copy all the .dll files from `…/build/bin` to `…/build/x64/release`
5.	Test that it works
	1.	Go to …/OpenPose/openpose/
	2.	Run an example like: `build/x64/Release/OpenPoseDemo.exe --video examples/media/video.avi`
	3.	Using a camera: `build/x64/Release/OpenPoseDemo.exe --hand --face --camera 1`

### Download RealSense2OpenPose3D exe
This is the easiest way to get up and running.
1. [Download here](https://github.com/foxtierney/RealSense2OpenPose3D/releases/tag/1.3)
2. Place the whole folder, including all the `.dll` files, where you would like.

### Compile RealSense2OpenPose3D
1.	Create a new empty C++ Project in Visual Studio
2.	Add the Intel RealSenseSDK 2.0 Property sheets
	1.	View -> Other Windows -> Property Manager
	2.	Right click on project name in the window that just opened
	3.	Add existing property sheet
	4.	Navigate to the SDK directory `“C:\Program Files (x86)\Intel RealSense SDK 2.0”` in my case
	5.	Select one of the `.props` files and click Open
	6.	Repeat for the other two `.props` files
3.	Test to see if it all works
	1.	Find `“rs-hello-realsense.cpp”` under `“Intel RealSense SDK 2.0\samples\hello-realsense”`
	2.	Add the file to the project
		1.	Right click on Source Files in the Solution Explorer
		2.	Add -> Existing Item
		3.	Select `“rs-hello-realsense.cpp”` and click Add
	3.	Run the program
		1.	Click on the green arrow at the top of the IDE
4.	Take a break. It wasn’t terrible, but figuring out how to do this wasn’t easy either.
5.	Download the source for RealSense2OpenPose3D
	1.	Download from [here](https://github.com/foxtierney/RealSense2OpenPose3D/blob/main/RealSense2OpenPose3D/source)
6.	Download the JSON library
	1.	Go to [JSON.hpp download](https://github.com/nlohmann/json/releases) or use the version included in the `"source"` folder from the previous step
	2.	Download the latest “json.hpp”
	3.	Send some thanks in the direction of the creators
	4.	Save the file in your working directory for the project and double check that the #include statement in RealSense2OpenPose.cpp has the correct path
7. Compile!

## Tips
* If you are not getting the framerate you want
	- This program can run at a maximum of ~40fps, if the pose detection exceeds this, the program will crash. OpenPose's framerate can be limited in the `launch.py` file if this happens.
	- You can increase the speed of OpenPose by installing a better or second GPU
	- In the `launch.py` file, you may alter the OpenPose launch flags to reduce the computational load
* The output files are disapearing
	- The `launch.py` program automatically deletes the output files as it runs to prevent filling up your hard drive
	- If you would like to keep all files, run the launch file with the argument `frames=-1`
	- If you would like to keep only the past `#` many, and be prompted to delete them or not at the end, then run the launch file with the argument `frames=#`
* While typing something, the program stops
	- The program has `q` as the default quit key.
	- This can be changed with the `quit=<key name>` argument to the launch file.


