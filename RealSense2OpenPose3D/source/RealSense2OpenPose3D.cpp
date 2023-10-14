//RealSense to OpenPose 3D "RealSense2OpenPose3D" 
//By Aidan Fox-Tierney, foxtier@atr.jp, June 29, 2022
//
//Frame reconstruction section based on solution by OldAccountFJShen: June 9, 2020 (Thank you so much)
//  Source: https://github.com/IntelRealSense/librealsense/issues/6522#issuecomment-641027578
//Overall project assisted greatly by the Intel examples, in particular: https://dev.intelrealsense.com/docs/rs-align
//Getting real world coordinates used solutions in "Converting depth into 3D world coordinates intel realsense"
//  by simontab and jb455: https://community.intel.com/t5/Software-Archive/Converting-depth-into-3D-world-coordinates-intel-real-sense/m-p/1057958


/*
Program Outline:
    1. Parse command line arguments to control the OpenPose output folder path
    2. Start a normal stream and wait a handful of frames for the cameras to stabilize
    3. Save a color frame from that normal stream
    4. Stop that normal stream
    5. Start a new stream with the color sensor disabled
    6. Signal that the color camera is free to use
        a. OpenPose should be started now
    7. Start a virtual device stream
    8. On every frame, save the depth frame and inject it and the color frame into the virtual device
    9. Align the frameset in the virtual device
    10. Check the OpenPose output folder for any new frames (or the first frame)
        a. If there is a new frame, load the JSON file
        b. Read through the JSON file to get all the points that OpenPose found
        c. For each point, add the depth information for that pixel from the latest aligned depth frame
        d. Save the JSON file
        e. Remember that this file was already updated so ignore it next time
*/

#include <iostream>
#include <math.h> //for fmod()

#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <librealsense2/hpp/rs_internal.hpp>


#include "./json.hpp" //Send some thanks this way -> https://github.com/nlohmann/json
using json = nlohmann::json;

//Functions
bool checkCmdLine(int argNumber, char** argStrings); //Parse the command line arguments
void getBaselineFrameAndCameraValues(); //Save the camera parameters and one color frame
void setReady(); //Set the "ready" text file to tell the rest of the programs that this program is ready 
void updateKeypoints(const rs2::vertex* depth, int& frameNumber); //Inject depth info into output files
int f2i(double x); //Round floats to nearest integers
void press2Close(); //Simple wait for user input to close the program

int depthWidth = 1280; //Sensor resolutions
int depthHeight = 720;
int colorWidth = 1280;
int colorHeight = 720;

const rs2::vertex* depthVertices = new rs2::vertex[colorWidth * colorHeight]; //Allocate memory for an array of vertecies, one vertex for every pixel

//Initial frame and camera information
rs2_intrinsics colorIntrinsics;
rs2_intrinsics depthIntrinsics;
rs2_extrinsics depth2ColorExtrinsics;
rs2::frame baselineColorFrame;

std::string OpenPoseOutPath("..\\openPoseOutput"); //Default OpenPose output directory path

int main(int argc, char* argv[])
{
    if (checkCmdLine(argc, argv) != true) //If user defined OpenPose output dir path provided, use that, otherwise default path
    {
        press2Close();
        return -1; //Something was wrong that required the program to exit
    }

    getBaselineFrameAndCameraValues(); //Run the sensor briefly to collect the intrinsics, exrinsics, and a baseline color frame
    setReady();


    //Create a new pipeline to stream the depth data
    rs2::pipeline pipe; //Create a pipeline
    rs2::config cfg; //Set up the configuration of the camera
    cfg.enable_stream(RS2_STREAM_DEPTH, depthWidth, depthHeight, RS2_FORMAT_Z16, 30); //Full resolution and FPS so depth is always up-to-date
    cfg.disable_stream(RS2_STREAM_COLOR); //Note that the color stream is DISABLED so it can be opened by OpenPose
    pipe.start(cfg);


    // Create software device to allow for merging of old color image frame and current depth frame: Frame Reconstruction
    rs2::software_device dev;

    auto depth_sensor = dev.add_sensor("Depth"); // New virtual sensors
    auto color_sensor = dev.add_sensor("Color");

    //Add video streams to the virtual sensors so that they can be interacted with
    auto depth_stream = depth_sensor.add_video_stream(
        { RS2_STREAM_DEPTH, 0, 0, depthWidth, depthHeight, 30, 2, RS2_FORMAT_Z16,
         depthIntrinsics });
    auto color_stream = color_sensor.add_video_stream(
        { RS2_STREAM_COLOR, 0, 1, colorWidth, colorHeight, 30, 3, RS2_FORMAT_BGR8,
         colorIntrinsics });

    //Set read only option to fix issue of only adding one of the two frames. Note the depth units constant
    depth_sensor.add_read_only_option(RS2_OPTION_DEPTH_UNITS, 0.001f);
    depth_sensor.add_read_only_option(RS2_OPTION_STEREO_BASELINE, 0.001f);

    depth_stream.register_extrinsics_to(color_stream, depth2ColorExtrinsics); // Add camera extrinsics

    // Create the syncer that matches timestamps of both frames
    dev.create_matcher(RS2_MATCHER_DEFAULT);
    rs2::syncer sync;

    depth_sensor.open(depth_stream);
    color_sensor.open(color_stream);

    depth_sensor.start(sync);
    color_sensor.start(sync);

    rs2::align align(RS2_STREAM_COLOR); // Prepare to align the depth frames to color frames

    rs2::frameset fsAligned; // Declare a frameset and depth frame to hold onto the frames that we want to collect from the virtual device
    rs2::frame depthAligned;

    //Save some info about the actual video/depth frames
    void* colorPx = (void*)baselineColorFrame.get_data();
    int colorStride = ((rs2::video_frame)baselineColorFrame).get_stride_in_bytes();
    int colorBPP = ((rs2::video_frame)baselineColorFrame).get_bytes_per_pixel();

    int idx = 0; //A frame number used internally by the syncer
    int frameNumber = 0; //Corresponds to the file name to be read from OpenPose

    std::cout << "Starting Main frame injection loop...\n";

    //Forever
    while (true)
    {
        //Wait for a depth frame and then save it to "depth"
        rs2::frameset frameset = pipe.wait_for_frames();
        auto depth = frameset.get_depth_frame();


        color_sensor.on_video_frame({ colorPx, // Frame pixels from baseline color capture
                                     [](void*) {}, // Custom deleter (if required)
                                     colorStride, colorBPP, // Stride and Bytes-per-pixel
                                     depth.get_timestamp(), RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME,
                                     idx, // Timestamp, Frame# for potential sync services
                                     color_stream });
        depth_sensor.on_video_frame({ (void*)depth.get_data(), // Frame pixels from capture API
                                     [](void*) {}, // Custom deleter (if required)
                                     depth.get_stride_in_bytes(), depth.get_bytes_per_pixel(), // Stride and Bytes-per-pixel
                                     depth.get_timestamp(), RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME,
                                     idx, // Timestamp, Frame# for potential sync services
                                     depth_stream });

        
        fsAligned = sync.wait_for_frames();
        if (fsAligned.size() == 2) //If both a color and depth frame are ready
        {
            fsAligned = align.process(fsAligned); //Align the depth to the color frame
            rs2::frame depthAligned = fsAligned.get_depth_frame(); //Get the aligned depth frame

            //Generate a point cloud and a vertex array from that to hand to "updateKeypoints()" so that it can find the world coordinates
            rs2::pointcloud ptCld = rs2::pointcloud();
            rs2::points depthPoints = ptCld.calculate(depthAligned);
            depthVertices = depthPoints.get_vertices();

            updateKeypoints(depthVertices, frameNumber); //Add the depth information to any new files generated
        }
        idx++;
    }//forever

    return 0;
}//main()



//Determines if there are enough or too few aruments passed in the command line and ensures the directory is ready for use.
//  If there are any problems, the default OpenPose output directory is kept.
bool checkCmdLine(int argNum, char** argStrings)
{
    char expected[] = "Expected input: .\\RealSense2OpenPose3D.exe \"path\\to\\openpose\\output\\directory\"\n";
    char defaultPath[] = "Using the default OpenPose output directory path.\n";
    char outWillBe[] = "The OpenPose output directory will be \"";

    if (argNum > 1) //There are arguments
    {
        if (argNum > 2) // More than one argument
        {
            std::cout << "Too many arguments.\n" << expected << defaultPath << outWillBe << OpenPoseOutPath << "\"\n";
        }//If more than one argument

        OpenPoseOutPath.assign(argStrings[1]);
    }
    else //There were no arguements
    {
        std::cout << "No arguments detected\n";
    }

    std::cout << outWillBe << OpenPoseOutPath << "\"\n";

    //Check if the directory needs to be cleaned up first or can be accessed at all
    std::ifstream input(OpenPoseOutPath + "\\000000000000_keypoints.json"); //open the first keypoint file created by OpenPose
    std::ifstream readyInput(OpenPoseOutPath + "\\ready.txt"); //open a file named "ready.txt"

    /* Allow opening old files
    if (!input.fail()) //If the file open succedded, there must be old files there!
    {
        std::cout << "There are old keypoint files in \"" << OpenPoseOutPath << "\" still. Please remove them or select another directory.\n";
        return false;
    }*/

    if (readyInput.fail()) //If opening the "ready" file failed
    {
        std::ofstream readyOutput(OpenPoseOutPath + "\\ready.txt");
        if (readyOutput.fail()) //If the file opening failed
        {
            std::cout << "The specified directory \"" << OpenPoseOutPath << "\"could not be written to. Please ensure that you have proper permissions and wrote the path correctly.\n";
            return false;
        }
        else
        {
            readyOutput << "false"; //Set the default value of the file to be "false" since this program is not ready for OpenPose to start yet
        }
    }
    else
    {
        readyInput.ignore(15); // Clear the old contents of the file
        readyInput.close();

        std::ofstream readyOutput(OpenPoseOutPath + "\\ready.txt");
        readyOutput << "false"; //Set the default value of the file to be "false" since this program is not ready for OpenPose to start yet
        readyOutput.close();
    }

    return true;
}//checkCmdLine()



//Runs the camera for a handful of frames until the exposure stabilizes and then collects the
//  intrinsics, extrinsics, and a single color frame as a baseline for future alignment.
void getBaselineFrameAndCameraValues()
{
    std::cout << "Capturing baseline...\n";

    rs2::pipeline pipe; // Create a pipeline

    rs2::config cfg; //set up the configuration of the camera
    cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30); //Full resolution and max FPS to speed up process
    cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30); //Note that the color stream is ENABLED, matched FPS with depth

    pipe.start(cfg);

    for (int i = 0; i < 30; i++) //throw out first ~1 sec of frames (30FPS * 30 = 1sec)
    {
        pipe.wait_for_frames(); //get a frame (but don't bother saving it)
    }// for ~1 second of frames

    rs2::frameset frameset = pipe.wait_for_frames(); //save a frameset

    //Get the separate color and depth frames
    auto depth = frameset.get_depth_frame();
    auto color = frameset.get_color_frame();

    //Get the camera intrinsics and save them to the global variables
    depthIntrinsics = depth.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
    colorIntrinsics = color.get_profile().as<rs2::video_stream_profile>().get_intrinsics();

    //Get the depth camera to color camera extrinsics and save them to the global variable
    depth2ColorExtrinsics = depth.get_profile().get_extrinsics_to(color.get_profile());

    color.keep(); //Retain this frame, by default, old frames are overwritten in memory
    baselineColorFrame = color; //Remember that this is actually an rs::video_frame

    pipe.stop();

    std::cout << "Baseline captured.\n";
}//getBaselineFrameAndCameraValues()



//Sets the ready file to true so that OpenPose knows it can use the color camera
void setReady()
{
    std::ifstream readyInput(OpenPoseOutPath + "\\ready.txt");
    readyInput.ignore(15); //Clear the old contents of the file

    std::ofstream readyOutput(OpenPoseOutPath + "\\ready.txt");
    readyOutput << "true"; //Insert "true" into the ready.txt file
}//setReady()



//Updates the keypointfiles generated by OpenPose with depth data when there is a new frame
void updateKeypoints(const rs2::vertex* depth, int& frameNumber)
{
    //Generate the new filename
    std::string fileName = "\\000000000000_keypoints.json";
    std::string digitString = std::to_string(frameNumber);
    int digitNumber = digitString.length(); //Get the length of the frame number

    fileName.replace(13 - digitNumber, digitNumber, digitString); //replace the zeros on the right with the frame number


    //Check if new file (i.e. a new frame)
    std::ifstream keyframeFile(OpenPoseOutPath + fileName);
    if (keyframeFile.good() == 0) //If file not able to be opened
    {
        return; //Do nothing
    }
    else //Else the file was loaded, so fill the depth data
    {
        //Load the file into a JSON object, iterate through all people, iterate through all joints, and update each with the corresponding depth
        //Insert the file into a JSON object for easy handling
        json jsn;
        try //Sometimes the files are opened too soon so the JSON interpreter throws an exception
        {
            keyframeFile >> jsn;
        }
        catch (const json::exception& e)
        {
            std::cout << "Likely an empty file. File Name: " << fileName << "\n";
            std::cerr << "JSON threw an exception: " << e.what() << "\n" << "ExceptionID: " << e.id << std::endl;
            return;
        }
        keyframeFile.close(); //close the file (since we will need to write to it shortly)

        unsigned int i, j;
        int x, y; //Iterators and temp keypoint pixel information
        double confidence; //Temp confidence value
        double tempPose3d[25 * 4], tempFace3d[69 * 4], tempLeftHand3d[21 * 4], tempRightHand3d[21 * 4]; //3D pose and face arrays to hold temp values to insert into each file
        rs2::vertex tempVertex;
        bool insertFace = true; //Assume that there are face keypoints
		bool insertHand = true; //...and hand keypoints

        for (i = 0; i < jsn["people"].size(); i++) //For all people
        {
            //Update body keypoints
            for (j = 0; j < 25; j++) //for all 25 pose keypoints
            {
                //Get the x and y coordinates of each keypoint
                x = f2i(jsn["people"][i]["pose_keypoints_2d"][3 * j]);
                y = f2i(jsn["people"][i]["pose_keypoints_2d"][3 * j + 1]);
                confidence = jsn["people"][i]["pose_keypoints_2d"][3 * j + 2];

                if (x > 0 && y > 0 && x < 1280 && y < 720) // if keypoint exists and within possible ranges
                {
                    //Set the x, y, and depth
                    tempVertex = depth[y * colorWidth + x];
                    tempPose3d[4 * j] = tempVertex.x;
                    tempPose3d[4 * j + 1] = tempVertex.y;
                    tempPose3d[4 * j + 2] = tempVertex.z;
                    tempPose3d[4 * j + 3] = confidence;
                }
                else //If there was no keypoint
                {
                    //Set the values to 0
                    tempPose3d[4 * j] = 0;
                    tempPose3d[4 * j + 1] = 0;
                    tempPose3d[4 * j + 2] = 0;
                    tempPose3d[4 * j + 3] = 0; // "Confidence" = 0
                }
            }//For all 25 body keypoints
			
			try{
				//Left Hand
				for (j = 0; j < 21; j++) //for all 21 pose keypoints
				{
					//Get the x and y coordinates of each keypoint
					x = f2i(jsn["people"][i]["hand_left_keypoints_2d"][3 * j]);
					y = f2i(jsn["people"][i]["hand_left_keypoints_2d"][3 * j + 1]);
					confidence = jsn["people"][i]["hand_left_keypoints_2d"][3 * j + 2];

					if (x > 0 && y > 0 && x < 1280 && y < 720) // if keypoint exists and within possible ranges
					{
						//Set the x, y, and depth
						tempVertex = depth[y * colorWidth + x];
						tempLeftHand3d[4 * j] = tempVertex.x;
						tempLeftHand3d[4 * j + 1] = tempVertex.y;
						tempLeftHand3d[4 * j + 2] = tempVertex.z;
						tempLeftHand3d[4 * j + 3] = confidence;
					}
					else //If there was no keypoint
					{
						//Set the values to 0
						tempLeftHand3d[4 * j] = 0;
						tempLeftHand3d[4 * j + 1] = 0;
						tempLeftHand3d[4 * j + 2] = 0;
						tempLeftHand3d[4 * j + 3] = 0; // "Confidence" = 0
					}
				}//For left hand 21 keypoints

				//Right Hand
				for (j = 0; j < 21; j++) //for all 21 pose keypoints
				{
					//Get the x and y coordinates of each keypoint
					x = f2i(jsn["people"][i]["hand_right_keypoints_2d"][3 * j]);
					y = f2i(jsn["people"][i]["hand_right_keypoints_2d"][3 * j + 1]);
					confidence = jsn["people"][i]["hand_right_keypoints_2d"][3 * j + 2];

					if (x > 0 && y > 0 && x < 1280 && y < 720) // if keypoint exists and within possible ranges
					{
						//Set the x, y, and depth
						tempVertex = depth[y * colorWidth + x];
						tempRightHand3d[4 * j] = tempVertex.x;
						tempRightHand3d[4 * j + 1] = tempVertex.y;
						tempRightHand3d[4 * j + 2] = tempVertex.z;
						tempRightHand3d[4 * j + 3] = confidence;
					}
					else //If there was no keypoint
					{
						//Set the values to 0
						tempRightHand3d[4 * j] = 0;
						tempRightHand3d[4 * j + 1] = 0;
						tempRightHand3d[4 * j + 2] = 0;
						tempRightHand3d[4 * j + 3] = 0; // "Confidence" = 0
					}
				}//For right hand 21 keypoints
			}
			catch{
				insertHand = false;
			}


            try //Sometimes there are no face keypoints
            {
                for (j = 0; j < 69; j++) //For all 69 Face keypoints (Nice)
                {
                    //Get the x and y coordinates of each keypoint
                    x = f2i(jsn["people"][i]["face_keypoints_2d"][3 * j]);
                    y = f2i(jsn["people"][i]["face_keypoints_2d"][3 * j + 1]);
                    confidence = jsn["people"][i]["face_keypoints_2d"][3 * j + 2];

                    if (x > 0 && y > 0 && x < 1280 && y < 720) // if keypoint exists
                    {
                        //Set the x, y, and depth
                        tempVertex = depth[y * colorWidth + x];
                        tempFace3d[4 * j] = tempVertex.x;
                        tempFace3d[4 * j + 1] = tempVertex.y;
                        tempFace3d[4 * j + 2] = tempVertex.z;
                        tempFace3d[4 * j + 3] = confidence;
                    }
                    else //If there was no keypoint
                    {
                        //Set the values to 0
                        tempFace3d[4 * j] = 0;
                        tempFace3d[4 * j + 1] = 0;
                        tempFace3d[4 * j + 2] = 0;
                        tempFace3d[4 * j + 3] = 0; // "Confidence" = 0
                    }
                }//For all 96 Face keypoints
            }
            catch(...)
            {
                insertFace = false;
            }

            //Insert the 3D points into this person
            jsn["people"][i]["pose_keypoints_3d"] = tempPose3d;
            if (insertFace)
            {
                jsn["people"][i]["face_keypoints_3d"] = tempFace3d;
            }    
			if (insertHand)
			{
				jsn["people"][i]["hand_left_keypoints_3d"] = tempLeftHand3d;
				jsn["people"][i]["hand_right_keypoints_3d"] = tempRightHand3d;
			}
        }//For all people

        //Save the updated file
        fileName.insert(23, sizeof(char), 'D'); //Place a D for depth/done at the end of the file name
        std::ofstream output(OpenPoseOutPath + fileName);
        output << std::setw(4) << jsn << std::endl;
        output.close();

        frameNumber++;
    }//Else the file was loaded
}//updateKeypoints()


//Converts floats to their nearest integer value
int f2i(double x)
{
    if (fmod(x, 1.0) >= 0.5)
    {
        return (int)x + 1; //Round up
    }
    else
    {
        return (int)x; //Round Down
    }
}



//Accepts anything as input to pause the program so that error messages can be read
void press2Close()
{
    std::cout << "\nPress enter to close this window...\n";
    if (std::cin.get() != '\n')
    {
        std::cout << ";)";
    }
}//press2Close()
