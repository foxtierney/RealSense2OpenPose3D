# Point Viewer
# By Aidan Fox-Tierney, foxtier@atr.jp, June 23, 2022
# 
# Displays the output from RealSense to OpenPose 3D as a point cloud
# Made with help from:
#  "python read json" by pythonbasics.com: https://pythonbasics.org/read-json-file/
#  "3D plotting in Python using matplotlib" by Shahbaz Khan on Like Geeks: https://likegeeks.com/3d-plotting-in-python/
#  "Live Updating Graphs with Matplotlib Tutorial" by PythonPrograming.net: https://pythonprogramming.net/python-matplotlib-live-updating-graphs/

import matplotlib.pyplot as plt #Needed for ploting the points
from matplotlib.animation import FuncAnimation

import sys #Get command line arguments
import time
import os #For checking if next file exists or not yet

import json #Used to read the JSON output files


#Global variables
outputPath = ""
fileEnd = "_keypointsD.json"

frameNumber = 0 #Keeps track of what file should be read

depthLim = 2.5 #Limit to how far the viewer will display depth points
upDownLim = [-1, 1] #Limit on how far above and below the camera to render [-above, below]
leftRightLim = [-1.5, 1.5] #Limit on how far to the left and right of the camera (from the camera's perspective) to render [-left, right]

#Create an empty list for each dimention
pointcloudX = []
pointcloudY = []
pointcloudZ = []

#Set up plot area
fig = plt.figure(figsize=(4,4)) #4x4grid (Apparently is in inches)
ax = fig.add_subplot(111, projection='3d') #Object for adding plots to the grid


#Define Parse Args
#Checks and parses the command line arguments to get the file path and any plot limits
#cmds: the list of commandline arguments (sys.argv)
def parseArgs(cmds):
    global outputPath
    global depthLim
    global upDownLim
    global leftRightLim
    
    expected = "Expected: \"...\\PointViwer.py path\\to\\output\n\t[d=(depth limit in meters)]\n\t[lr=(left limit in meters),(right limit in meters)]\n\t[ud=(up above limit in meters),(down belown limit in meters)]"

    lenCmds = len(cmds)
    
    #Check for 2-5 arguments
    if lenCmds < 2 or lenCmds > 5:
        print("Point Viewer given wrong arguments")
        print(expected)
        exit()
    else:    
        try:
        
            outputPath = cmds[1] #The first command must be the output path
            
            for cmdNum in range(2, lenCmds): #For all commands (except the program call itself and the path)
                field = cmd[cmdNum].split("=")[0] #Split the command into field and value
                value = cmd[cmdNum].split("=")[1]
                
                if(value == "d"): #Depth case
                    depthLim = float(value) #Set the depth limit to the given value
                elif(value == "lr"): #Left-Right Limit case
                    leftRightLim = [float(value.split(",")[0]), float(value.split(",")[1])] #Set the left-right limit to the given value
                elif(value == "ud"): #Left-Right Limit case
                    upDownLim = [float(value.split(",")[0]), float(value.split(",")[1])] #Set the left-right limit to the given value
                else:
                    print("Command could not be interpreted. What is: \"" + cmds[cmdNum] + "?\"")
                    print(expected)
                    exit()
        except:
            print("There was an error while reading the argumets. Please check the format and try again.")
            print(expected)
            exit()
            

#Define Get Frame
#Called every frame of the "animation"
#Does all the work of opening the frame file, selecting only good keypoints, then displaying
def getFrame(i):
    global frameNumber
    global pointcloudX
    global pointcloudY
    global pointcloudZ
    
    #Compose current file name/path
    digitNumberCurrent = len(str(frameNumber)) #Get number of digits of the frame number
    zerosCurrent = ""
    zerosCurrent = zerosCurrent.zfill(12-digitNumberCurrent)
    fileNameCurrent = outputPath + "\\" + zerosCurrent + str(frameNumber) + fileEnd #Put file name together
    #Next file name/path
    digitNumberNext = len(str(frameNumber+1)) #Get number of digits of the frame number
    zerosNext = ""
    zerosNext = zerosNext.zfill(12-digitNumberNext)
    fileNameNext = outputPath + "\\" + zerosNext + str(frameNumber+1) + fileEnd #Put file name together
    
    if (os.path.isfile(fileNameNext) == True): #If the next frame exists, load this frame, otherwise do nothing
        frameNumber += 1 #Increment the frame for next round
        
        #Empty the lists for each dimention
        pointcloudX = []
        pointcloudY = []
        pointcloudZ = []
    
        #Open files
        with open(fileNameCurrent, "r") as jFile:
            data = jFile.read()
        
            
        
        #Read the point cloud into an easy to manage 2D array
        poseFrame = json.loads(data) #load the file into a JSON object
        numPeople = len(poseFrame["people"])
        if(numPeople > 0): #If there is any data in this frame
            for person in range(0, numPeople): #For all people detected
                lenPose = len(poseFrame["people"][person]["pose_keypoints_3d"]) #Save the number of points in the pose/face
                lenFace = len(poseFrame["people"][person]["face_keypoints_3d"])
                
                #Add new sub-lists to denote a new person
                pointcloudX.append(list())
                pointcloudY.append(list())
                pointcloudZ.append(list())

                #Save pose points
                for i in range(0,lenPose,4):
                    poseDepth = poseFrame["people"][person]["pose_keypoints_3d"][i+2]
                    if(poseDepth > 0 and poseDepth < depthLim): #Only add the point if the depth is greater than 0 and less than depthLim
                        pointcloudX[person].append(poseFrame["people"][person]["pose_keypoints_3d"][i])
                        pointcloudY[person].append(poseFrame["people"][person]["pose_keypoints_3d"][i+1])
                        pointcloudZ[person].append(poseDepth)
                            
                        
                #Save face points
                for i in range(0,lenFace,4):
                    faceDepth = poseFrame["people"][person]["face_keypoints_3d"][i+2]
                    if(faceDepth > 0 and faceDepth < depthLim): #Only add the point if the depth is greater than 0 and less than depthLim
                        pointcloudX[person].append(poseFrame["people"][person]["face_keypoints_3d"][i])
                        pointcloudY[person].append(poseFrame["people"][person]["face_keypoints_3d"][i+1])
                        pointcloudZ[person].append(faceDepth)
            
            
            ax.clear() #Clear the plot
            ax.set_ylim([0,depthLim]) #Set the scale of the axes, see near top of file for limit declairations
            ax.set_xlim(leftRightLim)
            ax.set_zlim(upDownLim)
            ax.invert_zaxis()
            for person in range(0, numPeople):
                ax.scatter(pointcloudX[person], pointcloudZ[person], pointcloudY[person]) #Plot the points, note that y and z are swaped for easy viewing


if __name__ == '__main__':
    parseArgs(sys.argv)
    
    #For all points, plot them
    ani = FuncAnimation(fig, getFrame, interval=1000/13) #About twice the frame rate of OpenPose
    plt.show()
    