#Launch file for RealSense to OpenPose 3D
#By Aidan Fox-Tierney, foxtier@atr.jp, June 23, 2022

#Made with the help of:
# "Delete all files in a directory in Python" by Techie Delight </>: https://www.techiedelight.com/delete-all-files-directory-python/
# "Python Script Infinite Loop until specific Keypress" answer by funie200: https://stackoverflow.com/questions/59804164/python-script-infinite-loop-untill-specific-keypress
# "Python 2.6 on Windows: how to terminate subprocess.Popen with "shell=True" argument?" answer by BartoszKP: https://stackoverflow.com/questions/696345/python-2-6-on-windows-how-to-terminate-subprocess-popen-with-shell-true-argum

import subprocess #For starting other programs
import psutil #For killing OpenPose (It is a tough program to kill)
import time #For sleep
import os #For output cleanup
import sys #Get command line arguments
import keyboard #For keypress to stop program

##Defaults
#Paths
openPosePath = "C:\\Program Files\\OpenPose\\openpose"
openPoseEXE = ".\\build\\x64\\Release\\OpenPoseDemo.exe"
openPoseOutputPath = "C:\\Users\\user\\Desktop\\AndroidControl\\sensor\\faceDetection\\openPoseOutput"
openPoseArgs = ["start", "/WAIT", openPoseEXE, "-camera", "1", "-camera_resolution", "1920x1080", "-write_json", openPoseOutputPath, "-output_resolution", "1280x720"] #Add: "-net_resolution", "320x176", "-face_net_resolution", "320x320" to the end of this list if more speed is needed for fewer people. If crashing, add "-fps_max", "12"
RealSense2OpenPoseEXE = "C:\\Users\\user\\Desktop\\AndroidControl\\sensor\\faceDetection\\RealSense2OpenPose3D\\Release\\RealSense2OpenPose3D.exe"
PointViewer = "C:\\Users\\user\\Desktop\\AndroidControl\\sensor\\faceDetection\\PointViewer.py"
#Output
clearOutput = 'Y'
numHeldFrames = 0 #How many frames are held onto during operation? -1 = all, 0 = 10 but will auto delete at the end, >= 1 = 6+ and will prompt deletion at end
#Viwer
startViewer = False #Will pointViewer.py be started or not
viewerExtra = "" #Extra cli arguments to modify the viewer's behavior
#Quit Program Key
quitKey = "q"
#Camera Resolution
colorResoultion = "1920x1080"

#Define Parse Command
#Parses the command line arguments to get the number of held frames and whether to start the viewer or not
#cmds: the list of commands (argv) from the command line
def parseCmd(cmds):
	global numHeldFrames
	global startViewer
	global viewerExtra
	global colorResoultion
	global openPoseArgs
	
	expected = "Expected: \"...\\launch.py \n\t[frames=<Number of Frames>]\n\t[view=<true/false>]\n\t[quit=<key name>]\n\t[d=<depth limit in meters>]\n\t[lr=<left limit in meters>,<right limit in meters>]\n\t[ud=<up above limit in meters>,<down below limit in meters>]\n\t[color-res=<width>x<height>]\n\t[face=<true/false>]\n\t[hand=<true/false>]\""
	
	lenCmds = len(cmds)
	
	if(lenCmds > 1): #There was an argument passed
		if(lenCmds > 10): #Too many arguments
			print("Too many arguments.")
			print(expected)
			exit()
		else:
			for cmdNum in range(1,lenCmds): #For all commands
				try:
					field = cmds[cmdNum].split("=")[0] #Grab the field name
					value = cmds[cmdNum].split("=")[1] #Grab the value
					
					if(field == "frames"): #Hold frames case
						numHeldFrames = int(value)
					elif(field == "view"): #Open viwer or not case
						if(value.lower() in ["true", "t", "yes", "y", "1"]): #If "True"
							startViewer = True
						else:
							startViewer = False
					elif(field == "quit"): #Quit key keybind case
						quitKey = value
					elif(field == "d" or field == "lr" or field == "ud"): #Point viwer extra argument case
						viewerExtra += " " + cmds[cmdNum] #Add this command to the viewerExtra string
					elif(field == "color-res"): #Change the color camera resolution
						colorResoultion = value
						openPoseArgs[6] = value
					elif(field == "face"): #Generate face keypoints
						if(value.lower() in ["true", "t", "yes", "y", "1"]): #If "True"
							openPoseArgs.append("--face")
					elif(field == "hand"): #Generate hand keypoints
						if(value.lower() in ["true", "t", "yes", "y", "1"]): #If "True"
							openPoseArgs.append("--hand")
					else:
						Print("\"" + field + "\" is not a valid argument name")
				except:
					print("There was an error while parsing the command.")
					print(expected)
					exit()
	
	print("Launching with settings: Held Frames = " + str(numHeldFrames) + ", Start Viewer = " + str(startViewer))
	
	
	
	
if __name__ == '__main__':
	parseCmd(sys.argv)
		
	#Start RealSense to OpenPose 3D
	print("Starting Realsense to OpenPose 3D in a new window...")
	realsense2OpenPoseProc = subprocess.Popen([RealSense2OpenPoseEXE, openPoseOutputPath, colorResoultion], creationflags=subprocess.CREATE_NEW_CONSOLE)
	print("Started Realsense to OpenPose 3D")

	#Wait a little bit for the color camera to be ready for use by OpenPose
	time.sleep(1)

	cameraReady = ['false']

	while(cameraReady[0] == "false"):
		#check if the color camera is ready
		readyInput = open(openPoseOutputPath + "/ready.txt")
		cameraReady = readyInput.readlines()
		readyInput.close()

		time.sleep(0.2) #If camera not yet ready, wait a little bit more

	#Once the camera is ready, start OpenPose
	print("Starting OpenPose in a new window...")
	openPoseProc = subprocess.Popen(openPoseArgs, creationflags=subprocess.CREATE_NEW_CONSOLE, shell=True, cwd=openPosePath)
	print("Started OpenPose")

	#Initialize viewer
	if(startViewer):
		print("Starting Point Viwer in a new window...")
		pointViewerProc = subprocess.Popen(["python", PointViewer, openPoseOutputPath + viewerExtra], creationflags=subprocess.CREATE_NEW_CONSOLE)
		print("Started Point Viwer")
	
	#Main loop for deleting old frames
	print("Press \"" + quitKey + "\" to quit the program")
	quit = False
	while quit != True:
		if keyboard.is_pressed(quitKey):
			quit = True
		if(numHeldFrames > -1): #If not all frames should be kept
			if(len(os.listdir(openPoseOutputPath)) > 11 + numHeldFrames): #If there are more files than 10 frames, one ready file, plus the number desired to be held
				os.remove(os.path.join(openPoseOutputPath, os.listdir(openPoseOutputPath)[0])) #Remove the first file in the directory
	
	#Kill the child processes :(
	print("Stopping the sub-programs...")
	realsense2OpenPoseProc.kill()
	psutil.Process(openPoseProc.pid).children()[1].kill() #Get the whole process tree and then kill it from the leaves to the root
	psutil.Process(openPoseProc.pid).children()[0].kill()
	openPoseProc.kill() #Finally kill the parent of the openPose tree
	if(startViewer): #Only attemt to kill the point viwer if it has been started
		pointViewerProc.kill()
	print("Programs stopped.")

	#Clean out the output folder
	if(numHeldFrames != 0): #If the keep output prompt is requested
		clearOutput = input("Clear the output? [Y/n]: ")
	if(clearOutput != 'n' and clearOutput != 'N'):
		print("Clearing output...")
		
		#Empty the output directory
		for file in os.listdir(openPoseOutputPath):
			os.remove(os.path.join(openPoseOutputPath, file))
		
		print("Done. Ready for next run.")
	
