##########################################################################################################
instructions to run the project.
##########################################################################################################
	
	run the Gazebo with the robot at an unknown location in the map
	
	start the sonarsim.py to provide the sonar input. Or any other application that publishes to the
	sonar topic
	
	run the safegoto.py to run the program(it will wait for the input from the localization program to
	give the robots current location)
	
	run the localization.py, it should start to localize right away

##########################################################################################################
checklist before running the project:
##########################################################################################################
	
	open the mapGUI.py and change the PATH location to the complete location where the map is situated 
	with the map name and the extension
	
	open the safegoto.py and change the PATH location to the complete location where the map is situated 
	with the map name and the extension

