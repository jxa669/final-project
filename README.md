# Robot Pathfinding and NavigationVia Voice Inputs

## Prerequisites

### PyAudio:

* To install PyAudio for microphone input:
        
        sudo apt-get install python-pyaudio python3-pyaudio

### sound-play:

* To install sound-play for melodic ros use:
        
        sudo apt-get install ros-melodic-sound-play

* Run order:

        roslaunch nav start.launch
	roslaunch inst launch.launch
	roslaunch vtt launch.launch
