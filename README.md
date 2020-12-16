# Robot Pathfinding and NavigationVia Voice Inputs

## Prerequisites

### PyAudio:

* To install PyAudio for microphone input:
        
        sudo apt-get install python-pyaudio python3-pyaudio

### sound-play:

* To install sound-play for melodic ros use:
        
        sudo apt-get install ros-melodic-sound-play

* Compile laser_trace.cpp (provides laser ray tracing) as follows:
        
        cd <catkin_ws>/src/inst/src/laser_trace
        ./compile.sh

* Run order:

        roslaunch nav start.launch
        roslaunch inst launch.launch
        roslaunch vtt launch.launch
