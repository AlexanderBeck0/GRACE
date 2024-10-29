# Save working directory
current_dir=$(pwd)
# Go to catkin_ws
cd ../
# https://github.com/ipa320/ipa_coverage_planning/issues/8#issuecomment-1971892059
sudo apt-get install libopencv-dev libboost-all-dev coinor-libcoinutils-dev coinor-libosi-dev coinor-libclp-dev coinor-libcgl-dev coinor-libcbc-dev
sudo apt install ros-melodic-cob-map-accessibility-analysis
sudo apt install ros-melodic-opengm
sudo apt install ros-melodic-libdlib

# Go back to this directory
cd $current_dir

# Make folders if they do not already exist

# Format for creating a folder if it doesn't exist is:
# [[ -d NewFolder ]] || mkdir NewFolder

[[ -d out ]] || mkdir out && cd out

[[ -d Detections ]] || mkdir Detections
[[ -d RoomLabels ]] || mkdir RoomLabels
[[ -d Images ]] || mkdir Images
[[ -d MissionStatus ]] || mkdir MissionStatus
[[ -d Reward ]] || mkdir Reward
[[ -d SLAM ]] || mkdir SLAM

# Go back to previous directory
cd ../

sudo apt install python3-pip && pip3 install --upgrade pip
pip3 install pybbn && pip3 install rospkg && pip3 install matplotlib && pip2 install scikit-image

# Use this version since all others that I have tried in the past 2 hours are just not working
# Stuck at Building wheels for collected packages: opencv-python
pip3 install opencv-python==4.5.3.56