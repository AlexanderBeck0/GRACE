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
[[ -d out ]] || mkdir out && cd out

[[ -d Detections ]] || mkdir Detections
[[ -d Images ]] || mkdir Images
[[ -d MissionStatus ]] || mkdir MissionStatus
[[ -d Reward ]] || mkdir Reward
[[ -d SLAM ]] || mkdir SLAM

# Go back to previous directory
cd ../

sudo apt install python-pip && pip install pybbn