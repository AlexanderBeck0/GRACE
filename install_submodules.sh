# bash install_submodules.sh
cd src
# git clone --recursive https://github.com/leggedrobotics/darknet_ros.git -o src
git submodule update --init --recursive
# Get weights if they do not exist
cd darknet_ros/darknet_ros/yolo_network_config/weights/
if [ ! -f yolov3-tiny.weights ]; then
    echo "Missing yolov3-tiny.weights. Downloading..."
    wget http://pjreddie.com/media/files/yolov3-tiny.weights
fi
if [ ! -f yolov3.weights ]; then
    echo "Missing yolov3.weights. Downloading..."
    wget http://pjreddie.com/media/files/yolov3.weights
fi

# Get config files
cd ../cfg/
if [ ! -f yolov3-tiny.cfg ]; then
    echo "Missing yolov3-tiny.cfg. Downloading..."
    wget https://raw.githubusercontent.com/pjreddie/darknet/refs/heads/master/cfg/yolov3-tiny.cfg
fi
cd ../../config/
if [ ! -f yolov3-tiny.yaml ]; then
    echo "Missing yolov3-tiny.yaml. Creating it..."
    # Copy yolov3.yaml
    cp yolov3.yaml yolov3-tiny.yaml
    # Replace 'name: yolov3.cfg' with 'name: yolov3-tiny.cfg'
    sed -i 's/name: yolov3.cfg/name: yolov3-tiny.cfg/' yolov3-tiny.yaml
    # Replace 'name: yolov3.weights' with 'name: yolov3-tiny.weights'
    sed -i 's/name: yolov3.weights/name: yolov3-tiny.weights/' yolov3-tiny.yaml
fi

# Go back to src
cd ../../../

# Remove unused folders in ipa_coverage_planning
cd ipa_coverage_planning/
[[ -d ipa_room_exploration ]] && rm -r ipa_room_exploration
[[ -d ipa_coverage_planning ]] && rm -r ipa_coverage_planning
[[ -d ipa_building_navigation ]] && rm -r ipa_building_navigation

# Go back to src
cd ../

# git clone --recursive https://github.com/ipa320/ipa_coverage_planning.git -o src

# Changes that Zhentian made (that I can't upload because of the submodules)
# Check if the Class_distribution is within BoundingBox.msg
if ! grep -Fxq "float32[] Class_distribution" darknet_ros/darknet_ros_msgs/msg/BoundingBox.msg
then
    # Not Found
    echo "Class_distribution in BoundingBox.msg not found. Adding it..."
    echo "float32[] Class_distribution" >> darknet_ros/darknet_ros_msgs/msg/BoundingBox.msg
fi

# Check if CheckForObjects.action uses CompressedImage
if ! grep -Fxq "sensor_msgs/CompressedImage image" darknet_ros/darknet_ros_msgs/action/CheckForObjects.action
then
    # Not Found
    echo "CheckForObjects.action uses Image instead of CompressedImage. Changing it..."
    # Replace 'sensor_msgs/Image image' with 'sensor_msgs/CompressedImage image'
    sed -i 's/Image image/CompressedImage image/' darknet_ros/darknet_ros_msgs/action/CheckForObjects.action
fi

# Change yolo_v3.launch
if ! grep -Fq "darknet_ros)/config/yolov3-tiny.yaml" darknet_ros/darknet_ros/launch/yolo_v3.launch
then
    echo "darknet_ros/.../yolo_v3.launch is using yolo_v3.yaml instead of yolov3-tiny.yaml. Changing..."
    # Replace yolov3.yaml with yolov3-tiny.yaml
    sed -i 's|darknet_ros)/config/yolov3.yaml|darknet_ros)/config/yolov3-tiny.yaml|' darknet_ros/darknet_ros/launch/yolo_v3.launch
fi

if grep -Fq "camera/rgb/image_raw" darknet_ros/darknet_ros/launch/yolo_v3.launch
then
    echo "darknet_ros/.../yolo_v3.launch is using camera/rgb/image_raw instead of /camera/rgb/void. Changing..."
    # Replace camera/rgb/image_raw with camera/rgb/void
    sed -i 's|camera/rgb/image_raw|camera/rgb/void|' darknet_ros/darknet_ros/launch/yolo_v3.launch
fi

changed_c_files=false

# Change detection_layer.c
if grep -Fq "dets[index].prob[j] = (prob > thresh) ? prob : 0;" darknet_ros/darknet/src/detection_layer.c
then
    echo "detection_layer.c still uses threshold instead of probability. Changing now..."
    # Replace 'dets[index].prob[j] = (prob > thresh) ? prob : 0;' with 'dets[index].prob[j] = prob;'
    sed -i 's/dets\[index\].prob\[j\] = (prob > thresh) ? prob : 0/dets\[index\].prob\[j\] = prob/' darknet_ros/darknet/src/detection_layer.c

    # Ensure to remind user to recompile
    changed_c_files=true
fi

# Change region_layer.c
if grep -Fq "dets[index].prob[j] = (prob > thresh) ? prob : 0;" darknet_ros/darknet/src/region_layer.c
then
    echo "region_layer.c still uses threshold instead of probability. Changing now..."
    # Replace 'dets[index].prob[j] = (prob > thresh) ? prob : 0;' with 'dets[index].prob[j] = prob;'
    sed -i 's/dets\[index\].prob\[j\] = (prob > thresh) ? prob : 0/dets\[index\].prob\[j\] = prob/g' darknet_ros/darknet/src/region_layer.c

    # Ensure to remind user to recompile
    changed_c_files=true
fi

# Change ObjectDetection.cpp
if ! grep -Fxq "#include <sensor_msgs/CompressedImage.h>" darknet_ros/darknet_ros/test/ObjectDetection.cpp
then
    echo "Missing import in ObjectDetection.cpp. Adding it..."
    # Add "#include <sensor_msgs/CompressedImage.h>" under "#include <sensor_msgs/Image.h>"
    sed -i "/#include <sensor_msgs\/Image.h>/a\#include <sensor_msgs\/CompressedImage.h>" darknet_ros/darknet_ros/test/ObjectDetection.cpp

    # Ensure to remind user to recompile
    changed_c_files=true
fi

# Change YoloObjectDetector.hpp
if ! grep -Fq "#include <sensor_msgs/CompressedImage.h>" darknet_ros/darknet_ros/include/darknet_ros/YoloObjectDetector.hpp
then
    echo "Not including sensor_msgs/CompressedImage.h in YoloObjectDetector.hpp. Adding..."
    # Add include <sensor_msgs/CompressedImage.h>
    sed -i "/#include <sensor_msgs\/Image.h>/a\#include <sensor_msgs\/CompressedImage.h>" darknet_ros/darknet_ros/include/darknet_ros/YoloObjectDetector.hpp

    # Ensure to remind user to recompile
    changed_c_files=true
fi

if ! grep -Fq "std::vector<float> ClassDistribution;" darknet_ros/darknet_ros/include/darknet_ros/YoloObjectDetector.hpp
then
    echo "Missing ClassDistribution in RosBox_ in YoloObjectDetector.hpp. Adding now..."
    # Add (two spaces) std::vector<float> ClassDistribution; after int num, Class;
    sed -i "/int num, Class;/a\  std::vector<float> ClassDistribution;" darknet_ros/darknet_ros/include/darknet_ros/YoloObjectDetector.hpp

    # Ensure to remind user to recompile
    changed_c_files=true
fi

# Remind user to recompile if a c file was changed
if $changed_c_files; then
    echo "Modified c files. Remember to recompile"
fi