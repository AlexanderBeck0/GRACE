# bash install_submodules.sh
cd src
git clone --recursive https://github.com/leggedrobotics/darknet_ros.git -o src

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
    sed 's/# name: yolov3.cfg/name: yolov3-tiny.cfg/' yolov3-tiny.yaml
    # Replace 'name: yolov3.weights' with 'name: yolov3-tiny.weights'
    sed 's/# name: yolov3.weights/name: yolov3-tiny.weights/' yolov3-tiny.yaml
fi

# Go back to src
cd ../../../
git clone --recursive https://github.com/ipa320/ipa_coverage_planning.git -o src