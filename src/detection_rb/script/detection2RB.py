#!/usr/bin/env python2
import os
from time import time
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
import message_filters
# Brings in the SimpleActionClient
import actionlib
import darknet_ros_msgs.msg
import numpy as np
import copy
import cv2
# create the nodes
from object_ros_msgs.msg import RangeBearing, RangeBearings, Object2D, Object2DArray

class Detection2RB:
    def __init__(self):
        self.bridge = CvBridge()
        self.img = None
        self.depth = None

        self.latch = False
        self.isUpdated = False
        
        self.image_sub = message_filters.Subscriber('/camera/rgb/image_rect_color/compressed', CompressedImage)
        self.depth_sub = message_filters.Subscriber('/camera/depth_registered/image_raw/compressed', CompressedImage)
        
        self.range_pub = rospy.Publisher("/range_bearing", RangeBearings, queue_size=10)

        self.client = actionlib.SimpleActionClient('/darknet_ros/check_for_objects', darknet_ros_msgs.msg.CheckForObjectsAction)

        # Waits until the action server has started up and started
        # listening for goals.
        self.client.wait_for_server()

        info_msg = rospy.wait_for_message('/camera/rgb/camera_info', CameraInfo)

        self.fx = info_msg.K[0]
        self.cx = info_msg.K[2]
        self.fy = info_msg.K[4]
        self.cy = info_msg.K[5]
        
        self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.depth_sub], 10, 0.02)
        self.ts.registerCallback(self.callback)

    def depth_callback(self, depth_msg):
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        print(np.amax(depth))
        print(np.amin(depth))
        
    def run(self):
        while (True):
            sendGoal = False
            depth = None
            img_id = None
            if self.isUpdated:
                self.latch = True
                self.isUpdated = False
                # Creates a goal to send to the action server.
                goal = darknet_ros_msgs.msg.CheckForObjectsGoal()
                goal.id = 0
                # Changed to convert to compressed image since Image does not have format attribute
                # Note that it is likely that the republish in the turtlebot.launch was used for this purpose,
                # but simply uncommenting it does not fix it

                # goal.image = self.img
                goal.image = self.convert_Image_to_CompressedImage(self.img)
                img_id = self.img.header.seq
                self.client.send_goal(goal)
                depth = self.depth.copy()
                sendGoal = True
                self.latch = False

            if sendGoal:
                self.client.wait_for_result()
                result = self.client.get_result()

                range_msg = RangeBearings()
                range_bearings = []
                file = open(os.path.join(os.path.dirname(__file__), "../../../out/Detections/Detections.txt"), "a")
                line = str(img_id) + " " 
                for bounding_box in result.bounding_boxes.bounding_boxes:
                    depth_mask = depth[bounding_box.ymin:bounding_box.ymax, bounding_box.xmin:bounding_box.xmax]
                    obj_coords = np.nonzero(depth_mask)
                    z = depth_mask[obj_coords] / 1000.0

                    obj_coords = np.asarray(obj_coords).T
                    obj_coords = obj_coords + np.asarray([bounding_box.ymin, bounding_box.xmin])
                    ux = obj_coords[:, 1]
                    uy = obj_coords[:, 0]

                    x = (ux - self.cx) * z / self.fx
                    y = (uy - self.cy) * z / self.fx

                    x_mean = np.mean(x)
                    y_mean = np.mean(y)
                    z_mean = np.mean(z)

                    Oc = [x_mean, y_mean, z_mean]

                    obj_range = np.sqrt(Oc[0] * Oc[0] + Oc[2] * Oc[2])
                    bearing = np.arctan2(-Oc[0], Oc[2])

                    if (np.isfinite(obj_range) and np.isfinite(bearing)):
                        range_bearing = RangeBearing()
                        range_bearing.range = obj_range  
                        range_bearing.bearing = bearing 
                        range_bearing.id = bounding_box.id
                        range_bearing.obj_class = bounding_box.Class
                        range_bearing.probability = bounding_box.Class_distribution
                        range_bearings.append(range_bearing)

                    if bounding_box.Class == "person":
                        line += (str(bounding_box.xmin) + " " + str(bounding_box.xmax) + " " + \
                                str(bounding_box.ymin) + " " + str(bounding_box.ymax))

                file.write(line  + '\n')
                file.close()

                if range_bearings:
                    range_msg.header = copy.deepcopy(self.img.header)
                    range_msg.range_bearings = range_bearings # camera_rgb_optical_frame"
                    self.range_pub.publish(range_msg)

            
    def callback(self, compressed_img_msg, compressed_depth_msg):
        """Callback from img
        :type msg: Image
        """
        if not self.latch:
            self.latch = True

            self.img = self.convert_ros_compressed_msg_to_ros_msg(compressed_img_msg)
            self.depth = self.convert_ros_compressedDepth_to_cv2(compressed_depth_msg)
            
            # print(np.amax(self.depth))
            # print(np.amin(self.depth))

            # print(np.unique(self.depth))

            cv2.imwrite('depth_reconstrcted.jpg', self.depth)
            
            self.latch = False
            self.isUpdated = True

    def convert_ros_compressed_to_cv2(self, compressed_msg):
        np_arr = np.fromstring(compressed_msg.data, np.uint8)
        return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    
    def convert_ros_compressedDepth_to_cv2(self, compressed_msg):
        np_arr = np.frombuffer(compressed_msg.data, np.uint8)
        # print(np_arr.shape)
        # print(np.amax(np_arr))
        return cv2.imdecode(np_arr, -1)

    def convert_ros_compressed_msg_to_ros_msg(self, compressed_msg,
                                              encoding='rgb8'):
        cv2_img = self.convert_ros_compressed_to_cv2(compressed_msg)
        ros_img = self.bridge.cv2_to_imgmsg(cv2_img, encoding=encoding)
        ros_img.header = compressed_msg.header
        return ros_img
    
    def convert_compressedDepth_to_cv2(self, compressed_depth):
        """
        Convert a compressedDepth topic image into a cv2 image.
        compressed_depth must be from a topic /bla/compressedDepth
        as it's encoded in PNG
        Code from: https://answers.ros.org/question/249775/display-compresseddepth-image-python-cv2/
        """
        depth_fmt, compr_type = compressed_depth.format.split(';')
        # remove white space
        depth_fmt = depth_fmt.strip()
        compr_type = compr_type.strip()

        print(compr_type)

        # if compr_type != "compressedDepth":
        #     raise Exception("Compression type is not 'compressedDepth'."
        #                     "You probably subscribed to the wrong topic.")

        # remove header from raw data, if necessary
        if 'PNG' in compressed_depth.data[:12]:
            # If we compressed it with opencv, there is nothing to strip
            depth_header_size = 0
        else:
            # If it comes from a robot/sensor, it has 12 useless bytes apparently
            depth_header_size = 12
        
        depth_header_size = 0
        raw_data = compressed_depth.data[depth_header_size:]

        depth_img_raw = cv2.imdecode(np.frombuffer(raw_data, np.uint8),
                                     # the cv2.CV_LOAD_IMAGE_UNCHANGED has been removed
                                     -1)  # cv2.CV_LOAD_IMAGE_UNCHANGED)
        if depth_img_raw is None:
            # probably wrong header size
            raise Exception("Could not decode compressed depth image."
                            "You may need to change 'depth_header_size'!")
        return depth_img_raw
    
    def convert_Image_to_CompressedImage(self, image):
        """
        Converts `image` into a `CompressedImage`

        Parameters:
            image (Image): The image to convert into a `CompressedImage`.

        Returns:
            CompressedImage (CompressedImage): The compressed image
        """

        np_image = np.frombuffer(image.data, np.uint8).reshape(image.height, image.width, -1)

        success, encoded_image = cv2.imencode('.jpg', np_image)

        compressed_image = CompressedImage()
        compressed_image.header = image.header
        compressed_image.format = "jpeg"  # Update based on your compression method
        compressed_image.data = encoded_image.tobytes()

        return compressed_image
       
if __name__ == '__main__':
    rospy.init_node("detection_2_rb")
    rospy.loginfo("Press Ctrl + C to terminate")
    whatever = Detection2RB()
    whatever.run()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
