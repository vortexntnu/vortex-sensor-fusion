#!/usr/bin/env python

import cv2 as cv
import glob
import numpy as np

#ROS imports
import rospy
import tf.transformations  # Required for the orientiation estimation implementation

from cv_bridge import CvBridge, CvBridgeError
from rospkg import RosPack

from cv_msgs.msg import Centeroid, CenteroidArray
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes
from sensor_msgs.msg import Image
from std_msgs.msg import String

# Contains visualization tools
class DrawTools:

    def find_centeroid(self, dst):
        '''
        Finds the centeroid for a trapezoid.
        Taking in four corner points and finding the middle points between these.
        The middle points then gets utillized to find the center of the figure using these.

        p1----p14----p4  
        |             |
        |             |
       p12     c     p34
        |             |
        |             |
        p2----p23-----p3

        Args:
            Four cornerpoints in a (4,1,2) array

        Returns:
            The centeroid points as a (1,2) array
        '''

        points = np.reshape(dst, (4, 2))
        x_c = np.average(points[:, 0])
        y_c = np.average(points[:, 1])
        center = np.array([x_c, y_c])

        return center

    def drawcircle(self,
                   img,
                   center_coordinates=(471, 134),
                   color=(255, 0, 0),
                   radius=5,
                   thickness=2):
        img = cv.circle(img,(471, 134) , radius, color, thickness)
        return img

    def draw_all_circles(self, img, dst, radius):

        points = np.reshape(dst, (4, 2))
        self.drawcircle(img,
                        center_coordinates=(points[0][0], points[0][1]),
                        color=(255, 0, 0),
                        radius=radius)
        self.drawcircle(img,
                        center_coordinates=(points[1][0], points[1][1]),
                        color=(0, 255, 0),
                        radius=radius)
        self.drawcircle(img,
                        center_coordinates=(points[2][0], points[2][1]),
                        color=(0, 0, 255),
                        radius=radius)
        self.drawcircle(img,
                        center_coordinates=(points[3][0], points[3][1]),
                        color=(255, 255, 255),
                        radius=radius)

        return img

    def text_on_image(self, img, dst, image_type):
        points = np.reshape(dst, (4, 2))
        x, y = int(points[0][0]), int(points[0][1] - 15.0)
        h, w = img.shape[0], img.shape[1]
        text = image_type

        cv.putText(img, text, (x, y), cv.FONT_HERSHEY_SIMPLEX, 0.7,
                   (255, 255, 255), 2, cv.LINE_AA)
        
        return img

    def draw_all(self, img, dst, dst_packed, image_type, centeroid):
        centeroid[0] = int(round(centeroid[0]))
        centeroid[1] = int(round(centeroid[1]))
        a = int(centeroid[0])
        b = centeroid[1]

        img = self.draw_all_circles(img, dst, radius=8)
        img = self.draw_all_circles(img, dst_packed, radius=5)
        img = self.text_on_image(img, dst, image_type)
        img = self.drawcircle(img,
                              center_coordinates=(a, b))
        img = cv.polylines(img, [np.int32(dst)], True, 255, 3, cv.LINE_AA)

        return img



class SiftFeature:
    '''
    TODO: The orientation of the object can be estimated based on the images from image list. For this to be done 
    properly the image used for orientation estimation should have known orientation. A implementation securing
    this is not implemented yet. (One idea is to have some images for position detection and one image for 
    orientation estimation)

    TODO: Add multi image type support

    TODO: Add a function that takes care of objects detected in the same spot (where one or more is false detections), 
    compare the number of matches and cut out the false positives.  

    This node:

    * Reads images from a folder(multiple folders), generates kp (keypoints) and des (descriptors) and stores them for the session

    * Compares stored kp and des with the ones from the camera image.

    * When matches are within the threshold, homography and perspective trasform (opencv functions) gets applied

    * The corner points from the read images gets found in the cam image, aswell as the centeroid


    * Bounding box and found points gets drawn and published if self.visualize_detections is set to True

    '''

    def __init__(self):

        ##################
        ##### Params #####
        ##################
        # grayscale = 0, color = 1
        self.colormode = 1

        # Only for visual effects (draws bounding box, cornerpoints, etc...)
        self.visualize_detections = True

        # Minimum matches needed for drawing bounding box
        self.MIN_MATCH_COUNT = 10

        # Scale the bounding box (Only downscale)
        self.scale = 0.999

        # Folders to read from
        self.image_types = ["gman", "bootlegger"]

        self.lower_image_list_index = 0
        self.upper_image_list_index = len(self.image_types)

        #################
        ###### Node #####
        #################

        #Name of the node
        node_name = "sift_feature"

        # ROS node init
        rospy.init_node(node_name)

        # The frame object gets detected in
        self.frame_id = "zed2_left_camera_sensor"

        #Subscribers
        rospy.Subscriber("/fsm/state", String, self.update_object_search)
        rospy.Subscriber("/zed2i/zed_node/rgb/image_rect_color", Image,
                         self.callback)

        # Publishers
        self.BBoxPointsPub = rospy.Publisher(
            '/feature_detection/sift_detection_bbox',
            BoundingBoxes,
            queue_size=1)
        self.detections_pub = rospy.Publisher(
            '/feature_detection/sift_bbox_image', Image, queue_size=1)
        self.detection_centeroid_pub = rospy.Publisher(
            '/feature_detection/sift_detection_centeroid',
            CenteroidArray,
            queue_size=1)

        ################
        ###CV stuff ####
        ################
        self.drawtools = DrawTools()

        self.bridge = CvBridge()

        self.sift = cv.xfeatures2d.SIFT_create() # (opencv version 4.x.y)
        #self.sift = cv.xfeatures2d.SIFT_create()  # (opencv version 3.4.x)

        ###############################
        ##### kp and des cam feed #####
        ###############################

        rp = RosPack()
        path = str(
            rp.get_path('sift_feature_detection')) + '/data/sift_images/'

        self.image_list = []
        self.kp = []
        self.des = []

        for i in range(len(self.image_types)):
            temp_path = glob.glob(path + self.image_types[i] + "/*.png")
            temp = [cv.imread(file, self.colormode) for file in temp_path]
            self.image_list.append(temp)
            temp_path = None

        # Gets keypoints and descriptors for every loaded image
        for image_type in self.image_list:
            kp_temp = []
            des_temp = []
            for image in image_type:
                k, d = self.sift.detectAndCompute(image, None)
                kp_temp.append(k)
                des_temp.append(d)

            self.kp.append(kp_temp)
            self.des.append(des_temp)

        ############
        ##Init end##
        ############

    def update_object_search(self, status):
        mission = status.data

        if mission == "gate/execute":
            # Remove first element from search
            self.lower_image_list_index += 1

    def add_centeroid(self, img_type, centeroid):
        pub = Centeroid()
        pub.name = img_type
        pub.centre_x = centeroid[0]
        pub.centre_y = centeroid[1]
        pub.centre_z = 0

        self.CentroidArray_message.centeroid.append(pub)

    def add_bounding_box(self, bbox_points, obj_class):
        bbox = BoundingBox()
        bbox.probability = 69.69
        bbox.xmin = int(bbox_points[0][0])
        bbox.ymin = int(bbox_points[0][1])
        bbox.xmax = int(bbox_points[2][0])
        bbox.ymax = int(bbox_points[2][1])
        bbox.z = 100000.0
        bbox.id = 0
        bbox.Class = obj_class

        self.BBoxPoints_message.bounding_boxes.append(bbox)

    def scale_bounding_box(self, dst, scale=1):
        '''
        This function downscales a bounding box to a smaller one (no upscale).
        The idea is to make sure the bounding box is to remove more points on the
        outside of the object.

        Keep the scale close to 1, (e.g. 0.999) if your bounding box gets uncanny when
        choosing a number further away from 1.

            p1-----------p4
            | \        /  |
            |   p1s  p4s  |
            |             |
            |   p2s  p3s  |
            |  /        \ |
            p2------------p3
        
        args:
            dst    : Array of (4,1,2), four cornerpoints
            scale    : Scale in range of [0, 1]

        Returns:
            dst_scaled_cv_packed    : Array of (4,1,2)
            dst_scaled    : Array of (4,2)
        '''
        sf = scale - 1
        sf1 = 1 + np.abs(sf)
        sf2 = 1 - np.abs(sf)

        points = np.reshape(dst, (4, 2))

        p1 = points[0]
        p2 = points[1]
        p3 = points[2]
        p4 = points[3]

        p1 = p1 * sf1
        p2[0], p2[1] = p2[0] * sf1, p2[1] * sf2
        p3 = p3 * sf2
        p4[0], p4[1] = p4[0] * sf2, p4[1] * sf1

        dst_scaled = np.array([p1, p2, p3, p4])

        dst_scaled_cv_packed = np.reshape(dst_scaled, (4, 1, 2))

        return dst_scaled_cv_packed, dst_scaled

    def compare_matches(self, cam_image, kp2, des2, single_image_list, kp, des,
                        image_type):

        FLANN_INDEX_KDTREE = 1
        index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
        search_params = dict(checks=50)
        flann = cv.FlannBasedMatcher(index_params, search_params)

        good_best = []

        for i in range(len(single_image_list)):
            matches = flann.knnMatch(des[i], des2, k=2)

            # store all the good matches as per Lowe's ratio test.
            good = []
            for m, n in matches:
                if m.distance < 0.7 * n.distance:
                    good.append(m)

            if len(good) > len(good_best):
                good_best = good
                kp_best = kp[i]
                best_image = single_image_list[i]

        if len(good_best) > self.MIN_MATCH_COUNT:
            src_pts = np.float32([kp_best[m.queryIdx].pt
                                  for m in good_best]).reshape(-1, 1, 2)
            dst_pts = np.float32([kp2[m.trainIdx].pt
                                  for m in good_best]).reshape(-1, 1, 2)
            M, _ = cv.findHomography(src_pts, dst_pts, cv.RANSAC, 5.0)

            if len(best_image.shape) == 2:
                h, w = best_image.shape
            else:
                h, w, _ = best_image.shape

            pts = np.float32([[0, 0], [0, h - 1], [w - 1, h - 1],
                              [w - 1, 0]]).reshape(-1, 1, 2)
            dst = cv.perspectiveTransform(pts, M)

            # Gets orientation of the object, this is based on the orientation of compare image
            #M0 = np.zeros((4,4))
            #M0[:3, :3] = M[:3, :3]
            #M0[3,3] = 1
            #orientation = tf.transformations.quaternion_from_matrix(M0)

            # Scales the bounding box
            dst_scaled_cv_packed, dst_scaled = self.scale_bounding_box(
                dst, self.scale)

            #self.publish_centeroid(i, centeroid, orientation)
            self.add_bounding_box(dst_scaled, image_type)

            # Add centeroid
            centeroid = self.drawtools.find_centeroid(dst)
            self.add_centeroid(image_type, centeroid)

            if self.visualize_detections == True:
                cam_image = self.drawtools.draw_all(cam_image, dst,
                                                    dst_scaled_cv_packed,
                                                    image_type, centeroid)

        return cam_image

    def cv_image_publisher(self, publisher, image, msg_encoding="bgra8"):
        pub_img = self.bridge.cv2_to_imgmsg(image, encoding=msg_encoding)
        publisher.publish(pub_img)

    def callback(self, cam_image_ros):



        try:
            cam_image = self.bridge.imgmsg_to_cv2(cam_image_ros, "passthrough")
            if self.colormode == 0:
                cam_image = cv.cvtColor(cam_image, cv.COLOR_BGR2GRAY)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        stamp = rospy.get_rostime()
        frame_id = self.frame_id
        # Creates a CenteroidArray
        self.CentroidArray_message = CenteroidArray()

        # Creates a Boundingbox array
        self.BBoxPoints_message = BoundingBoxes()

        kp2, des2 = self.sift.detectAndCompute(cam_image, None)

        # Loops through all of the image lists and the images within
        for i in range(self.lower_image_list_index,
                       self.upper_image_list_index):
            single_image_list = self.image_list[i]
            kp = self.kp[i]
            des = self.des[i]
            image_type = self.image_types[i]
            cam_image = self.compare_matches(cam_image, kp2, des2,
                                             single_image_list, kp, des,
                                             image_type)

        # Centeroid message
        self.CentroidArray_message.header.stamp = stamp
        self.CentroidArray_message.header.frame_id = frame_id
        self.detection_centeroid_pub.publish(self.CentroidArray_message)

        # Boundingbox message
        self.BBoxPoints_message.header.stamp = stamp
        self.BBoxPoints_message.header.frame_id = frame_id
        self.BBoxPointsPub.publish(self.BBoxPoints_message)

        # Visualization pub
        if self.visualize_detections == True:
            if self.colormode == 0:
                self.cv_image_publisher(self.detections_pub,
                                        cam_image,
                                        msg_encoding="mono8")
            else:
                self.cv_image_publisher(self.detections_pub,
                                        cam_image,
                                        msg_encoding="bgra8")


if __name__ == '__main__':
    feature = SiftFeature()
    while not rospy.is_shutdown():
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
