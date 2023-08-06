#!/usr/bin/env python3

# import debugpy
# print("Waiting for VSCode debugger...")
# debugpy.listen(5678)
# debugpy.wait_for_client()

from matplotlib.transforms import Bbox
import numpy as np

import rospy
import tf2_geometry_msgs.tf2_geometry_msgs
import tf2_ros

# Import msg types
from cv_msgs.msg import PointArray, Centeroid, CenteroidArray
from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes
from vortex_msgs.msg import ObjectPosition
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry

# Import classes
from pointcloud_mapping import PointCloudMapping

class PointcloudProcessingNode():
    """
    Handles tasks related to pointcloud processing
    """
    cameraframe_x = 1280    # Param to set the expected width of cameraframe in pixels
    cameraframe_y = 720     # Param to set the expected height of cameraframe in pixels
    use_reduced_pc = False  # Param to change wether or not to use reduced pointcloud data
    pose_name = "/pointcloud_processing/poseStamped"

    def __init__(self):
        
        rospy.init_node('pointcloud_processing_node')
        # Decide which pointcloud to use, this onsly works on topics described below
        if self.use_reduced_pc:
            self.pointcloud_reducedSub = rospy.Subscriber('/pointcloud_downsize/output', PointCloud2, self.pointcloud_camera_cb)
        else:
            self.pointcloudSub = rospy.Subscriber('/zed2i/zed_node/point_cloud/cloud_registered', PointCloud2, self.pointcloud_camera_cb, queue_size=1)

        #self.siftSub = rospy.Subscriber('/feature_detection/detection_bbox', BoundingBoxes, self.sift_cb, queue_size=1)
        self.siftCentSub = rospy.Subscriber('/feature_detection/sift_detection_centeroid', CenteroidArray, self.sift_centeroid_cb, queue_size=1)

        # Defining classes
        self.pointcloud_data = PointCloud2()
        self.pointcloud_mapper = PointCloudMapping()
        self.pose_transformer = tf2_geometry_msgs.tf2_geometry_msgs
        self._tfBuffer = tf2_ros.Buffer()
        self.__listener = tf2_ros.TransformListener(self._tfBuffer)

        self.object_name = "INIT"
        self.prev_object_name = "INIT"

    def sift_centeroid_cb(self, msg):
        """
        CB to sift feature cenetoid. Gets the centerpoint of a detection, 
        creates circular points around center and finds position and orientation of feature.
        """
        
        for ctrd in msg.centeroid:
            feat_name = ctrd.name
            feat_x = ctrd.centre_x
            feat_y = ctrd.centre_y
            feat_z = ctrd.centre_z

            r = 10
            n = 200

            if feat_name == "gate_actual":
                point_list = self.pointcloud_mapper.generate_points_circle(r,n,[feat_x,feat_y])
                rot, pos = self.pointcloud_mapper.sift_feature_centeroid(point_list, self.pointcloud_data)
            elif feat_name in ["torpedo_target"]:
                point_list = self.pointcloud_mapper.generate_torpedo_target([feat_x, feat_y], [-7, -14])
                rot, pos = self.pointcloud_mapper.sift_feature_centeroid(point_list, self.pointcloud_data)
                
            else:
                if feat_name in ["gate", "octagon"]:
                    r = 3
                    n = 25
                if feat_name in ["buoy", "torpedo_poster"]:
                    r = 5
                    n = 50
                point_list = self.pointcloud_mapper.generate_points_circle(r, n, [feat_x,feat_y])
                #print(point_list)
                #print(self.pointcloud_data)
                rot, pos = self.pointcloud_mapper.sift_feature_centeroid(point_list, self.pointcloud_data)

            try:
                self.send_poseStamped_world(pos, rot, "feat_name")
            except Exception as e:
                rospy.logdebug("Sift pose message error: ", e)

    def sift_cb(self, msg):
        """
        Callback to sift feature detection.

        Args:
            msg: Message containing boundingboxes in current image frame.
        """

        for box in msg.bounding_boxes:
            # This loop will iterate for all the bboxes
            
            bbox_data = np.array([[box.xmin, box.xmax], [box.ymin, box.ymax]])
            box_point_list = [(box.xmin,box.ymin),
                                (box.xmin,box.ymax),
                                (box.xmax,box.ymin),
                                (box.xmax,box.ymax)]

            if box.Class == "gate":
                or_data, pos_data = self.pointcloud_mapper.sift_feature_gate([box.xmin, box.xmax, box.ymin, box.ymax/2], self.pointcloud_data)
                rospy.info(or_data)
                self.send_poseStamped_world(pos_data, or_data, box.Class)

            if str(box.Class) in ["gman", "bootlegger"]:
                if ((box.xmax > box.xmin) and (box.ymax > box.ymin)):
                    svd_data = self.pointcloud_mapper.sift_feature_area(bbox_data, self.pointcloud_data)

            # Testing
            if box.Class in ["gate"]:
                rot, pos = self.pointcloud_mapper.object_orientation_from_point_list(box_point_list, self.pointcloud_data)
                self.send_poseStamped_world(pos, rot, "gate")

    def feat_det_cb(self, msg):
        """
        Callback if a message from feature detection node is recieved. Will read through a PointArray message and add each point to a new list.
        It will use this pointlist to get orientation- and positiondata for the detected object, and send this to a remote filter.

        Args:
            msg: The message recieved from feature_detection_node. It should be a PointArray message.
        """
        try:
            rot, pos = self.pointcloud_mapper.object_orientation_position(msg.point_array, self.pointcloud_data)
        except TypeError:
            quit()
        self.send_poseStamped_world(pos, rot)

    def bbox_cb(self, msg):
        """
        Callback if a message from darknet ros node is recieved. Will read through a PointArray message and add each point to a new list.
        It will use this pointlist to get orientation- and positiondata for the detected object, and send this to a remote filter.
        Args:
            msg: The message recieved from feature_detection_node. It should be a PointArray message.
        """
        headerdata = msg.header
        self.object_name = msg.bounding_boxes[0].Class

        rospy.loginfo(self.object_name)

        bbox = [msg.bounding_boxes[0].xmin,
                msg.bounding_boxes[0].xmax,
                msg.bounding_boxes[0].ymin,
                msg.bounding_boxes[0].ymax]

        # Calls function to find object centre and orientation
        #orientationdata, positiondata = self.pointcloud_mapper.object_orientation_from_xy_area(bbox, self.pointcloud_data)
        orientationdata, positiondata = self.pointcloud_mapper.object_orientation_from_xy_area(bbox, self.pointcloud_data)
        self.send_poseStamped_world(positiondata, orientationdata)
        self.send_ObjectPose_message(headerdata, positiondata, orientationdata)

        self.prev_object_name = self.object_name

    def pointcloud_camera_cb(self, msg_data):
        """
        Callback for pointcloud message. Checks to see if message is correct type
        and stores it as a class variable which is updated everytime a new message is recieved.

        Args:
            msg_data: pointcloud-data message
        """
        assert isinstance(msg_data, PointCloud2)
        self.pointcloud_data = msg_data

    def send_poseStamped_world(self, position_data, quaternion_data, name):
        """
        Transform a pose from zed2_left_camera_frame to a pose in odom, before publishing it as a pose.

        Args:
            position_data: position data describing the position of the pose
            quaternion_data: the quaternion data describing the orientation of the pose
            name: identifyer for the detected object
        """
        #parent_frame = "odom"
        #child_frame = "zed2_left_camera_frame"
        #
        #tf_lookup_world_to_camera = self._tfBuffer.lookup_transform(parent_frame, child_frame, rospy.Time.now(), rospy.Duration(5))

        # if self.prev_object_name != self.object_name:
        # posePub = rospy.Publisher("/pointcloud_processing/object_pose/" + self.object_name, PoseStamped, queue_size=1)

        posePub = rospy.Publisher("/pointcloud_processing/poseStamped/" + name, PoseStamped, queue_size=1)

        # Pose generation
        pose_msg_camera = PoseStamped()
        # Format header
        pose_msg_camera.header.frame_id = "zed2_left_camera_frame"
        pose_msg_camera.header.stamp = rospy.get_rostime()

        # Build pose

        # Trying with w being last in the quaternion extraction
        pose_msg_camera.pose.position.x = position_data[0]
        pose_msg_camera.pose.position.y = position_data[1]
        pose_msg_camera.pose.position.z = position_data[2]
        pose_msg_camera.pose.orientation.x = quaternion_data[0]
        pose_msg_camera.pose.orientation.y = quaternion_data[1]
        pose_msg_camera.pose.orientation.z = quaternion_data[2]
        pose_msg_camera.pose.orientation.w = quaternion_data[3]

        # WPF TODO: contact BenG or IvanG
        if max(position_data) > 7 or any(np.isnan(position_data)) or any(np.isnan(quaternion_data)):
            quit()


        #pose_msg_odom = self.pose_transformer.do_transform_pose(pose_msg_camera, tf_lookup_world_to_camera)
        posePub.publish(pose_msg_camera)       

    def send_pointStamped_message(self, position, name):
        """
        Publishes a PointStamped as a topic under /pointcloud_processing/object_point

        Args:
            headerdata: Headerdata to be used as a header will not be created in this function
            position: A position xyz in the form [x, y, z] where xyz are floats
            name: name to be given to the point published, must not contain special characters.

        Returns:
            Topic:
                /pointcloud_processing/object_point/name where name is your input
        """
        # For testing
        pointPub = rospy.Publisher('/pointcloud_processing/object_point/' + name, PointStamped, queue_size= 1)
        new_point = PointStamped()
        new_point.header.frame_id = "zed2_left_camera_frame"
        new_point.header.stamp = rospy.get_rostime()
        new_point.point.x = position[0]
        new_point.point.y = position[1]
        new_point.point.z = position[2]
        pointPub.publish(new_point)

    def send_pose_message(self, headerdata, position_data, quaternion_data):
        """
        Publishes a PoseStamped as a topic under /pointcloud_processing/object_pose

        Args:
            headerdata: Headerdata to be used as a header will not be created in this function
            position_data: A position xyz in the form [x, y, z] where xyz are floats
            quaternion_data: A quaternion wxyz in the form [w, x, y, z]
            name: name to be given to the point published, must not contain special characters.

        Returns:
            Topic:
                /pointcloud_processing/object_pose/name where name is your input
        """
        #if self.prev_object_name != self.object_name:
        #    posePub = rospy.Publisher('/pointcloud_processing/object_pose_rviz/' + self.object_name, PoseStamped, queue_size= 1)
        p_msg = PoseStamped()
        # Format header
        p_msg.header = headerdata
        p_msg.header.stamp = rospy.get_rostime()

        # Build pose
        p_msg.pose.position.x = position_data[0]
        p_msg.pose.position.y = position_data[1]
        p_msg.pose.position.z = position_data[2]
        p_msg.pose.orientation.x = 1
        p_msg.pose.orientation.y = quaternion_data[2]
        p_msg.pose.orientation.z = 1
        p_msg.pose.orientation.w = 1
        posePub.publish(p_msg)

    def send_ObjectPose_message(self, headerdata, position_data, quaternion_data):
        """
        Publishes a PoseStamped as a topic under /pointcloud_processing/object_pose
        Args:
            headerdata: Headerdata to be used as a header will not be created in this function
            position_data: A position xyz in the form [x, y, z] where xyz are floats
            quaternion_data: A quaternion wxyz in the form [w, x, y, z]
            name: string name to be given to the point published, must not contain special characters.
        Returns:
            Topic:
                /pointcloud_processing/object_pose/name where name is your input
        """

        #parent_frame = "odom"
        #child_frame = "zed2_left_camera_frame"
        #
        #tf_lookup_world_to_camera = self._tfBuffer.lookup_transform(parent_frame, child_frame, rospy.Time.now(), rospy.Duration(5))
        # if self.prev_object_name != self.object_name:
        # objposePub = rospy.Publisher('/pointcloud_processing/object_pose/' + self.object_name, ObjectPosition, queue_size= 1)
        p_msg = ObjectPosition()
        p_msg.objectID = self.object_name

        # Build pose
        p_msg.objectPose.header = headerdata

        #p_msg.objectPose.pose.position.x = position_data[0]
        #p_msg.objectPose.pose.position.y = position_data[1]
        #p_msg.objectPose.pose.position.z = position_data[2]
        #p_msg.objectPose.pose.orientation.x = 1
        #p_msg.objectPose.pose.orientation.y = quaternion_data[2]
        #p_msg.objectPose.pose.orientation.z = 1
        #p_msg.objectPose.pose.orientation.w = 1
        #self.objposePub.publish(p_msg)
        # # quaternion_data = np.array([1, 1, quaternion_data[2], 1])
        # # quaternion_data = quaternion_data / np.linalg.norm(quaternion_data)
        p_msg.objectPose.pose.position.x = position_data[0]
        p_msg.objectPose.pose.position.y = position_data[1]
        p_msg.objectPose.pose.position.z = position_data[2]
        p_msg.objectPose.pose.orientation.x = quaternion_data[0]
        p_msg.objectPose.pose.orientation.y = quaternion_data[1]
        p_msg.objectPose.pose.orientation.z = quaternion_data[2]
        p_msg.objectPose.pose.orientation.w = quaternion_data[3]

        #p_msg.objectPose = self.pose_transformer.do_transform_pose(p_msg.objectPose, tf_lookup_world_to_camera)
        self.objposePub.publish(p_msg)

        #self.objposePub.publish(p_msg)

if __name__ == '__main__':
    node = PointcloudProcessingNode()

    while not rospy.is_shutdown():
        rospy.spin()

