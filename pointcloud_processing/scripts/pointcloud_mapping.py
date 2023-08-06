import numpy as np
import math
import ros_numpy as rnp
import rospy

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2

import tf.transformations as tft

class PointCloudMapping():
    """
    Class used for various tasks surrounding pointcloud mappng
    """

    def generate_torpedo_target(self, center_point, offset):
        """"
        Generate a torpedo target for the torpedo poster.
        Need position data from somewhere on the target board, but centered in the target hole.
        
        Args:
            center_point: centerpoint of the torpedo target [x,y]
            offset: offset in [x,y]

        Returns:
            point_list: Points on the same xy image plane, where depth data is the same as where the target hole is.
        """
        point_list = self.generate_points_circle(r= 5, n= 25, center_point=[center_point[0] + offset[0], center_point[1] + offset[1]])
        return point_list

    def generate_points_circle(self, r, n, center_point):
        """
        Generate list of n points around a center, with radius r
        
        Args:
            r: radius of circle, where you want to generate points
            n: number of points to generate
            center_point: center point for points to generate
        """
        point_list = np.empty((n,2),float)
        try:
            for i in range(n):
                ang = np.random.uniform(0,1) * 2 * math.pi
                hyp = math.sqrt(np.random.uniform(0, 1)) * r
                adj = math.cos(ang) * hyp
                opp = math.sin(ang) * hyp
                point_list[i] = [int(center_point[0] + adj), int(center_point[1] + opp)]
        except Exception as e:
            rospy.loginfo(e)
        return point_list

    def sift_feature_centeroid(self, point_list, pointcloud_data, use_standard = False):
        if not use_standard:
            cloud_points_as_matrix = rnp.pointcloud2_to_array(pointcloud_data)
            point_array = np.empty((0, 3), float) 
            for point in point_list:
                pc_data_points = np.array(cloud_points_as_matrix.item(int(point[1]), int(point[0])))
                if np.isfinite(np.sum(pc_data_points)):
                    point_array = np.append(point_array, [pc_data_points[:3]], axis=0)
                else:
                    rospy.logdebug("Point has nans, not adding: %s", str(pc_data_points))

            if np.shape(point_array)[0] != 0:
                return self.plane_with_SVD(point_array)
        else:
            new_point_list = []
            for point in point_list:
                pt_gen = point_cloud2.read_points(pointcloud_data, skip_nans=True, uvs=[[int(point[0]),int(point[1])]])
                for pt in pt_gen:
                    new_point_list.append([pt[0], pt[1], pt[2]])
            
            return self.plane_with_SVD(new_point_list)

        


    def sift_feature_gate(self, point_list, pointcloud_data):
        
        xmin = point_list[0]
        xmax = point_list[1]
        ymin = point_list[2]
        ymax = point_list[3]

        point_list = []
        for x in [xmin, xmax]:
            for y in [ymin, ymax]:
                pt_gen = point_cloud2.read_points(pointcloud_data, skip_nans=True, uvs=[[x,y]])
                for pt in pt_gen:
                    point_list.append([pt[0], pt[1], pt[2]])

        orientationdata, positiondata = self.points_to_plane(point_list)


        cloud_points_as_matrix = rnp.pointcloud2_to_array(pointcloud_data)
        point_array = np.empty((0, 3), float) 
        for point in point_list:
            pc_data_points = np.array(cloud_points_as_matrix.item(point.x, point.y))
            point_array = np.append(point_array, [pc_data_points[:3]], axis=0)

        if np.shape(point_array)[0] != 0:
            return self.plane_with_SVD(point_array)

        return orientationdata, positiondata


    def sift_feature_area(self, point_list, pointcloud_data):
        """
        """
        cloud_points_as_matrix = rnp.pointcloud2_to_array(pointcloud_data)
        rows = point_list[0][1] - point_list[0][0]
        cols = point_list[1][1] - point_list[1][0]
        test_array = np.zeros(shape=(rows, cols))

        

        svd_data = 0
        return svd_data

    def object_orientation_position(self, point_list, pointcloud_data):
        """
        Uses known points to find object and its orientation
        
        Args:
            point_list: list of points as tuples [(y,x),(y,x)]
            pointcloud_data: poiintcloud data extracted from camera
        
        Returns:
            orientationdata = [x, y, z, w]\n
            positiondata = [x, y, z]
        """
        cloud_points_as_matrix = rnp.pointcloud2_to_array(pointcloud_data)
        point_array = np.empty((0, 3), float) 
        for point in point_list:
            pc_data_points = np.array(cloud_points_as_matrix.item(point.x, point.y)) # TODO: x, y are sendt incorrectly from feature_detection
            point_array = np.append(point_array, [pc_data_points[:3]], axis=0)

        if np.shape(point_array)[0] != 0:
            return self.plane_with_SVD(point_array)

    def object_orientation_position_from_limits(self, square_area, pointcloud_data):
        cloud_points_as_matrix = rnp.pointcloud2_to_array(pointcloud_data)
        x_range = np.array(range(square_area[0], square_area[1]+1))
        y_range = np.array(range(square_area[2], square_area[3]+1))

        point_array = np.empty((0, 3), float)         
        for x in x_range:
            for y in y_range:
                pc_data_points = np.array(cloud_points_as_matrix.item(y, x)) # TODO: x, y are sendt incorrectly from feature_detection
                point_array = np.append(point_array, [pc_data_points[:3]], axis=0)
                
        return self.plane_with_SVD(point_array)

    def object_orientation_from_xy_area(self, area_with_limits, pointcloud_data): # Do not use
        """
        Reads the point cloud data from a given area

        Args:
            area_with_limits: list of data [xmin, xmax, ymin, ymax]
            pointcloud_data: poiintcloud data extracted from camera

        Returns:
            orientationdata = [x, y, z, w]
            positiondata = [x, y, z]
        """
        # Generates a readable version of the point cloud data
        assert isinstance(pointcloud_data, PointCloud2)

        xmin = area_with_limits[0]
        xmax = area_with_limits[1]
        ymin = area_with_limits[2]
        ymax = area_with_limits[3]

        # loops through the area data and adds points to a list
        point_list = []
        for x in range(xmin  -1, xmax -1):
            for y in range(ymin - 1, ymax - 1):
                pt_gen = point_cloud2.read_points(pointcloud_data, skip_nans=True, uvs=[[x,y]])
                for pt in pt_gen:
                    point_list.append([pt[0], pt[1], pt[2]])

        #orientationdata, positiondata = self.points_to_plane(point_list)

        orientationdata, positiondata = self.plane_with_SVD(point_list)
        return orientationdata, positiondata

    def object_position_from_xy_point(self, x_pixel, y_pixel, pointcloud_data): # Do not use
        """
        NOT FOR CPP
        Reads the point cloud data from a given x, y coordinate

        Args:
            x_pixel: position in x direction of point you want clouddata from
            y_pixel: position in y direction of point you want clouddata from
            pointcloud_data: poiintcloud data extracted from camera

        Returns:
            Point cloud data for a point in the camera frame as list [x, y, z]
        """
        # Generates a readable version of the point cloud data
        is_pointcloud = isinstance(pointcloud_data, PointCloud2)
        if is_pointcloud:
            # Reads the point cloud data at given uvs: u = x cord, v = y cord
            pt_gen = point_cloud2.read_points(pointcloud_data, skip_nans=False, uvs=[[x_pixel, y_pixel]])
            for pt in pt_gen:
                self.pointcloud_x = pt[0]
                self.pointcloud_y = pt[1]
                self.pointcloud_z = pt[2]

        x, y, z = self.pointcloud_x, self.pointcloud_y, self.pointcloud_z
        return [x, y, z]

    def object_orientation_from_point_list(self, point_list, pointcloud_data): # Do not use
        """
        Uses known points to find object and its orientation
        
        Args:
            point_list: list of points as tuples [(x,y),(x,y)]
            pointcloud_data: poiintcloud data extracted from camera
        
        Returns:
            orientationdata = [x, y, z, w] \n
            positiondata = [x, y, z]
        """
        assert isinstance(pointcloud_data, PointCloud2)
        new_point_list = []
        for point in point_list:
            pt_gen = point_cloud2.read_points(pointcloud_data, skip_nans=True, uvs=[[point[0],point[1]]])
            for pt in pt_gen:
                # new_point_list.append(pt)
                new_point_list.append([pt[0], pt[1], pt[2]])

        

        #orientationdata, positiondata = self.points_to_plane(new_point_list)

        orientationdata, positiondata = self.plane_with_SVD(new_point_list)
        return orientationdata, positiondata

    def points_to_plane(self, points_list):
        """
        NOT FOR CPP
        Function will give you an estimated plane from points in a 3D plane

        Args:
            points_list: list of points on the form [[x1,y1,z1],[x2,y2,z2]] or [(x1,y1,z1),(x2,y2,z2)], contents must be floats.

        Returns:
            vectordata: the equation as list of [a, b, c]
            middle_point: middle point cord as a list of points [x_center, y_center, z_center]
        """
        xs = []
        ys = []
        zs = []
        for point in points_list:
            xs.append(point[0])
            ys.append(point[1])
            zs.append(point[2])
            
        # IVAN: a smoother way of extracting the points?
        #xs = points_list[:,0]
        #ys = points_list[:,1]
        #zs = points_list[:,2]

        # do fit
        tmp_A = []
        tmp_b = []
        for i in range(len(xs)):
            tmp_A.append([xs[i], ys[i], 1])
            tmp_b.append(zs[i])
        b = np.matrix(tmp_b).T
        A = np.matrix(tmp_A)
        fit = (A.T * A).I * A.T * b
        errors = b - A * fit
        residual = np.linalg.norm(errors)
        planar_equation = ("%f x + %f y + %f = z" % (fit[0], fit[1], fit[2]))
        
        vectordata = []
        for i in range(3):
            numb = np.matrix.item(fit,i)
            if not math.isnan(numb):
                vectordata.append(numb)

        middle_point = self.get_middle_point(points_list)

        return vectordata, middle_point

    def plane_with_SVD(self, X):
        """
        Creates a plane position and orientation basis through Singluar Value Decomposition (SVD) of the pointcloud data

        Args:
            X: a M x 3 matrix of pointcloud data, M points with their x, y, z coordinate
                X = [x1, y1, z1]
                    [.   .    .]

                        ...

                    [xM, yM, zM]
        Returns:
            Rot: a 3x3 orthonormal basis (rotation matrix)
            pos: the average x, y, z position of the pointcloud data

        """
        pos = np.average(X,0)
        X_mean_centered = X - pos
        Rot_homo = np.eye(4)

        try:
            _, _, PT = np.linalg.svd(X_mean_centered)
        except:
            quit()

        Rot_homo[:3, :3] = PT.T
        quat = tft.quaternion_from_matrix(Rot_homo)

        return quat, pos

    def get_middle_point(self, points_list):
        """
        NOT FOR CPP
        Creates a middle point of n given points. n is the amount of points in points_list

        Args:
            points_list: list of points to get the middlepoint of. Must be on the form [[x1,y1,z1],[x2,y2,z2]] or [(x1,y1,z1),(x2,y2,z2)], contents must be floats.

        Returns:
            middle_point: coordinates as a list of points [x, y, z]
        """
        x_sum = 0.0
        y_sum = 0.0
        z_sum = 0.0
        for point in points_list:
            x_sum += point[0]
            y_sum += point[1]
            z_sum += point[2]
        
        n_points = len(points_list)
        x_middle_pos = x_sum/n_points
        y_middle_pos = y_sum/n_points
        z_middle_pos = z_sum/n_points

        middle_point = [x_middle_pos, y_middle_pos, z_middle_pos]
        return middle_point
