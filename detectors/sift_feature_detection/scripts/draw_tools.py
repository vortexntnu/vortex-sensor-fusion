import cv2 as cv
import numpy as np


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
                   center_coordinates=(10, 20),
                   color=(255, 0, 0),
                   radius=5,
                   thickness=2):
        img = cv.circle(img, center_coordinates, radius, color, thickness)
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
        img = self.draw_all_circles(img, dst, radius=8)
        img = self.draw_all_circles(img, dst_packed, radius=5)
        img = self.text_on_image(img, dst, image_type)
        img = self.drawcircle(img,
                              center_coordinates=(centeroid[0], centeroid[1]))
        img = cv.polylines(img, [np.int32(dst)], True, 255, 3, cv.LINE_AA)

        return img
