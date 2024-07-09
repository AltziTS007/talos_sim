#!/usr/bin/python

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header
from cv_bridge import CvBridge
import ros_numpy


# define range of red and yellow color in HSV
lower_red = np.array([0, 100, 50])
upper_red = np.array([5, 255, 255])

lower_yellow = np.array([23, 150, 150])
upper_yellow = np.array([32, 255, 255])


class tape_localization(object):
    def __init__(self):
        # Params
        self.image = None
        self.br = CvBridge()
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(3)

        # Publishers
        self.pub_tcloud = rospy.Publisher('tape_cloud', PointCloud2, queue_size=10000)

        # Subscribers
        rospy.Subscriber("/camera/color/image_raw", Image, self.callback)

        rospy.Subscriber("/camera/depth/color/points", PointCloud2, self.callback_pcl)

    def callback(self, msg):
        """ Subscribes to camera's image, and applies the mask to the image
        Args:
            sensor_msgs/Image message
        Returns:
            None
        """

        rospy.loginfo('Image received...')
        rospy.loginfo("-----------------")

        self.image = self.br.imgmsg_to_cv2(msg)

        # Convert images to numpy arrays
        self.color_image = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)

        # convert BGR images to HSV
        hsv = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2HSV)

        # create masks for red and yellow
        mask_red = cv2.inRange(hsv, lower_red, upper_red)
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # finds the non zero indices
        yellow_tape = np.where(mask_yellow > 0)
        self.yellow_tape = np.array(yellow_tape)
        # rospy.loginfo(self.yellow_tape[0])

    def callback_pcl(self, msg):
        """ Subscribes to camera's pointcloud, and converts yellow_tape pixels to pcl points 
        Args:
            sensor_msgs/PointCloud2 message
        Returns:
            None
        """

        rospy.loginfo('Pointcloud2 received...')
        rospy.loginfo("-----------------------")

        pc = ros_numpy.numpify(msg) #https://github.com/eric-wieser/ros_numpy {sensor_msgs.msg.PointCloud2 ==> structured np.array}
        height = pc.shape[0]
        width = msg.width
        np_points = np.zeros((height * width, 3), dtype=np.float32)
        np_points[:, 0] = np.resize(pc['x'], height * width)
        np_points[:, 1] = np.resize(pc['y'], height * width)
        np_points[:, 2] = np.resize(pc['z'], height * width)

        self.tape_points = np_points[self.yellow_tape[0] * msg.width + self.yellow_tape[1]]         # from 1d index array to 2d index (y,x)
        # rospy.loginfo(self.tape_points)

    def point_cloud(self, points, parent_frame):
        """ Creates a point cloud message.
        Args:
            points: Nx3 array of xyz positions (m)
            parent_frame: frame in which the point cloud is defined
        Returns:
            sensor_msgs/PointCloud2 message
        """
        ros_dtype = PointField.FLOAT32
        dtype = np.float32
        itemsize = np.dtype(dtype).itemsize

        data = points.astype(dtype).tobytes()

        fields = [PointField(
            name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
            for i, n in enumerate('xyz')]

        header = Header(frame_id=parent_frame, stamp=rospy.Time.now())

        return PointCloud2(
            header=header,
            height=1,
            width=points.shape[0],
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=(itemsize * 3),
            row_step=(itemsize * 3 * points.shape[0]),
            data=data
        )

    def start(self):
        """ Ignites the process and starts publishing pcl
        Args:
            None
        Returns:
            None
        """

        while not rospy.is_shutdown():
            if self.image is not None:
                pcl = self.point_cloud(self.tape_points, 'camera_depth_optical_frame')
                rospy.loginfo('publishing pcl')
                # self.pub.publish(self.br.cv2_to_imgmsg(self.color_image))
                self.pub_tcloud.publish(pcl)
            self.loop_rate.sleep()


if __name__ == '__main__':
    rospy.init_node("tape_localization", anonymous=False)
    tlocal = tape_localization()
    tlocal.start()