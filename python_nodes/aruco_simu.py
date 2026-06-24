#!/usr/bin/env python3
import rclpy, tf_transformations
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
from math import sin, cos, atan2, sqrt
import cv2
import numpy as np

class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        self.declare_parameter('compression', False)
        self.declare_parameter('marker_length', 0.1)

        self.marker_length = self.get_parameter('marker_length').get_parameter_value().double_value

        self.mtx = None
        self.dist = None
        self.c_pose = [0.0,0.0,0.0]

        self.Ryz = np.array([[0.0, 0.0, 1.0], [-0.9912, 0.1323, 0.0], [-0.1323, -0.9912, 0.0]])
        self.Tur = np.eye(4)
        self.Trc = np.eye(4)
        self.Tca = np.eye(4)
        self.Trc[:3, :3] = self.Ryz
        self.Trc[:3, 3] = [0.08, 0.0, 0.07]

        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)

        ml = self.marker_length

        self.obj_points = np.array([
            [-ml / 2,  ml / 2, 0],
            [ ml / 2,  ml / 2, 0],
            [ ml / 2, -ml / 2, 0],
            [-ml / 2, -ml / 2, 0]
        ], dtype=np.float32)

        self.bridge = CvBridge()
        self.sub_info = self.create_subscription(CameraInfo, 'video_source/camera_info', self.camera_info_cb, 10)
        #if self.get_parameter('compression').get_parameter_value().bool_value:
        self.subscription = self.create_subscription(CompressedImage, 'video_source/compressed', self.comp_img_clb, 10)
        #else:
            #self.subscription = self.create_subscription(Image, 'video_source/raw', self.img_clb, 10)

        self.create_subscription(Odometry, "odom", self.odom_clb, 10)
        self.pub = self.create_publisher(PoseArray, 'aruco_poses', 10)


    def odom_clb(self, data):
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        k = data.pose.pose.orientation
        q = [k.x, k.y, k.z, k.w]
        _, _, yaw = tf_transformations.euler_from_quaternion(q)
        self.c_pose = [x, y, yaw]

    def getRz(self, q):
        return np.array([[cos(q), -sin(q), 0.0], [sin(q), cos(q), 0.0], [0.0, 0.0, 1.0]])

    def camera_info_cb(self, msg):
        if self.mtx is not None:
            return
        self.mtx  = np.array(msg.k, dtype=np.float32).reshape((3, 3))
        self.dist = np.array(msg.d, dtype=np.float32)
        self.get_logger().info("Calibración recibida correctamente.")

    def img_clb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.get_aruco(frame)

    def comp_img_clb(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.get_aruco(frame)

    def get_aruco(self, frame):
        corners, ids, rejected = self.detector.detectMarkers(frame)
        msg = PoseArray()

        if ids is not None and len(self.c_pose) != 0:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            self.Tur[:3, :3] = self.getRz(self.c_pose[2])
            self.Tur[:3, 3] = [self.c_pose[0], self.c_pose[1], 0.0]
            z = []

            for i in range(len(ids)):
                success, rvec, tvec = cv2.solvePnP(self.obj_points, corners[i], self.mtx, self.dist, flags=cv2.SOLVEPNP_ITERATIVE)

                if success and ids[i][0] < 9:
                    rotation_mat, _ = cv2.Rodrigues(rvec)
                    self.Tca[:3, :3] = rotation_mat
                    self.Tca[:3, 3] = tvec.flatten()
                    z.append(self.Tur @ self.Trc @ self.Tca)
                    z[-1][3,3] = ids[i][0]
                    z[-1][0,3] -= 0.085*cos(self.c_pose[2])
                    z[-1][1,3] -= 0.085*sin(self.c_pose[2])
                    
                    self.get_logger().info(f'Marker {ids[i][0]} detected in {z[-1][:3, 3]}')

            if len(z) > 0:
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'world'

                for marker in z:
                    pose = Pose()
                    pose.position.x = marker[0,3]
                    pose.position.y = marker[1,3]
                    pose.position.z = 0.1 + marker[3,3]/10.0
                    q = tf_transformations.quaternion_from_matrix(marker)
                    pose.orientation.x = q[0]
                    pose.orientation.y = q[1]
                    pose.orientation.z = q[2]
                    pose.orientation.w = q[3]
                    msg.poses.append(pose)

        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()