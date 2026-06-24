#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, CameraInfo
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
import time


class CalibrationNode(Node):
  def __init__(self):
    super().__init__('calibration_node')
    self.declare_parameter('marker_length', 0.1)
    self.marker_length = self.get_parameter('marker_length').get_parameter_value().double_value
    self.mtx = None
    self.dist = None
    self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
    self.parameters = cv2.aruco.DetectorParameters()
    self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)
    ml = self.marker_length
    self.obj_points = np.array([[-ml/2, ml/2, 0], [ml/2, ml/2, 0], [ml/2, -ml/2, 0], [-ml/2, -ml/2, 0]], dtype=np.float32)
    self.state = "searching"
    self.aligned_since = None
    self.frame_center_x = None
    self.centroid_x = None
    self.kw = 0.003
    self.tol_px = 10
    self.bridge = CvBridge()
    self.msg_twist = Twist()
    self.create_subscription(CameraInfo, 'video_source/camera_info', self.camera_info_cb, 10)
    self.create_subscription(CompressedImage, "video_source/compressed", self.comp_img_clb, 10)
    self.pub = self.create_publisher(Twist, 'cmd_vel', 1)
    self.pub2 = self.create_publisher(Bool, "aligned_bot", 1)
    self.create_timer(0.1, self.state_machine)
    self.get_logger().info("Nodo de alineacion iniciado.")
    self.offset = -15
    self.flag = Bool()

  def camera_info_cb(self, msg):
    if self.mtx is not None:
      return
    self.mtx  = np.array(msg.k, dtype=np.float32).reshape((3, 3))
    self.dist = np.array(msg.d, dtype=np.float32)

  def comp_img_clb(self, msg):
    np_arr = np.frombuffer(msg.data, np.uint8)
    frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    self.process_frame(frame)

  def process_frame(self, frame):
    h, w = frame.shape[:2]
    self.frame_center_x = w // 2 + self.offset
    self.centroid_x = None
    corners, ids, _ = self.detector.detectMarkers(frame)
    cv2.line(frame, (self.frame_center_x, 0), (self.frame_center_x, h), (0, 255, 255), 2)
    if ids is not None:
      for i in range(len(ids)):
        marker_id = int(ids[i][0])
        #if marker_id < 0 or marker_id > 5:
        if marker_id != 0:
          continue
        c = corners[i][0]
        pts = c.astype(int)
        cv2.polylines(frame, [pts], isClosed=True, color=(0, 255, 0), thickness=2)
        cx = int(np.mean(c[:, 0]))
        cy = int(np.mean(c[:, 1]))
        cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
        cv2.putText(frame, f"{ids[i][0]}", (pts[0][0], pts[0][1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        if self.centroid_x is None:
          self.centroid_x = cx
    cv2.imshow("Calibration Node", frame)
    cv2.waitKey(1)

  def move_robot(self, v, w):
    self.msg_twist.linear.x  = v
    self.msg_twist.angular.z = w
    self.pub.publish(self.msg_twist)

  def stop_robot(self):
    self.move_robot(0.0, 0.0)

  def state_machine(self):
    match self.state:
      case "searching":
        if self.centroid_x is None:
          self.move_robot(0.0, 0.3)
        else:
          self.state = "aligning"
      case "aligning":
        if self.centroid_x is None:
          self.state = "searching"
          return
        error_px = self.centroid_x - self.frame_center_x
        if abs(error_px) <= self.tol_px:
          self.stop_robot()

          self.flag.data = True
          self.pub2.publish(self.flag)


          self.aligned_since = time.time()
          self.state = "waiting"
        else:
          w = -self.kw * error_px
          w = max(min(w, 0.3), -0.3)
          self.move_robot(0.0, w)
      case "waiting":
        if time.time() - self.aligned_since >= 5.0:
          self.stop_robot()
          raise SystemExit


def main(args=None):
  rclpy.init(args=args)
  node = CalibrationNode()
  try:
    rclpy.spin(node)
  except (KeyboardInterrupt, SystemExit):
    pass
  finally:
    node.get_logger().info("Nodo de alineacion terminado.")
    node.stop_robot()
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
  main()