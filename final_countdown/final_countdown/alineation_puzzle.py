
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
    self.declare_parameter('marker_length', 0.028)
    self.marker_length = self.get_parameter('marker_length').get_parameter_value().double_value
    self.mtx = None
    self.dist = None
    self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
    self.parameters = cv2.aruco.DetectorParameters()
    self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)
    ml = self.marker_length
    self.obj_points = np.array([[-ml/2, ml/2, 0], [ml/2, ml/2, 0], [ml/2, -ml/2, 0], [-ml/2, -ml/2, 0]], dtype=np.float32)
    self.state = "searching"
    self.aligned_since = None
    self.frame_center_x = None
    self.centroid_x = None
    self.centroid_y = None
    self.frame_height = None
    self.kw = 0.006
    self.tol_px = 10
    self.bridge = CvBridge()
    self.msg_twist = Twist()
    self.create_subscription(CameraInfo, 'video_source/camera_info', self.camera_info_cb, 10)
    self.create_subscription(CompressedImage, "video_source/compressed", self.comp_img_clb, 10)
    self.create_subscription(Bool, "a_reached", self.reach_cb, 10)
    self.pub = self.create_publisher(Twist, 'cmd_vel', 1)
    self.pub2 = self.create_publisher(Bool, "aligned_bot2", 1)
    self.create_timer(0.1, self.state_machine)
    self.get_logger().info("Nodo de alineacion iniciado.")
    self.offset = 30
    self.flag = Bool()

    self.max_linear = 0.04
    self.push_linear = 0.04
    self.push_duration = 4.5
    self.push_start_time = None
    self.bottom_threshold = 0.90

    # Ultimo comando conocido antes de perder el marcador
    self.last_linear = 0.0
    self.last_w = 0.0
    # Estado de espera antes de aplicar ultimo comando
    self.lost_wait_start = None
    self.lost_wait_duration = 2.0
    self.a_reached = False

  def reach_cb(self, msg):
    self.a_reached = msg.data
  
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
    self.frame_height = h
    self.centroid_x = None
    self.centroid_y = None
    corners, ids, _ = self.detector.detectMarkers(frame)
    cv2.line(frame, (self.frame_center_x, 0), (self.frame_center_x, h), (0, 255, 255), 2)
    threshold_y = int(h * self.bottom_threshold)
    cv2.line(frame, (0, threshold_y), (w, threshold_y), (0, 0, 255), 2)
    if ids is not None:
      for i in range(len(ids)):
        marker_id = int(ids[i][0])
        if marker_id < 30 or marker_id > 35:
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
          self.centroid_y = cy
    cv2.putText(frame, f"State: {self.state}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
    if self.centroid_y is not None:
      cv2.putText(frame, f"cy: {self.centroid_y} / thr: {threshold_y}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 0), 2)
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
        if self.a_reached:
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
          self.aligned_since = time.time()
          self.state = "advancing"
        else:
          w = -self.kw * error_px
          w = max(min(w, 0.3), -0.3)
          self.move_robot(0.0, w)

      case "advancing":
        if self.centroid_x is None:
          # Perdimos el marcador: esperamos 2s quietos y luego metemos ultimo comando
          if self.lost_wait_start is None:
            self.lost_wait_start = time.time()
            self.stop_robot()
          elif time.time() - self.lost_wait_start >= self.lost_wait_duration:
            self.lost_wait_start = None
            self.state = "pushing"
            self.push_start_time = time.time()
          return

        # Si volvio a aparecer, reseteamos el lost_wait
        self.lost_wait_start = None

        if self.centroid_y is not None and self.frame_height is not None:
          if self.centroid_y >= int(self.frame_height * self.bottom_threshold):
            self.push_start_time = time.time()
            self.state = "pushing"
            return

        error_px = self.centroid_x - self.frame_center_x

        w = -self.kw * error_px
        w = max(min(w, 0.3), -0.3)

        kv = 0.0003
        linear = self.max_linear - kv * abs(error_px)
        linear = max(min(linear, self.max_linear), 0.01)

        # Guardamos el ultimo comando calculado
        self.last_linear = linear
        self.last_w = w

        self.move_robot(linear, w)

      case "pushing":
          elapsed = time.time() - self.push_start_time
          if elapsed >= self.push_duration:
              self.move_robot(self.push_linear, 0.0)  # un tick mas
              self.flag.data = True
              self.pub2.publish(self.flag)
              time.sleep(0.1)
              self.stop_robot()
              raise SystemExit
          
          else:
            self.move_robot(self.push_linear, 0.0)

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