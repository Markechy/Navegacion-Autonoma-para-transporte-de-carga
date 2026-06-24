import rclpy, tf_transformations
import numpy as np
from rclpy import qos
from rclpy.node import Node
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from math import sin, cos, sqrt, atan2, pi

class DeadReckoningClass(Node):
  def __init__(self):
    super().__init__("localisation")
    self.declare_parameter('child-frame-id', '')
    self.declare_parameter('x0', 0.0)
    self.declare_parameter('y0', 0.0)
    self.declare_parameter('q0', 0.0)
    self.rate=20

    # Covariance
    sig2R, sig2L = 0.0331, 0.0333
    self.Lk = np.matrix([[sig2R, 0.00001],[0.00001, sig2L]])
    sig = 0.1
    self.Rk = np.matrix([[sig**2, -0.0001],[-0.0001, sig**2]])
    self.Ek = np.matrix([[0.0, 0.0, 0.0],[0.0, 0.0, 0.0],[0.0, 0.0, 0.0]])

    self.create_timer(1.0/self.rate, self.odom_clb)
    self.wR, self.wL = 0.0, 0.0
    self.R, self.L = 0.0473, 0.182
    if self.get_clock().ros_time_is_active:
      self.get_logger().info("Using SIM TIME")
      self.q = self.get_parameter("q0").value
      self.x = self.get_parameter("x0").value - (0.05 * cos(self.q))
      self.y = self.get_parameter("y0").value - (0.05 * sin(self.q))
    else:
      self.get_logger().info("Using SYSTEM TIME")
      self.q = self.get_parameter("q0").value
      self.x = self.get_parameter("x0").value
      self.y = self.get_parameter("y0").value
    self.tf_msg = TransformStamped()
    self.p = TransformBroadcaster(self)
    self.tf_msg.header.frame_id = "world"
    self.tf_msg.child_frame_id = self.get_parameter("child-frame-id").get_parameter_value().string_value+"base_footprint"
    self.pub_odom=self.create_publisher(Odometry, "odom", 1)
    self.o_msg = Odometry()
    self.o_msg.header.frame_id = "world"
    self.o_msg.child_frame_id = self.get_parameter("child-frame-id").get_parameter_value().string_value+"base_footprint"
    my_qos=qos.qos_profile_sensor_data
    self.create_subscription(Float32, "VelocityEncR", self.wR_clb, my_qos)
    self.create_subscription(Float32, "VelocityEncL", self.wL_clb, my_qos)
    self.create_subscription(PoseArray, 'aruco_poses', self.aruco_clb, 10)
    self.create_subscription(PoseArray, 'mini_aruco_pose', self.mini_aruco_clb, 10)
    self.create_subscription(Float32, "heading", self.heading_cb, 1)
    self.path_pub = self.create_publisher(Path, 'robot_path', 10)
    self.path_msg = Path()
    self.path_msg.header.frame_id = 'world'
    self.arucos = {}
    self.t0=self.get_clock().now()
    self.q_init= False
    self.delay=0.0

  def aruco_clb(self, msg):
    self.arucos = {}
    for aruco in msg.poses:
      self.arucos[int((aruco.position.z - 0.1)*10)] = [aruco.position.x, aruco.position.y]
    print(self.arucos)
  
  def heading_cb(self, msg):
    if not self.q_init:
      self.q_init= True
      self.q= msg.data 
      
  def mini_aruco_clb(self, msg):
    self.mini_arucos = {}
    for mini_aruco in msg.poses:
      self.mini_arucos[int((mini_aruco.position.z - 0.1)*10)] = [mini_aruco.position.x, mini_aruco.position.y]

  def getG(self, x, y):
    dk = sqrt((x-self.x)**2 + (y-self.y)**2)
    ak = atan2(y-self.y, x-self.x) - self.q
    ak = atan2(sin(ak), cos(ak))
    return np.matrix([[dk], [ak]])

  def odom_clb(self):
    if self.q_init:
      print(self.q)
      now=self.get_clock().now()
      elap=(now - self.t0).nanoseconds/1e9
      self.t0=now

      w = (self.wR - self.wL)*self.R/self.L
      v = (self.wR + self.wL)*self.R/2.0

      # Best prediction of the robot (Mu)
      if self.delay >= 10.0:
        self.x += v * elap * cos(self.q)
        self.y += v * elap * sin(self.q)
        self.q += w * elap

      Muk = np.matrix([[self.x], [self.y], [self.q]])

      self.Lk = 10.0 * np.matrix([[0.00816*abs(self.wR)+0.00169, 0],[0, 0.0664*abs(self.wL)+0.0242]])

      # Update of covariance of state Ek
      sq, cq = sin(self.q), cos(self.q)
      Hk = np.matrix([[1.0, 0.0, -elap*v*sq], [0.0, 1.0, elap*v*cq], [0.0, 0.0, 1.0]])
      Fk = 0.5*self.R*elap*np.matrix([[cq, cq], [sq, sq], [2/self.L, -2/self.L]])
      self.Ek = Hk*self.Ek*Hk.T + Fk*self.Lk*Fk.T
      g = []

      if self.delay >= 20.0:
        for a in self.arucos:
          match a:
            case 0:
              g = self.getG(1.4, -0.06)
            case 1:
              g = self.getG(0.0, 1.0)
            case 2:
              g = self.getG(-1.4, 0.06)
            case 3:
              g = self.getG(0.0, -1.0)
            case _:
              g = self.getG(0, 0)

          xmi = self.arucos[a][0]
          ymi = self.arucos[a][1]
          dk = sqrt((xmi-self.x)**2 + (ymi-self.y)**2)

          sig = (0.0318*abs(dk**2) + 0.0367) * 2.0
          self.Rk = 6.0 * np.matrix([[sig**2, -0.0001],[-0.0001, sig**2]])

          if abs(dk) < 3.5:
            ak = atan2(ymi-self.y, xmi-self.x) - self.q
            ak = atan2(sin(ak), cos(ak))
            Zk = np.matrix([[dk], [ak]])

            dx, dy = xmi-self.x, ymi-self.y
            Gk = np.matrix([[-dx/sqrt(dx**2 + dy**2), -dy/sqrt(dx**2 + dy**2), 0],
                            [dy/(dx**2 + dy**2), -dx/(dx**2 + dy**2), -1]])
            Kk = self.Ek*Gk.T*np.linalg.inv(Gk*self.Ek*Gk.T+self.Rk)
            Muk = Muk + Kk*(Zk - g)
            self.x = Muk[0,0]
            self.y = Muk[1,0]
            self.q = Muk[2,0]
            self.Ek = self.Ek - Kk*Gk*self.Ek

      self.delay += 1.0


      self.tf_msg.header.stamp = now.to_msg()
      self.tf_msg.transform.translation.x = self.x
      self.tf_msg.transform.translation.y = self.y
      self.tf_msg.transform.translation.z = 0.0
      #print(self.q)
      q = tf_transformations.quaternion_from_euler(0, 0, self.q)
      self.tf_msg.transform.rotation.x = q[0]
      self.tf_msg.transform.rotation.y = q[1]
      self.tf_msg.transform.rotation.z = q[2]
      self.tf_msg.transform.rotation.w = q[3]
      self.p.sendTransform(self.tf_msg)

      self.o_msg.header.stamp = self.tf_msg.header.stamp
      self.o_msg.pose.pose.position.x = self.x
      self.o_msg.pose.pose.position.y = self.y
      self.o_msg.pose.pose.position.z = 0.0

      self.o_msg.pose.pose.orientation.x = q[0]
      self.o_msg.pose.pose.orientation.y = q[1]
      self.o_msg.pose.pose.orientation.z = q[2]
      self.o_msg.pose.pose.orientation.w = q[3]
      self.o_msg.pose.covariance[0] = self.Ek[0,0]
      self.o_msg.pose.covariance[1] = self.Ek[0,1]
      self.o_msg.pose.covariance[5] = self.Ek[0,2]
      self.o_msg.pose.covariance[6] = self.Ek[1,0]
      self.o_msg.pose.covariance[7] = self.Ek[1,1]
      self.o_msg.pose.covariance[11] = self.Ek[1,2]
      self.o_msg.pose.covariance[30] = self.Ek[2,0]
      self.o_msg.pose.covariance[31] = self.Ek[2,1]
      self.o_msg.pose.covariance[35] = self.Ek[2,2]
      self.pub_odom.publish(self.o_msg)

      self.tf_msg.header.stamp = self.tf_msg.header.stamp
      pose = PoseStamped()
      pose.header.stamp = self.tf_msg.header.stamp

      pose.pose.position.x = self.x
      pose.pose.position.y = self.y
      pose.pose.position.z = 0.0

      pose.pose.orientation.x = q[0]
      pose.pose.orientation.y = q[1]
      pose.pose.orientation.z = q[2]
      pose.pose.orientation.w = q[3]
      self.path_msg.poses.append(pose)

      self.path_pub.publish(self.path_msg)

  def wR_clb(self, msg):
    self.wR = msg.data

  def wL_clb(self, msg):
    self.wL = msg.data


def main(args=None):
  rclpy.init(args=args)
  node = DeadReckoningClass()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    print("Terminated by user")

if __name__ == '__main__':
  main()