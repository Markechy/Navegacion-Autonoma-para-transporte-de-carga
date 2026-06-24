import rclpy, tf_transformations
from rclpy.node import Node
from nav_msgs.msg import Odometry
from math import sin, cos, sqrt, atan2, pi, radians, degrees
from math import radians as rads
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float32, Bool, String
import numpy as np
from time import sleep, time

class BugAlgoClass(Node):
  def __init__(self):
    super().__init__("reactive_navigation")
    self.get_logger().info("Reactive navigation node has been started...")
    self.create_timer(0.1, self.state_machine)
    self.create_subscription(Odometry, "odom", self.odom_clb, 1)
    self.create_subscription(Pose, "target", self.target_clb, 1)
    self.create_subscription(LaserScan, "scan", self.lidar_clb, 1)
    self.create_subscription(PoseArray, 'mini_aruco_pose', self.mini_aruco_clb, 10)
    self.create_subscription(PoseArray, 'aruco_poses', self.aruco_clb, 10)
    self.create_subscription(Bool, "aligned_bot", self.align_clb, 10)
    self.create_subscription(Bool, "aligned_bot2", self.align_clb2, 10)
    self.target_pub = self.create_publisher(Pose, "target", 1)
    self.pub = self.create_publisher(Twist, "cmd_vel", 1)
    self.pub2 = self.create_publisher(Float32, "ServoAngle", 1)
    self.pub3= self.create_publisher(Float32, "heading", 1)
    self.pub4= self.create_publisher(Bool, "a_reached", 1)
    self.pub5 = self.create_publisher(String, "sound" ,1)



    self.i_pose = []
    self.c_pose = [0.0,0.0,0.0]
    self.t_pose = []
    self.n_target = False
    self.tol_target = 0.03
    self.flag2 = False
    self.flag3 = False
    self.flag4 = False
    self.flag5 = False
    self.flag6 = False
    self.flag7 = False
    self.flag8 = False
    self.flag9 = False
    self.flag10 = False
    self.beta = 0.0

    self.kl, self.kw = 0.5, 1.5

    self.msg = Twist()

    self.readings = []
    self.robot_view = {sector: 100.0 for sector in ['front_right', 'right', 'back', 'left', 'front_left', 'front']}
    self.sectors = [-158.0,-112.0,-68.0, 68.0, 112.0, 158.0]
    #self.sectors = [22.0, 68.0, 112.0, 248.0, 292.0, 338.0]

    self.state = "send_heading"
    self.lpt = 0.0
    self.mini_arucos = {}
    self.arucos= {}
    self.offset = 12.0

    self.nemo_was_found = False
    self.box_was_collected = False
    self.box_was_dropped= False 
    self.flag=False

    self.theta = 0.0
    self.a = 0.0
    self.ax = 0.0
    self.ay = 0.0
    self.b = 0.0
    self.bx = 0.0
    self.by = 0.0
    self.c = 0.30
    self.cx = 0.0
    self.cy = 0.0
    self.lastEq= 0.0

    self.t0 = time()
    self.ti = 0.0

  def send_target(self, x, y, theta=0.0):
      msg = Pose()
      msg.x = x
      msg.y = y
      msg.theta = theta
      self.target_pub.publish(msg)
  
  def mini_aruco_clb(self, msg):
    self.mini_arucos = {}
    for mini_aruco in msg.poses:
      x = mini_aruco.position.x
      y = mini_aruco.position.y
      k = mini_aruco.orientation
      q = [k.x, k.y, k.z, k.w]
      _, _, yaw = tf_transformations.euler_from_quaternion(q)
      self.mini_arucos[0] = [ mini_aruco.position.x, mini_aruco.position.y, (90-degrees(yaw))]
  
  def aruco_clb(self, msg):
    self.arucos = {}
    for aruco in msg.poses:
      self.arucos[int((aruco.position.z - 0.1)*10)] = [aruco.position.x, aruco.position.y]

  def align_clb(self,msg):
    self.flag = msg.data

  def align_clb2(self,msg):
    self.flag8 = msg.data

  def lidar_clb(self, msg):
    ranges = np.array(msg.ranges)

    self.readings = np.nan_to_num(ranges, nan=msg.range_min, posinf=msg.range_max, neginf=msg.range_max)

    angles_deg = np.degrees(msg.angle_min) + np.arange(len(ranges)) * np.degrees(msg.angle_increment)

    mask_fl = (angles_deg > self.sectors[0]) & (angles_deg <= self.sectors[1])
    mask_l = (angles_deg > self.sectors[1]) & (angles_deg <= self.sectors[2])
    mask_b = (angles_deg > self.sectors[2]) & (angles_deg <= self.sectors[3])
    mask_r = (angles_deg > self.sectors[3]) & (angles_deg <= self.sectors[4])
    mask_fr = (angles_deg > self.sectors[4]) & (angles_deg < self.sectors[5])
    mask_f = (angles_deg <= self.sectors[0]) | (angles_deg >= self.sectors[5])

    self.robot_view = {
      'front_right' : float(np.min(self.readings[mask_fr])),
      'right' : float(np.min(self.readings[mask_r])),
      'back' : float(np.min(self.readings[mask_b])),
      'left' : float(np.min(self.readings[mask_l])),
      'front_left' : float(np.min(self.readings[mask_fl])),
      'front' : float(np.min(self.readings[mask_f]))
    }

  def target_clb(self, msg):
    new_target = [msg.x, msg.y, msg.theta]
    if len(self.t_pose) == 0 or new_target != self.t_pose:
      self.t_pose = new_target
      self.i_pose = self.c_pose
      self.n_target = True
      self.lpt = self.get_clock().now()
      self.get_logger().info("Got new target.")

  def odom_clb(self, data):
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    k = data.pose.pose.orientation
    q = [k.x, k.y, k.z, k.w]
    _, _, yaw = tf_transformations.euler_from_quaternion(q)
    self.c_pose = [x, y, yaw]
    #print(f"C_pose: {self.c_pose}")

  def move_robot(self, v, w):
    self.msg.linear.x = min(max(v, -0.15), 0.15)
    self.msg.angular.z = min(max(w, -1.0), 1.0)
    if ((self.c_pose[1] > 0.8 or self.c_pose[1] < -0.8) or (self.c_pose[0] > 1.2 or self.c_pose[0] < -1.2)):
      if not self.flag10:
        self.t0 = time()
        print(self.t0)
        self.flag10 = True
      if self.t0 + 3.7 > time():
        self.msg.linear.x = 0.0
        self.msg.angular.z = 0.8
        self.pub.publish(self.msg)
        print("Girando")
      elif self.t0 + 3.7 + 1.5 > time():
        self.msg.linear.x = 0.15
        self.msg.angular.z = 0.0
        self.pub.publish(self.msg)
        print("Regresando")
      else:
        self.flag10 = False
        
    else:
      self.pub.publish(self.msg)

  def move_lift(self, angle):
    msg = Float32()
    msg.data = angle
    self.pub2.publish(msg)

  def set_heading(self, angle):
    msg = Float32()
    msg.data = angle
    self.pub3.publish(msg)


  def isAruco(self):
    return len(self.arucos) > 0 


  def finding_nemo(self):
    if self.flag:
      self.send_target(0.7, 0.0)
      for a in self.arucos:
      #print(self.arucos)
        match a:
          case 0:
            self.set_heading(0.0)
          case 1:
            self.set_heading(pi/2.0)
          case 2:
            self.set_heading(pi)
          case 3:
            self.set_heading(-pi/2.0)
        
      self.nemo_was_found = True

  def stop_robot(self):
    self.move_robot(0.0, 0.0)

  def go_to_goal(self):
    Ex = self.t_pose[0] - self.c_pose[0]
    Ey = self.t_pose[1] - self.c_pose[1]
    Eq = atan2(Ey, Ex) - self.c_pose[2]
    Eq = atan2(sin(Eq), cos(Eq))

    dis = sqrt(Ex**2 + Ey**2)

    self.move_robot(min(dis*self.kl, 0.4), max(min(Eq*self.kw, 3.0), -3.0))

  def collecting_minibox(self):
    if not self.flag2:

      if (self.mini_arucos[0][2]) >= 0:
        self.theta = 90.0 - (self.mini_arucos[0][2])
      else:
        self.theta = -90 - (self.mini_arucos[0][2])

      self.a = sin(radians(self.mini_arucos[0][2])) * (self.mini_arucos[0][0])
      self.a += 0.0
      self.ax = sin(self.c_pose[2] + radians(self.theta)) * self.a + self.c_pose[0]
      self.ay = cos(self.c_pose[2] + radians(self.theta)) * self.a + self.c_pose[1]
      print("Meloarucos")
      print(self.mini_arucos[0][2])
      print(self.mini_arucos[0][0])
      self.b = cos(radians(self.mini_arucos[0][2])) * (self.mini_arucos[0][0])
      self.b -= 0.0
      self.bx = sin(self.c_pose[2] + (pi/2)) * (self.b + self.c_pose[0])
      self.by = cos(self.c_pose[2] + (pi/2)) * (self.b + self.c_pose[1])
      if len(self.mini_arucos) > 0:
        self.bx = self.mini_arucos[0][0] + (0.085 * cos(self.c_pose[2]))
        self.by = self.mini_arucos[0][1] + (0.085 * sin(self.c_pose[2]))

      print("Debugging Manual")
      print(self.ax, self.ay)
      print(self.bx, self.by)
      print(self.c_pose)

      print(f'theta: {self.theta}, a: {self.a}, b: {self.b}')
      print(self.mini_arucos[0])
      self.flag2 = True
      self.move_lift(75.0)
      print("Bajando pala")

    Eq = radians(self.theta) - (self.c_pose[2])
    Eq = atan2(sin(Eq), cos(Eq))

    if Eq >= 0 :
      w = 0.3
    else :
      w = -0.3

    if abs(Eq) > 0.1 and (not self.flag4) :
      self.move_robot(0.0, w)
      print("Corriguiendo tetha")

    else:
      print(self.c_pose)
      Ex = self.ax - self.c_pose[0]
      Ey = self.ay - self.c_pose[1]
      self.flag4 = True

      if abs(Ex) > 0.02 and abs(Ey) > 0.02 and (not self.flag5):
        v = 0.10
        self.move_robot(v,0.0)
        print("Corrigiendo a")
        print(Ex)
        print(Ey)

      else:

        if not self.flag5:
          if self.theta <= 0:
            self.beta = self.c_pose[2] + (pi/2) + 0.08
            self.offset *= -1.0
          else:
            self.beta = self.c_pose[2] - (pi/2) + 0.08

        self.flag5 = True
      
        Eq = self.beta - (self.c_pose[2])
        Eq = atan2(sin(Eq), cos(Eq))

        if len(self.mini_arucos) > 0:
          Eq = -1.0 * radians(self.mini_arucos[0][2] + self.offset)

        if Eq >= 0 :
          w = 0.3
        else:
          w = -0.3

        if abs(Eq) > 100.0 and (not self.flag6):
          self.move_robot(0.0, w)
          self.lastEq = self.c_pose[2]
          print("Corrigiendo 90 grados")

        else: 
          if not self.flag6:
            self.move_robot(0.0, 0.0)
            msg= Bool()
            msg.data= True
            self.pub4.publish(msg)
          self.flag6 = True
          
          if not self.flag8:
            print("Corrigiendo b")
          else:
            if not self.flag7:
              self.move_lift(-75.0)
              msg = String()
              msg.data = "hehe"
              self.pub5.publish(msg)
            self.flag7 = True
            self.flag8 = True

            if not self.flag9:
              self.cx = self.c_pose[0] - cos(self.c_pose[2]) * (self.c)
              self.cy = self.c_pose[1] - sin(self.c_pose[2]) * (self.c)
              self.flag9 = True

            Ex = self.cx - self.c_pose[0]
            Ey = self.cy - self.c_pose[1]
            print("Reverzon")
            print(f"ex {Ex}, ey {Ey}")

            v = -0.1
            self.move_robot(v,0.0)

            if abs(Ex) < 0.03 and abs(Ey) < 0.05:
              self.box_was_collected = True
              self.send_target(-1.05, 0.0)

  def drop_box(self):
    if not self.flag3:
      self.move_lift(75.0)

      self.cx = self.c_pose[0] - cos(self.c_pose[2]) * (self.c)
      self.cy = self.c_pose[1] - sin(self.c_pose[2]) * (self.c)
      self.flag3 = True

    Ex = self.cx - self.c_pose[0]
    Ey = self.cy - self.c_pose[1]

    v = -0.1
    self.move_robot(v,0.0)

    if abs(Ex) < 0.03 and abs(Ey) < 0.05:
      self.box_was_dropped= True 
      self.send_target(0.0, 0.0)

  def turning(self):
    self.move_robot(0.0, 0.25)
  
  def atTarget(self):
    Ex = self.t_pose[0] - self.c_pose[0]
    Ey = self.t_pose[1] - self.c_pose[1]
    dis = sqrt(Ex**2 + Ey**2)
    return dis <= self.tol_target

  def gotNewTarget(self):
    return self.n_target

  def follow_wall(self):
    front = self.robot_view.get("front")
    right = self.robot_view.get("right")
    front_right = self.robot_view.get("front_right")
    left = self.robot_view.get("left")
    front_left= self.robot_view.get("front_left")

    dis_to_wall = 0.25
    tol = 0.05

    if right < left:
      if front < 0.3:
        v, w = 0.0, 0.6
      elif front_right < (dis_to_wall - tol):
        v, w = 0.1, 0.5
      elif front_left > (dis_to_wall + tol):
        v, w = 0.1, -0.5
      else:
        v, w = 0.2, 0.0
    else:
      if front < 0.3:
        v, w = 0.0, -0.6
      elif front_left < (dis_to_wall - tol):
        v, w = 0.1, -0.5
      elif front_right > (dis_to_wall + tol):
        v, w = 0.1, 0.5
      else:
        v, w = 0.2, 0.0

    self.move_robot(v,w)

  def isWallCleared(self):
    Ex = self.t_pose[0] - self.c_pose[0]
    Ey = self.t_pose[1] - self.c_pose[1]
    Eq = atan2(Ey, Ex) - self.c_pose[2]
    Eq = atan2(sin(Eq), cos(Eq))
    return (self.robot_view.get("front") > 0.6 and abs(Eq) < 0.2)

  def isObstacleAhead(self):
    dis_to_obs = self.robot_view.get("front")
    return dis_to_obs < 0.3

  def print_perf(self):
    self.get_logger().info(f"""
                            ---------------------------------------
                            | Point to point Time: {(self.get_clock().now() - self.lpt).nanoseconds/1e9}.
                            | From point: x: {round(self.i_pose[0], 4)} y: {round(self.i_pose[1], 4)}.
                            | To point: x: {self.t_pose[0]} y: {self.t_pose[1]}.
                            ---------------------------------------
                            """)
    
  def isMiniArucoAhead(self):
    return len(self.mini_arucos) > 0
  
  def nemoFound(self):
    return self.nemo_was_found 
  
  def boxCollected(self):
    return self.box_was_collected

  def boxDropped(self):
    return self.box_was_dropped
  
  def isTurned(self):
    return abs(self.c_pose[2]) < 0.1

  def state_machine(self):
    if len(self.c_pose) > 0:
      match self.state:
        case "send_heading":
          self.finding_nemo()
          if self.nemoFound():
            self.state = "stop_robot"
            self.get_logger().info("I found Nemo.")
        case "stop_robot":
          self.stop_robot()
          if self.gotNewTarget():
            self.state = "go_to_goal"
            self.n_target = False
            self.get_logger().info("Going target.")
        case "go_to_goal":
          self.go_to_goal()
          if self.isObstacleAhead():
            self.state = "follow_wall"
            self.get_logger().info("Obstacle reached.")
          if (self.c_pose[0] <= 0.7+ self.tol_target and self.c_pose[0] >= 0.7 - self.tol_target) and (self.c_pose[1] <= 0.0 + self.tol_target and self.c_pose[1] >= -0.0 - self.tol_target ) and (self.atTarget()):
            self.state = "turning"
            self.get_logger().info("Turning")
          elif (self.c_pose[0] >= -1.05 - self.tol_target and self.c_pose[0] <= -1.05 + self.tol_target) and (self.c_pose[1] <= 0.0 + self.tol_target and self.c_pose[1] >= -0.0 - self.tol_target ) and (self.atTarget()):
            self.state = "drop_box"
            self.get_logger().info("Dropping box.")
          elif self.atTarget():
            self.state = "stop_robot"
            self.get_logger().info("Target reached.")
            print(self.c_pose)
            print(self.c_pose[0] <= 0.83 and self.c_pose[0] >= 0.78) 
            print(self.c_pose[1] <= 0.02 and self.c_pose[1] >= -0.02 ) 
            print(self.atTarget()) 
            print(self.isMiniArucoAhead())
            print('---------------------')
            self.print_perf()
        case "follow_wall":
          self.follow_wall()
          if self.isWallCleared():
            self.state = "go_to_goal"
            self.get_logger().info("Obstacle cleared.")
        case "collecting_minibox":
          self.collecting_minibox()
          if self.boxCollected():
            self.state = "stop_robot"
            self.get_logger().info("Box collected.")
        case "drop_box":
          self.drop_box()
          if self.boxDropped():
            self.state= "stop_robot"
            self.get_logger().info("Box dropped.")
        case "turning":
          self.turning()
          if self.isTurned():
            self.state = "collecting_minibox"
            self.get_logger().info("Collecting Mini Box.")



def main(args=None):
  rclpy.init(args=args)
  node = BugAlgoClass()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    print("Terminated by user")

if __name__ == '__main__':
  main()