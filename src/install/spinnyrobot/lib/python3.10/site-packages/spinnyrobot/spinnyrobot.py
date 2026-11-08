import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from spinnyrobot.taskmanager import TaskManager
from std_msgs.msg import Float32

bridge = CvBridge()


class SpinnyRobot(Node):
    def __init__(self):
        self.tm = TaskManager()
        super().__init__('spinnyrobot')
        self.anglesub = self.create_subscription(Float32, '/desired_angle', self.angle_sub_callback, 10)
        self.imagepub = self.create_publisher(Image, '/robotcam', 10)
        self.anglepub = self.create_publisher(Float32, '/current_angle', 10)
        self.pubtimer = self.create_timer(1 / 30, self.timer_callback)

    def angle_sub_callback(self, message: Float32):
        self.tm.set_joint_angle(message.data)

    def timer_callback(self):
        rendered_image = self.tm.render_image()
        joint_angle = self.tm.get_joint_angle()
        image = bridge.cv2_to_imgmsg(rendered_image, encoding='bgr8')
        self.imagepub.publish(image)
        angle = Float32()
        angle.data = joint_angle
        self.anglepub.publish(angle)


def main(args=None):
    rclpy.init(args=args)
    sr = SpinnyRobot()

    while True:
        sr.tm.spin_once()
        rclpy.spin_once(sr)
