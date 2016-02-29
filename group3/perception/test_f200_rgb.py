import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class TestF200Rgb:
    def __init__(self):
        self.show = True
        self.bridge = CvBridge()

    def callback(self, data):
        if self.show:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            (rows,cols,channels) = cv_image.shape
            print rows, cols, channels
            cv2.imshow("Image window", cv_image)
            cv2.waitKey(500)
            self.show = False

    def listener(self):
        rospy.init_node("listener", anonymous=True)
        rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)
        rospy.spin()

if __name__ == '__main__':
    TestF200Rgb().listener()