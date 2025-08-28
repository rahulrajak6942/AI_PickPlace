#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
# from ultralytics import YOLO
from geometry_msgs.msg import PoseStamped

class Yolov8DetectionNode(Node):
    def __init__(self):
        super().__init__('yolov8_detection_node')
        # Initialize cv_bridge
        self.bridge = CvBridge()
        self.pose_pub = self.create_publisher(PoseStamped, '/detected_target_pose', 10)
        self.scale=0.0
        # Subscribe to camera image topic (adjust topic as needed)
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Change to your actual camera image topic
            self.image_callback,
            10)

        # Load pretrained YOLO model
        # self.model = YOLO('yolo11n.pt')  # Change to your YOLOv11 model if needed

        # Define class indices for bottle 
        # self.class_names = self.model.names

    def image_callback(self, msg):
        self.calib(msg)
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define HSV ranges for red & blue (tweak as needed)
        lower_red1 = (0, 100, 100)
        upper_red1 = (10, 255, 255)
        lower_red2 = (160, 100, 100)
        upper_red2 = (179, 255, 255)

        lower_blue = (100, 150, 50)
        upper_blue = (140, 255, 255)

    

        # Create masks
        mask_red = cv2.inRange(hsv, lower_red1, upper_red1) | cv2.inRange(hsv, lower_red2, upper_red2)
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

        # Find contours for both colors
        for color, mask in [ ('blue', mask_blue)]:
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                

                x, y, w, h = cv2.boundingRect(cnt)
                x_centroid = x + w // 2
                y_centroid = y + h // 2

                # Draw detection
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(cv_image, color, (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.circle(cv_image, (x_centroid, y_centroid), 5, (0, 0, 255), -1)
                # Publish pose (same scaling you already had)
                pose = PoseStamped()
                pose.header.frame_id = "world"
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x = (y_centroid - 240)*self.scale / 100.0 
                pose.pose.position.y = (x_centroid - 320)*self.scale / 100.0
                pose.pose.position.z = 0.1
                pose.pose.orientation.w = 1.0
                self.pose_pub.publish(pose)

                self.get_logger().info(f"Published {color} cylinder pose: {pose.pose.position}")

        # Show detection image
        cv2.imshow('Cylinder Detection', cv_image)
        cv2.waitKey(1)
    
    def calib(self,msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_green = (35, 40, 40)
        upper_green = (85, 255, 255)
        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        for color, mask in [('green', mask_green)]:
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                

                x, y, w, h = cv2.boundingRect(cnt)
                x_centroid = x + w // 2
                y_centroid = y + h // 2

                # Draw detection
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(cv_image, color, (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.circle(cv_image, (x_centroid, y_centroid), 5, (0, 0, 255), -1)


                y_centroid = 240-y_centroid

                
                
                self.scale = 20/y_centroid
        


               
                
        




def main(args=None):
    rclpy.init(args=args)
    node = Yolov8DetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
