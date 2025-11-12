#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np

class ConeDetector(Node):
    def __init__(self):
        super().__init__('cone_detector')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/depth_camera/image_raw',   # usa o feed certo
            self.image_callback,
            10)
        self.publisher = self.create_publisher(Bool, '/cone_detected', 10)
        self.get_logger().info('Cone detector ativo')

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Erro na conversão da imagem: {e}')
            return

        hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (5,5), 0), cv2.COLOR_BGR2HSV)

        # --- Máscara laranja ---

        lower_orange = np.array([2, 120, 80])
        upper_orange = np.array([25, 255, 255])
        mask_orange = cv2.inRange(hsv, lower_orange, upper_orange)

        # --- Máscara branca ---
        lower_white = np.array([0, 0, 180])
        upper_white = np.array([180, 60, 255])
        mask_white = cv2.inRange(hsv, lower_white, upper_white)

        # Combina as duas (procura áreas próximas)
        combined = cv2.bitwise_or(mask_orange, mask_white)

        contours, _ = cv2.findContours(combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detected = False

        for c in contours:
            area = cv2.contourArea(c)
            if area < 400:
                continue
            x, y, w, h = cv2.boundingRect(c)
            aspect_ratio = h / float(w)

            # o cone é vertical (altura maior que largura)
            if aspect_ratio < 1.3:
                continue

            # elimina superfícies muito grandes (paredes)
            if h > frame.shape[0] * 0.8 or w > frame.shape[1] * 0.5:
                continue

            detected = True
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

        self.publisher.publish(Bool(data=detected))
        if detected:
            self.get_logger().info('Cone detetado')

        # Debug visual
        cv2.imshow("Mask Orange", mask_orange)
        cv2.imshow("Mask White", mask_white)
        cv2.imshow("Combined", combined)
        cv2.imshow("View", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ConeDetector()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
