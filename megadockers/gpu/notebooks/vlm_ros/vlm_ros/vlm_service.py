#!/usr/bin/env python3
"""
VLM Service Node - Interfaces with llama.cpp server for vision-language tasks
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import requests
import base64
import json
import cv2
import numpy as np


class VLMService(Node):
    def __init__(self):
        super().__init__('vlm_service')

        # Parameters
        self.declare_parameter('llama_server_url', 'http://localhost:8080')
        self.llama_url = self.get_parameter('llama_server_url').value

        # Subscriber for images
        self.image_sub = self.create_subscription(
            Image,
            'camera/image',
            self.image_callback,
            10
        )

        # Publisher for VLM responses
        self.response_pub = self.create_publisher(
            String,
            'vlm/response',
            10
        )

        self.bridge = CvBridge()
        self.get_logger().info(f'VLM Service started, connecting to {self.llama_url}')

    def image_callback(self, msg):
        """Process incoming image and query VLM"""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Encode image to base64
            _, buffer = cv2.imencode('.jpg', cv_image)
            img_base64 = base64.b64encode(buffer).decode('utf-8')

            # Query llama.cpp server
            prompt = "Describe what you see in this image."
            response = self.query_vlm(prompt, img_base64)

            # Publish response
            response_msg = String()
            response_msg.data = response
            self.response_pub.publish(response_msg)

            self.get_logger().info(f'VLM Response: {response[:100]}...')

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def query_vlm(self, prompt, image_base64):
        """Query llama.cpp server with image and prompt"""
        payload = {
            "prompt": prompt,
            "image_data": [{"data": image_base64, "id": 1}],
            "n_predict": 128
        }

        try:
            response = requests.post(
                f'{self.llama_url}/completion',
                json=payload,
                timeout=30
            )
            response.raise_for_status()
            result = response.json()
            return result.get('content', 'No response')
        except Exception as e:
            self.get_logger().error(f'Error querying VLM: {str(e)}')
            return f'Error: {str(e)}'


def main(args=None):
    rclpy.init(args=args)
    node = VLMService()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
