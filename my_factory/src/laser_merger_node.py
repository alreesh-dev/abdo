#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class LaserMerger(Node):
    def __init__(self):
        super().__init__('laser_merger_node')
        # المواضيع التي تظهر في سجلاتك: /front_right_scan و /back_left_scan
        self.sub_front = self.create_subscription(LaserScan, '/front_right_scan', self.front_callback, 10)
        self.sub_back = self.create_subscription(LaserScan, '/back_left_scan', self.back_callback, 10)
        self.publisher = self.create_publisher(LaserScan, '/scan', 10)
        
        self.front_data = None
        self.back_data = None

    def front_callback(self, msg):
        self.front_data = msg
        self.merge_and_publish()

    def back_callback(self, msg):
        self.back_data = msg
        self.merge_and_publish()

    def merge_and_publish(self):
        if self.front_data is None or self.back_data is None:
            return
        
        merged_scan = self.front_data # نأخذ خصائص الهيدر من الليزر الأمامي
        merged_scan.header.frame_id = 'base_link'
        
        # دمج البيانات (ببساطة نأخذ القيمة الأصغر بين المستشعرين لكل زاوية)
        combined_ranges = np.minimum(self.front_data.ranges, self.back_data.ranges)
        merged_scan.ranges = combined_ranges.tolist()
        
        self.publisher.publish(merged_scan)

def main(args=None):
    rclpy.init(args=args)
    node = LaserMerger()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()