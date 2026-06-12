#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import time

def main():
    rclpy.init()
    
    # تهيئة الملاح ككائن مستقل لضمان استقراره
    nav = BasicNavigator()
    
    # الانتظار حتى يصبح Nav2 نشطاً
    nav.waitUntilNav2Active()
    print("نظام Nav2 نشط.")
    
    # وظيفة إنشاء الهدف
    def create_pose(x, y):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = nav.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.w = 1.0
        return pose

    # قائمة المهام (تم تصحيح القوس هنا)
    mission_goals = [
        create_pose(-8.018, 6.449),
        create_pose(8.299, -11.171)
    ]
    
    # تنفيذ المهام
    for i, goal in enumerate(mission_goals):
        print(f"جاري التوجه للهدف رقم {i+1}")
        nav.goToPose(goal)
        
        while not nav.isTaskComplete():
            time.sleep(0.5)
        
        if nav.getResult() == TaskResult.SUCCEEDED:
            print(f"وصلنا! انتظار 5 ثوانٍ...")
            time.sleep(5) 
        else:
            print(f"فشل الوصول للهدف رقم {i+1}")
            break
            
    rclpy.shutdown()

if __name__ == '__main__':
    main()