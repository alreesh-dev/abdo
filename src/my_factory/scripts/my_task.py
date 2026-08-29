#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import time

class IndustrialMissionManager(Node):
    def __init__(self):
        super().__init__('industrial_mission_manager')
        # 1. تهيئة الملاح
        self.nav = BasicNavigator()
        self.get_logger().info("--- جاري تشغيل نظام الملاحة الصناعي (نسخة الثبات عند الوصول) ---")
        
        # 2. الانتظار حتى يصبح Nav2 نشطاً بالكامل 
        self.nav.waitUntilNav2Active()
        
        self.get_logger().info("نظام Nav2 نشط.")
        
        # 3. تأخير بسيط لضمان استقرار شجرة التحويلات
        time.sleep(2) 
        
        self.get_logger().info("نظام التحويلات مستقر. جاهز للمهمة.")

    def create_pose(self, x, y, yaw_z=0.0, yaw_w=1.0):
        """إنشاء نقطة هدف احترافية مع التحكم باتجاه الروبوت"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.nav.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.z = float(yaw_z)  # تحديد زاوية الدوران Z
        pose.pose.orientation.w = float(yaw_w)  # تحديد زاوية الدوران W
        return pose

    def execute_mission(self, waypoints):
        """تنفيذ المهمة بالانتقال بين النقاط مع انتظار صامت عند كل نقطة"""
        
        for i, goal in enumerate(waypoints):
            self.get_logger().info(f"جاري التوجه للهدف رقم {i+1}: {goal.pose.position.x}, {goal.pose.position.y}")
            
            # إرسال الهدف
            self.nav.goToPose(goal)
            
            # مراقبة الوصول
            while not self.nav.isTaskComplete():
                time.sleep(0.5)
            
            # تحليل النتيجة
            result = self.nav.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f"تم الوصول للهدف رقم {i+1} بنجاح.")
                
                # 💡 التعديل هنا: الانتظار 10 ثوانٍ بدلاً من 5
                self.get_logger().info("جاري الانتظار في الموقع (10 ثوانٍ)...")
                time.sleep(10) 
                
            else:
                self.get_logger().error(f"فشل الوصول للهدف رقم {i+1}!")
                break

def main():
    rclpy.init()
    manager = IndustrialMissionManager()
    
    # 🟢 تعريف قائمة المهام بالإحداثيات الجديدة 🟢
    mission_goals = [
        # 💡 التعديل هنا: الهدف الأول (X=2.0, Y=-17.0)
        manager.create_pose(2.0, -17.0, yaw_z=0.0, yaw_w=1.0),
        
        # 💡 التعديل هنا: الهدف الثاني (X=-19.0, Y=-19.0)
        manager.create_pose(-19.0, -19.0, yaw_z=0.0, yaw_w=1.0)
    ]
    
    try:
        manager.execute_mission(mission_goals)
        manager.get_logger().info("تمت المهمة بنجاح.")
    except KeyboardInterrupt:
        manager.get_logger().info("تم إيقاف المهمة من قبل المستخدم.")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()