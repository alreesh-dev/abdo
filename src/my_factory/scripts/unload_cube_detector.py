import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

def empty(a):
    pass

class LoadArmVision(Node):
    def __init__(self):
        # 🟢 1. تم تعديل اسم العقدة لتدل على أنها خاصة بذراع التحميل
        super().__init__('load_arm_vision_node')
        self.get_logger().info("👀 نظام الرؤية لذراع التحميل (Load Arm) يعمل! جاري البحث عن المكعب...")
        
        self.bridge = CvBridge()
        
        # مسارات الكاميرا بقيت كما هي لأننا ربطناها في الـ Bridge بنفس الاسم
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
            
        self.depth_sub = self.create_subscription(
            Image, '/camera/image_raw/depth_image', self.depth_callback, 10)
            
        # 🟢 2. تعديل مسار النشر ليصبح خاصاً بذراع التحميل (Namespace)
        self.target_pub = self.create_publisher(Point, '/load_arm/cube_target_position', 10)
        
        self.target_cx = None
        self.target_cy = None
        self.locked_on_target = False

        # إعدادات الكاميرا (Camera Intrinsics)
        fov = 1.047
        self.width = 640
        self.height = 480
        self.fx = (self.width / 2.0) / math.tan(fov / 2.0)
        self.fy = self.fx
        self.cx_cam = self.width / 2.0
        self.cy_cam = self.height / 2.0
        
        # 🌐 موقع الكاميرا الثابت في خريطة العالم (Ground Truth)
        # متوافق مع إحداثيات load_arm الجديدة
        self.cam_world_x = 6.0
        self.cam_world_y = -16.8
        self.cam_world_z = 1.06

        # نوافذ التحكم بالألوان (HSV)
        cv2.namedWindow("Trackbars")
        cv2.resizeWindow("Trackbars", 400, 250)
        cv2.createTrackbar("Hue Min", "Trackbars", 0, 179, empty)
        cv2.createTrackbar("Hue Max", "Trackbars", 179, 179, empty)
        cv2.createTrackbar("Sat Min", "Trackbars", 0, 255, empty)
        cv2.createTrackbar("Sat Max", "Trackbars", 243, 255, empty)
        cv2.createTrackbar("Val Min", "Trackbars", 0, 255, empty)
        cv2.createTrackbar("Val Max", "Trackbars", 255, 255, empty)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            h_min = cv2.getTrackbarPos("Hue Min", "Trackbars")
            h_max = cv2.getTrackbarPos("Hue Max", "Trackbars")
            s_min = cv2.getTrackbarPos("Sat Min", "Trackbars")
            s_max = cv2.getTrackbarPos("Sat Max", "Trackbars")
            v_min = cv2.getTrackbarPos("Val Min", "Trackbars")
            v_max = cv2.getTrackbarPos("Val Max", "Trackbars")
            
            lower_bound = np.array([h_min, s_min, v_min])
            upper_bound = np.array([h_max, s_max, v_max])
            
            mask = cv2.inRange(img_hsv, lower_bound, upper_bound)
            
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            output_frame = frame.copy()
            # رسم مركز الكاميرا
            cv2.drawMarker(output_frame, (int(self.cx_cam), int(self.cy_cam)), (255, 0, 0), cv2.MARKER_CROSS, 20, 2)
            
            best_contour = None
            min_distance = float('inf')
            best_center = (0, 0)
            
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 500:
                    M = cv2.moments(cnt)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        
                        dist = math.sqrt((cx - self.cx_cam)**2 + (cy - self.cy_cam)**2)
                        
                        if dist < min_distance:
                            min_distance = dist
                            best_contour = cnt
                            best_center = (cx, cy)
            
            if best_contour is not None:
                self.target_cx, self.target_cy = best_center
                self.locked_on_target = True
                
                # رسم التحديد حول الهدف
                cv2.drawContours(output_frame, [best_contour], -1, (0, 255, 0), 3)
                cv2.circle(output_frame, (self.target_cx, self.target_cy), 5, (0, 0, 255), -1)
                
            else:
                self.locked_on_target = False
            
            cv2.imshow("Load Arm Vision System", output_frame)
            cv2.waitKey(1)
            
        except Exception as e:
            pass

    def depth_callback(self, msg):
        if self.locked_on_target and self.target_cx is not None and self.target_cy is not None:
            try:
                depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
                distance_z = depth_image[self.target_cy, self.target_cx]
                
                # 1. تطبيق المعادلات الرياضية للإسقاط الثلاثي الأبعاد
                real_x = (self.target_cx - self.cx_cam) * distance_z / self.fx
                real_y = (self.target_cy - self.cy_cam) * distance_z / self.fy
                
                # 2. تحويل الإحداثيات لتصبح بالنسبة لعالم المحاكي
                world_x = self.cam_world_x - real_y
                world_y = self.cam_world_y + real_x
                world_z = self.cam_world_z - distance_z
                
                # 3. الارتفاع المستهدف للذراع (أعلى من المكعب بـ 15 سم لزوم المسك)
                target_z = world_z + 0.15
                
                self.get_logger().info(f"📍 إحداثيات الهدف للذراع (Load Arm): X={world_x:.3f} | Y={world_y:.3f} | Z={target_z:.3f}")
                
                # 📢 4. بث الإحداثيات على المسار الجديد
                target_msg = Point()
                target_msg.x = world_x
                target_msg.y = world_y
                target_msg.z = target_z
                
                self.target_pub.publish(target_msg)
                
            except Exception as e:
                self.get_logger().error(f"❌ خطأ في معالجة العمق: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = LoadArmVision()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()