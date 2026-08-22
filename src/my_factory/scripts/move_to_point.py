import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

class NativeArmMover(Node):
    def __init__(self):
        super().__init__('native_arm_mover')
        self.get_logger().info("🦾 تهيئة نظام الحركة المباشر (Native Action Client)...")
        # اسم سيرفر الأكشن الخاص بـ MoveIt
        self.action_client = ActionClient(self, MoveGroup, 'move_action')

    def send_goal(self, map_x, map_y, target_z):
        self.get_logger().info("⏳ جاري الاتصال بمحرك MoveIt...")
        self.action_client.wait_for_server()

        # التحويل الرياضي للإحداثيات من الخريطة إلى الذراع
        local_x = map_x - 5.36
        local_y = map_y + 16.9
        local_z = target_z
        
        self.get_logger().info(f"📍 إحداثيات الخريطة المطلوبة: X={map_x}, Y={map_y}")
        self.get_logger().info(f"🎯 الإحداثيات المحلية للذراع: X={local_x:.2f}, Y={local_y:.2f}, Z={local_z:.2f}")

        # بناء طلب الحركة
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'panda_arm'
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        
        # سرعة آمنة للذراع
        goal_msg.request.max_velocity_scaling_factor = 0.5
        goal_msg.request.max_acceleration_scaling_factor = 0.5
        
        # إجبار النظام على التنفيذ الفوري بعد التخطيط
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.replan = True
        
        # 1. قيد الموضع (Position Constraint)
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = 'panda_link0'
        pos_constraint.link_name = 'panda_hand'
        
        bv = BoundingVolume()
        sp = SolidPrimitive()
        sp.type = SolidPrimitive.SPHERE
        sp.dimensions = [0.01]  # نسبة خطأ مسموحة 1 سم
        bv.primitives.append(sp)
        
        pose = Pose()
        pose.position.x = float(local_x)
        pose.position.y = float(local_y)
        pose.position.z = float(local_z)
        bv.primitive_poses.append(pose)
        
        pos_constraint.constraint_region = bv
        pos_constraint.weight = 1.0

        # 2. قيد الاتجاه (Orientation Constraint) - توجيه القابض للأسفل 
        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = 'panda_link0'
        ori_constraint.link_name = 'panda_hand'
        ori_constraint.orientation.x = 1.0  # هذا الإحداثي يجعل يد الباندا تنظر للأسفل
        ori_constraint.orientation.y = 0.0
        ori_constraint.orientation.z = 0.0
        ori_constraint.orientation.w = 0.0
        ori_constraint.absolute_x_axis_tolerance = 0.1
        ori_constraint.absolute_y_axis_tolerance = 0.1
        ori_constraint.absolute_z_axis_tolerance = 0.1
        ori_constraint.weight = 1.0

        # تجميع القيود وإرسالها
        constraints = Constraints()
        constraints.position_constraints.append(pos_constraint)
        constraints.orientation_constraints.append(ori_constraint)
        goal_msg.request.goal_constraints.append(constraints)

        self.get_logger().info("🚀 جاري إرسال الهدف للتخطيط والتنفيذ...")
        self._send_goal_future = self.action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ تم رفض الهدف: النقطة قد تكون خارج نطاق حركة الذراع!")
            rclpy.shutdown()
            return

        self.get_logger().info("✅ تم قبول التخطيط! الذراع يتحرك الآن...")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.error_code.val == 1:
            self.get_logger().info("🎉 وصل الذراع إلى الهدف بنجاح تام!")
        else:
            self.get_logger().error(f"⚠️ انتهت الحركة برمز خطأ: {result.error_code.val}")
        
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = NativeArmMover()
    
    # الإحداثيات المستخرجة من صورتك + الارتفاع (50 سم)
    map_x = 5.8
    map_y = -16.5
    height_z = 0.35 
    
    node.send_goal(map_x, map_y, height_z)
    
    # إبقاء الكود يعمل حتى ينتهي الروبوت من الحركة
    rclpy.spin(node)

if __name__ == '__main__':
    main()