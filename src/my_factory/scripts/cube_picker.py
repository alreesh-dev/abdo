import rclpy 
from rclpy.node import Node 
from geometry_msgs.msg import Point, Pose 
from rclpy.action import ActionClient 
from moveit_msgs.action import MoveGroup 
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume 
from shape_msgs.msg import SolidPrimitive 
from control_msgs.action import GripperCommand 

class CubePicker(Node): 
    def __init__(self): 
        super().__init__('cube_picker') 
        self.get_logger().info("🦾 التسلسل الكامل: (البداية ⬅️ تحليق ⬅️ انتظار ⬅️ فتح ⬅️ نزول دقيق ⬅️ إغلاق محكم)...") 
         
        self.target_sub = self.create_subscription(Point, '/cube_target_position', self.target_callback, 10) 
        self.arm_client = ActionClient(self, MoveGroup, 'move_action') 
        self.gripper_client = ActionClient(self, GripperCommand, '/panda_hand_controller/gripper_cmd') 
         
        self.arm_moved = False 
        self.current_state = "IDLE" 
        self.target_x = 0.0 
        self.target_y = 0.0 
        self.target_z = 0.0 
        self.wait_timer = None 

    def target_callback(self, msg): 
        if not self.arm_moved: 
            self.arm_moved = True 
            self.target_x = msg.x 
            self.target_y = msg.y 
            self.target_z = msg.z 
             
            self.get_logger().info(f"📩 تم استلام الهدف: X={msg.x:.3f}, Y={msg.y:.3f}, Z={msg.z:.3f}") 
            self.current_state = "GO_HOME" 
            self.go_home() 

    # ========================================== 
    # 1️⃣ الخطوة الأولى: وضعية البداية 
    # ========================================== 
    def go_home(self): 
        self.get_logger().info("1️⃣ جاري التحرك لوضعية البداية (Home)...") 
        self.arm_client.wait_for_server() 
         
        goal_msg = MoveGroup.Goal() 
        goal_msg.request.group_name = 'panda_arm' 
        goal_msg.request.num_planning_attempts = 15 
        goal_msg.request.allowed_planning_time = 5.0 
        goal_msg.request.max_velocity_scaling_factor = 0.5 
        goal_msg.request.max_acceleration_scaling_factor = 0.5 
         
        home_joints = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785] 

        joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3', 
'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7'] 

        constraints = Constraints() 
        for i in range(7): 
            jc = JointConstraint() 
            jc.joint_name = joint_names[i] 
            jc.position = float(home_joints[i]) 
            jc.tolerance_above = 0.01 
            jc.tolerance_below = 0.01 
            jc.weight = 1.0 
            constraints.joint_constraints.append(jc) 

        goal_msg.request.goal_constraints.append(constraints) 
        self._send_goal_future = self.arm_client.send_goal_async(goal_msg) 
        self._send_goal_future.add_done_callback(self.arm_response_callback) 

    # ========================================== 
    # 2️⃣ و 5️⃣: دالة الذراع (تحليق ونزول) 
    # ========================================== 
    def move_arm_to_cube(self, z_offset): 
        self.arm_client.wait_for_server() 
        tune_y = -0.025
        local_x = (self.target_x - 5.36) 
        local_y = (self.target_y + 16.9) + tune_y 
        local_z = (self.target_z - 0.11) + z_offset 

        goal_msg = MoveGroup.Goal() 
        goal_msg.request.group_name = 'panda_arm' 
        goal_msg.request.num_planning_attempts = 20 
        goal_msg.request.allowed_planning_time = 5.0 
        goal_msg.request.max_velocity_scaling_factor = 0.3 
        goal_msg.request.max_acceleration_scaling_factor = 0.3 
         
        pos_constraint = PositionConstraint() 
        pos_constraint.header.frame_id = 'panda_link0' 
        pos_constraint.link_name = 'panda_hand' 
        pos_constraint.weight = 1.0 
         
        bv = BoundingVolume() 
        sp = SolidPrimitive() 
        sp.type = SolidPrimitive.SPHERE 
        sp.dimensions = [0.015] 
        bv.primitives.append(sp) 
         
        pose = Pose() 
        pose.position.x = float(local_x) 
        pose.position.y = float(local_y) 
        pose.position.z = float(local_z) 
        bv.primitive_poses.append(pose) 
        pos_constraint.constraint_region = bv 

        ori_constraint = OrientationConstraint() 
        ori_constraint.header.frame_id = 'panda_link0' 
        ori_constraint.link_name = 'panda_hand' 
        ori_constraint.weight = 1.0 
         
        ori_constraint.orientation.x = 1.0 
        ori_constraint.orientation.y = 0.0 
        ori_constraint.orientation.z = 0.0 
        ori_constraint.orientation.w = 0.0 
         
        ori_constraint.absolute_x_axis_tolerance = 0.02 
        ori_constraint.absolute_y_axis_tolerance = 0.02 
        ori_constraint.absolute_z_axis_tolerance = 0.02 

        constraints = Constraints() 
        constraints.position_constraints.append(pos_constraint) 
        constraints.orientation_constraints.append(ori_constraint) 
        goal_msg.request.goal_constraints.append(constraints) 

        self._send_goal_future = self.arm_client.send_goal_async(goal_msg) 
        self._send_goal_future.add_done_callback(self.arm_response_callback) 

    # ========================================== 
    # 4️⃣ و 6️⃣: دالة القابض (معدلة للتحكم بقوة العصر) 
    # ========================================== 
    def operate_gripper(self, width, effort): 
        self.get_logger().info(f"✊ إرسال أمر للقابض: المسافة = {width}، القوة (Effort) = {effort} نيوتن") 
        self.gripper_client.wait_for_server() 
         
        goal_msg = GripperCommand.Goal() 
        goal_msg.command.position = float(width) 
        goal_msg.command.max_effort = float(effort) 

        self._send_gripper_future = self.gripper_client.send_goal_async(goal_msg) 
        self._send_gripper_future.add_done_callback(self.gripper_response_callback) 

    def gripper_response_callback(self, future): 
        goal_handle = future.result() 
        if not goal_handle.accepted: 
            self.get_logger().error("❌ تم رفض أمر القابض!") 
            return 
        self._gripper_result_future = goal_handle.get_result_async() 
        self._gripper_result_future.add_done_callback(self.gripper_result_callback) 

    def gripper_result_callback(self, future): 
        if self.current_state == "OPENING": 
            self.get_logger().info("✅ القابض مفتوح! 5️⃣ جاري النزول للمكعب...") 
            self.current_state = "PLUNGE" 
            # 💡 السر هنا: النزول 4.5 سم فقط لتجنب الاصطدام العنيف الذي يجمد المحاكي 
            self.move_arm_to_cube(z_offset=-0.045)  
             
        elif self.current_state == "CLOSING": 
            self.get_logger().info("🎉 تمت المهمة بنجاح! المكعب معصور ومستقر تماماً في قبضتنا. 📦🦾") 

    # ========================================== 
    # ⏰ المؤقت الزمني 
    # ========================================== 
    def timer_callback(self): 
        if self.wait_timer: 
            self.wait_timer.cancel() 
             
        if self.current_state == "WAITING": 
            self.get_logger().info("3️⃣ انتهى الوقت! جاري فتح القابض...") 
            self.current_state = "OPENING" 
            self.operate_gripper(width=0.04, effort=5.0) 

    # ========================================== 
    # 🔄 الردود لحركة الذراع 
    # ========================================== 
    def arm_response_callback(self, future): 
        goal_handle = future.result() 
        if not goal_handle.accepted: 
            self.get_logger().error(f"❌ تم رفض الحركة للحالة: {self.current_state}!") 
            return 
        self._result_future = goal_handle.get_result_async() 
        self._result_future.add_done_callback(self.arm_result_callback) 

    def arm_result_callback(self, future): 
        result = future.result().result 

        if result.error_code.val == 1: 
            if self.current_state == "GO_HOME": 
                self.get_logger().info("✅ الذراع في وضعية البداية! ننتقل للخطوة الثانية...") 
                self.current_state = "HOVER" 
                self.move_arm_to_cube(z_offset=0.00) 

            elif self.current_state == "HOVER": 
                self.get_logger().info("✅ الذراع تقف الآن فوق المكعب عمودياً. ⏳ ننتظر ثانيتين...") 
                self.current_state = "WAITING" 
                self.wait_timer = self.create_timer(2.0, self.timer_callback) 

            elif self.current_state == "PLUNGE": 
                self.get_logger().info("✅ وصلنا للمكعب! 6️⃣ جاري الإغلاق...") 
                self.current_state = "CLOSING" 
                # 💡 نغلق لمسافة 0.01 (عرض المكعب تقريباً) بدلاً من 0.0 بقوة 20 نيوتن لمنع اختراق المجسم 
                self.operate_gripper(width=0.01, effort=20.0) 

        else: 
            self.get_logger().error(f"⚠️ فشلت الحركة في الحالة {self.current_state}! رمز الخطأ: {result.error_code.val}") 

def main(args=None): 
    rclpy.init(args=args) 
    node = CubePicker() 
    rclpy.spin(node) 
    node.destroy_node() 
    rclpy.shutdown() 

if __name__ == '__main__': 
    main()