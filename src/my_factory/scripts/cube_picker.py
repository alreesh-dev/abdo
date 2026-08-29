import rclpy 
from rclpy.node import Node 
from rclpy.action import ActionClient 
from moveit_msgs.action import MoveGroup 
from moveit_msgs.msg import Constraints, JointConstraint 
from control_msgs.action import GripperCommand 

class SimpleArmMover(Node): 
    def __init__(self): 
        super().__init__('load_arm_simple_mover') 
        self.get_logger().info("🦾 وضعية التشغيل: (Home -> مكان1 -> فتح -> مكان2 -> إغلاق -> مكان3)...") 
         
        self.arm_client = ActionClient(self, MoveGroup, 'move_action') 
        self.gripper_client = ActionClient(self, GripperCommand, '/load_hand_controller/gripper_cmd') 
         
        self.current_state = "IDLE" 
        self.timer = self.create_timer(1.0, self.start_sequence)

    def start_sequence(self):
        self.timer.cancel() 
        self.current_state = "GO_HOME"
        self.go_home()

    # ========================================== 
    # 1️⃣ الخطوة الأولى: وضعية البداية (Home)
    # ========================================== 
    def go_home(self): 
        self.get_logger().info("1️⃣ جاري التحرك لوضعية البداية (Home)...") 
        self.arm_client.wait_for_server() 
         
        goal_msg = self.create_arm_goal([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
        self._send_goal_future = self.arm_client.send_goal_async(goal_msg) 
        self._send_goal_future.add_done_callback(self.arm_response_callback) 

    # ========================================== 
    # 2️⃣ الخطوة الثانية: المكان المحفوظ الأول (تحضير)
    # ========================================== 
    def go_to_saved_pose_1(self):
        self.get_logger().info("2️⃣ جاري التحرك للمكان المحفوظ الأول...") 
        self.arm_client.wait_for_server() 
         
        joints_1 = [1.32557, 0.03834, -1.27129, -2.20870, 0.07434, 2.21861, 0.54474] 
        goal_msg = self.create_arm_goal(joints_1)
        self._send_goal_future = self.arm_client.send_goal_async(goal_msg) 
        self._send_goal_future.add_done_callback(self.arm_response_callback)

    # ========================================== 
    # 3️⃣ و 5️⃣: دالة التحكم بالقابض (فتح وإغلاق)
    # ========================================== 
    def operate_gripper(self, width, effort): 
        self.gripper_client.wait_for_server() 
         
        goal_msg = GripperCommand.Goal() 
        goal_msg.command.position = float(width) 
        goal_msg.command.max_effort = float(effort) 

        self._send_gripper_future = self.gripper_client.send_goal_async(goal_msg) 
        self._send_gripper_future.add_done_callback(self.gripper_response_callback) 

    # ========================================== 
    # 4️⃣ الخطوة الرابعة: المكان المحفوظ الثاني (نزول)
    # ========================================== 
    def go_to_saved_pose_2(self):
        self.get_logger().info("4️⃣ جاري التحرك للمكان المحفوظ الثاني...") 
        self.arm_client.wait_for_server() 
         
        joints_2 = [1.21765, 0.22525, -1.18212, -2.24339, 0.31316, 2.30478, 0.36614] 
        
        goal_msg = self.create_arm_goal(joints_2)
        self._send_goal_future = self.arm_client.send_goal_async(goal_msg) 
        self._send_goal_future.add_done_callback(self.arm_response_callback)

    # ========================================== 
    # 6️⃣ الخطوة السادسة: المكان المحفوظ الثالث (رفع بعد الإمساك)
    # ========================================== 
    def go_to_saved_pose_3(self):
        self.get_logger().info("6️⃣ جاري التحرك للمكان المحفوظ الثالث (الرفع)...") 
        self.arm_client.wait_for_server() 
         
        # 🟢 الزوايا الجديدة للمكان الثالث التي قمت بتسجيلها
        joints_3 = [0.72298, 0.18984, 2.95055, -1.84816, -0.04158, 1.82911, -2.26889] 
        
        goal_msg = self.create_arm_goal(joints_3)
        self._send_goal_future = self.arm_client.send_goal_async(goal_msg) 
        self._send_goal_future.add_done_callback(self.arm_response_callback)


    # ========================================== 
    # 🛠️ دالة مساعدة لتجهيز طلب الحركة
    # ========================================== 
    def create_arm_goal(self, angles):
        goal_msg = MoveGroup.Goal() 
        goal_msg.request.group_name = 'load_arm' 
        goal_msg.request.num_planning_attempts = 15 
        goal_msg.request.allowed_planning_time = 5.0 
        
        goal_msg.request.max_velocity_scaling_factor = 0.7 
        goal_msg.request.max_acceleration_scaling_factor = 0.7 
         
        joint_names = ['load_joint1', 'load_joint2', 'load_joint3', 'load_joint4', 'load_joint5', 'load_joint6', 'load_joint7'] 
        constraints = Constraints() 
        for i in range(7): 
            jc = JointConstraint() 
            jc.joint_name = joint_names[i] 
            jc.position = float(angles[i]) 
            jc.tolerance_above = 0.01 
            jc.tolerance_below = 0.01 
            jc.weight = 1.0 
            constraints.joint_constraints.append(jc) 

        goal_msg.request.goal_constraints.append(constraints)
        return goal_msg

    # ========================================== 
    # 🔄 إدارة الردود وتسلسل الحركات (State Machine)
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
                self.get_logger().info("✅ وصلنا لـ Home! 🚀 جاري التوجه للمكان الأول...") 
                self.current_state = "GO_SAVED_POSE_1" 
                self.go_to_saved_pose_1() 

            elif self.current_state == "GO_SAVED_POSE_1":
                self.get_logger().info("✅ وصلنا للمكان الأول! 👐 3️⃣ جاري فتح القابض...") 
                self.current_state = "OPENING" 
                self.operate_gripper(width=0.04, effort=5.0) 
                
            elif self.current_state == "GO_SAVED_POSE_2":
                self.get_logger().info("✅ وصلنا للمكان الثاني! ✊ 5️⃣ جاري إغلاق القابض...") 
                self.current_state = "CLOSING"
                self.operate_gripper(width=0.01, effort=20.0)
                
            elif self.current_state == "GO_SAVED_POSE_3":
                self.get_logger().info("✅🎉 تمت عملية الالتقاط بنجاح أسطوري! الذراع في الأعلى والقابض ممسك بالهدف.") 

        else: 
            self.get_logger().error(f"⚠️ فشلت الحركة في الحالة {self.current_state}! رمز الخطأ: {result.error_code.val}") 

    def gripper_response_callback(self, future): 
        goal_handle = future.result() 
        if not goal_handle.accepted: 
            self.get_logger().error("❌ تم رفض أمر القابض!") 
            return 
        self._gripper_result_future = goal_handle.get_result_async() 
        self._gripper_result_future.add_done_callback(self.gripper_result_callback) 

    def gripper_result_callback(self, future): 
        if self.current_state == "OPENING":
            self.get_logger().info("✅ القابض مفتوح! 🚀 4️⃣ جاري التوجه للمكان الثاني للالتقاط...") 
            self.current_state = "GO_SAVED_POSE_2"
            self.go_to_saved_pose_2()
            
        elif self.current_state == "CLOSING":
            self.get_logger().info("✅ القابض مغلق بإحكام! 🚀 6️⃣ جاري رفع الذراع للمكان الثالث...") 
            self.current_state = "GO_SAVED_POSE_3"
            self.go_to_saved_pose_3()

def main(args=None): 
    rclpy.init(args=args) 
    node = SimpleArmMover() 
    rclpy.spin(node) 
    node.destroy_node() 
    rclpy.shutdown() 

if __name__ == '__main__': 
    main()